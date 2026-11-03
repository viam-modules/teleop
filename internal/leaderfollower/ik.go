package leaderfollower

import (
	"context"
	"fmt"
	"math"
	"math/rand"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/mat"

	"teleop/internal/posemath"
)

// IKOptions tunes SolveBestEffort.
type IKOptions struct {
	// OrientationWeightMMPerRad converts orientation error to the same units as position
	// error: one radian of orientation error costs as much as this many millimetres. On a
	// 5-DoF leader the solver trades the two off, so this decides whether it prefers to be in
	// the right place or pointing the right way. Default 50.
	OrientationWeightMMPerRad float64
	// MaxIterations per restart. Default 200.
	MaxIterations int
	// Restarts is how many perturbed seeds are tried after the given one. Default 8.
	Restarts int
	// Damping is the Levenberg-Marquardt lambda, in millimetres. Default 2.
	Damping float64
	// MaxStepRad caps a single iteration's joint update. Default 0.25.
	MaxStepRad float64
	// Seed for the restart perturbations, so a solve is reproducible. Default 1.
	Seed int64
}

func (o IKOptions) withDefaults() IKOptions {
	if o.OrientationWeightMMPerRad <= 0 {
		o.OrientationWeightMMPerRad = 50
	}
	if o.MaxIterations <= 0 {
		o.MaxIterations = 200
	}
	if o.Restarts < 0 {
		o.Restarts = 0
	} else if o.Restarts == 0 {
		o.Restarts = 8
	}
	if o.Damping <= 0 {
		o.Damping = 2
	}
	if o.MaxStepRad <= 0 {
		o.MaxStepRad = 0.25
	}
	if o.Seed == 0 {
		o.Seed = 1
	}
	return o
}

// IKResult is the closest configuration SolveBestEffort found.
type IKResult struct {
	Inputs              []referenceframe.Input
	Pose                spatialmath.Pose // forward kinematics at Inputs
	PositionErrorMM     float64
	OrientationErrorDeg float64
	Cost                float64 // weighted squared error, mm²
}

// SolveBestEffort finds joint inputs that bring frame's pose as close as it can get to target,
// in the weighted sense of IKOptions, starting from seed. It never fails for being unable to
// reach the target: an unreachable pose returns the nearest reachable one with the residual
// reported. It is damped least squares on a numerical Jacobian with a handful of perturbed
// restarts, which is enough for a 5-DoF arm whose seed is already near the answer (the leader
// is normally parked a few hundred millimetres from where it is being sent).
//
// It is written here rather than borrowed from rdk's motionplan/ik because that solver is
// nlopt-backed and needs cgo, which the windows/amd64 build target of this module cannot have.
func SolveBestEffort(
	ctx context.Context,
	frame referenceframe.Frame,
	seed []referenceframe.Input,
	target spatialmath.Pose,
	opts IKOptions,
) (IKResult, error) {
	opts = opts.withDefaults()
	limits := frame.DoF()
	n := len(limits)
	if n == 0 {
		return IKResult{}, fmt.Errorf("frame has no degrees of freedom")
	}
	if len(seed) != n {
		return IKResult{}, fmt.Errorf("seed has %d inputs, frame has %d", len(seed), n)
	}

	rng := rand.New(rand.NewSource(opts.Seed)) //nolint:gosec // reproducibility, not security
	best := IKResult{Cost: math.Inf(1)}
	for restart := 0; restart <= opts.Restarts; restart++ {
		if err := ctx.Err(); err != nil {
			return best, err
		}
		start := make([]float64, n)
		copy(start, seed)
		if restart > 0 {
			// Perturbations grow with the restart index: the first few stay close to the
			// seed (a nearby local minimum), later ones roam.
			spread := 0.15 * float64(restart)
			for i := range start {
				start[i] = clampToLimit(start[i]+rng.NormFloat64()*spread, limits[i])
			}
		}
		res, err := solveFrom(frame, start, target, limits, opts)
		if err != nil {
			return best, err
		}
		if res.Cost < best.Cost {
			best = res
		}
		if best.Cost < 1e-6 {
			break
		}
	}
	return best, nil
}

func solveFrom(
	frame referenceframe.Frame,
	q []float64,
	target spatialmath.Pose,
	limits []referenceframe.Limit,
	opts IKOptions,
) (IKResult, error) {
	n := len(q)
	const h = 1e-5

	for j := range q {
		q[j] = clampToLimit(q[j], limits[j])
	}
	r, err := residualVec(frame, q, target, opts.OrientationWeightMMPerRad)
	if err != nil {
		return IKResult{}, err
	}
	cost := dot(r, r)

	jac := mat.NewDense(6, n, nil)
	jjt := mat.NewDense(6, 6, nil)
	rhs := mat.NewVecDense(6, nil)
	y := mat.NewVecDense(6, nil)
	qTrial := make([]float64, n)

	for iter := 0; iter < opts.MaxIterations && cost > 1e-8; iter++ {
		// Numerical Jacobian of f(q) = goal - r(q): J[:, j] = (r(q) - r(q + h e_j)) / h.
		for j := 0; j < n; j++ {
			copy(qTrial, q)
			qTrial[j] = clampToLimit(qTrial[j]+h, limits[j])
			rj, err := residualVec(frame, qTrial, target, opts.OrientationWeightMMPerRad)
			if err != nil {
				return IKResult{}, err
			}
			for i := 0; i < 6; i++ {
				jac.Set(i, j, (r[i]-rj[i])/h)
			}
		}
		// Damped least squares: dq = Jᵀ (J Jᵀ + λ² I)⁻¹ r.
		jjt.Mul(jac, jac.T())
		lambda2 := opts.Damping * opts.Damping
		for i := 0; i < 6; i++ {
			jjt.Set(i, i, jjt.At(i, i)+lambda2)
		}
		for i := 0; i < 6; i++ {
			rhs.SetVec(i, r[i])
		}
		if err := y.SolveVec(jjt, rhs); err != nil {
			// Singular even with damping: give up on this restart with what we have.
			break
		}
		dq := make([]float64, n)
		maxAbs := 0.0
		for j := 0; j < n; j++ {
			s := 0.0
			for i := 0; i < 6; i++ {
				s += jac.At(i, j) * y.AtVec(i)
			}
			dq[j] = s
			maxAbs = math.Max(maxAbs, math.Abs(s))
		}
		if maxAbs > opts.MaxStepRad {
			f := opts.MaxStepRad / maxAbs
			for j := range dq {
				dq[j] *= f
			}
		}

		// Backtracking line search so a bad linearization cannot make things worse.
		improved := false
		for step := 1.0; step > 1.0/64; step /= 2 {
			for j := 0; j < n; j++ {
				qTrial[j] = clampToLimit(q[j]+step*dq[j], limits[j])
			}
			rt, err := residualVec(frame, qTrial, target, opts.OrientationWeightMMPerRad)
			if err != nil {
				return IKResult{}, err
			}
			ct := dot(rt, rt)
			if ct < cost {
				copy(q, qTrial)
				r, cost = rt, ct
				improved = true
				break
			}
		}
		if !improved {
			break
		}
	}

	pose, err := frame.Transform(q)
	if err != nil {
		return IKResult{}, err
	}
	posErr := target.Point().Sub(pose.Point()).Norm()
	oriErr := posemath.OrientationError(pose.Orientation(), target.Orientation()).Norm()
	return IKResult{
		Inputs:              append([]referenceframe.Input(nil), q...),
		Pose:                pose,
		PositionErrorMM:     posErr,
		OrientationErrorDeg: oriErr * 180 / math.Pi,
		Cost:                cost,
	}, nil
}

// residualVec is [target.pos - fk.pos (mm); w * rotation vector from fk.ori to target.ori (rad)].
func residualVec(frame referenceframe.Frame, q []float64, target spatialmath.Pose, w float64) ([6]float64, error) {
	pose, err := frame.Transform(q)
	if err != nil {
		return [6]float64{}, err
	}
	dp := target.Point().Sub(pose.Point())
	dr := posemath.OrientationError(pose.Orientation(), target.Orientation())
	return [6]float64{dp.X, dp.Y, dp.Z, w * dr.X, w * dr.Y, w * dr.Z}, nil
}

func clampToLimit(v float64, l referenceframe.Limit) float64 {
	if v < l.Min {
		return l.Min
	}
	if v > l.Max {
		return l.Max
	}
	return v
}

func dot(a, b [6]float64) float64 {
	s := 0.0
	for i := range a {
		s += a[i] * b[i]
	}
	return s
}
