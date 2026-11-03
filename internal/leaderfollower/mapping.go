// Package leaderfollower mirrors a leader arm's end-effector pose onto a follower of
// different kinematics through viam-server's builtin motion service teleop pipeline.
//
// The leader is read through the standard arm API (EndPosition, i.e. forward kinematics on a
// position read); the follower is driven through the motion service's teleop_start /
// teleop_move / teleop_stop DoCommands, the same pipeline the vive module and the mobile
// app's Arm Controller use. Nothing here imports a component or service package, so the
// same Engine runs inside the module (services/leaderfollower) and in a standalone client
// (cmd/teleop-client) over gRPC.
package leaderfollower

import (
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"
)

// Mapping relates the leader's end-effector pose, in the leader's base frame as EndPosition
// reports it, to the follower's target pose in the follower machine's world frame:
//
//	follower = Origin ∘ scale(leader ∘ LeaderToolOffset) ∘ ToolRotation
//
// Origin places the leader's base in the follower's world. LeaderToolOffset moves the point
// being mirrored from the leader's reported end-effector frame out to its physical tip, in the
// tool frame, BEFORE scaling: a leader whose EndPosition is a wrist frame some distance behind
// the handle tip pivots the follower about a point scale×that distance off its tip unless the
// offset is set. Scale multiplies the leader's (offset) translation about its own base origin,
// so a small desktop leader can span a large follower workspace. ToolRotation is a constant
// rotation applied in the tool frame that aligns the two arms' tool-frame conventions.
//
// There is deliberately no clutch and no per-session reference: the mapping is absolute, which
// is what lets Engine.Start drive the leader to the follower's pose (Inverse) before mirroring.
type Mapping struct {
	Origin           spatialmath.Pose
	Scale            float64
	LeaderToolOffset r3.Vector
	ToolRotation     spatialmath.Orientation
}

func (m Mapping) origin() spatialmath.Pose {
	if m.Origin == nil {
		return spatialmath.NewZeroPose()
	}
	return m.Origin
}

func (m Mapping) scale() float64 {
	if m.Scale <= 0 {
		return 1
	}
	return m.Scale
}

func (m Mapping) toolRotation() spatialmath.Pose {
	if m.ToolRotation == nil {
		return spatialmath.NewZeroPose()
	}
	return spatialmath.NewPoseFromOrientation(m.ToolRotation)
}

func (m Mapping) leaderToolOffset() spatialmath.Pose {
	return spatialmath.NewPoseFromPoint(m.LeaderToolOffset)
}

// Forward maps a leader end-effector pose to the follower target it corresponds to.
func (m Mapping) Forward(leader spatialmath.Pose) spatialmath.Pose {
	tip := spatialmath.Compose(leader, m.leaderToolOffset())
	scaled := spatialmath.NewPose(tip.Point().Mul(m.scale()), tip.Orientation())
	return spatialmath.Compose(spatialmath.Compose(m.origin(), scaled), m.toolRotation())
}

// Inverse maps a follower pose back to the leader end-effector pose that Forward would send
// there. The result is a kinematic *request*: a 5-DoF leader usually cannot reach it exactly,
// which is what SolveBestEffort is for.
func (m Mapping) Inverse(follower spatialmath.Pose) spatialmath.Pose {
	// follower = O ∘ S(l ∘ D) ∘ T   =>   l = S⁻¹(O⁻¹ ∘ follower ∘ T⁻¹) ∘ D⁻¹
	p := spatialmath.Compose(spatialmath.PoseInverse(m.origin()), follower)
	p = spatialmath.Compose(p, spatialmath.PoseInverse(m.toolRotation()))
	tip := spatialmath.NewPose(p.Point().Mul(1/m.scale()), p.Orientation())
	return spatialmath.Compose(tip, spatialmath.PoseInverse(m.leaderToolOffset()))
}

// Residual is the constant correction that makes the mapped leader pose coincide with the
// follower's actual pose after an imperfect alignment. Translation is applied in the follower's
// world frame; Rotation is applied in the tool frame (right-multiplied), so it behaves like a
// fixed tool offset rather than a world-frame tilt.
//
// It exists because Engine.Start moves the leader *as close as it can* to the follower's pose,
// and "as close as it can" on a 5-DoF arm is not zero: without the residual the first
// teleop_move would ask the follower to close that gap, i.e. the follower would move at start.
type Residual struct {
	Translation r3.Vector
	Rotation    spatialmath.Orientation
}

// ResidualBetween returns the Residual r such that r.Apply(mapped) == actual.
func ResidualBetween(mapped, actual spatialmath.Pose) Residual {
	// Apply right-multiplies, so the rotation is expressed in mapped's own frame:
	// mapped ∘ R = actual  =>  R = mapped⁻¹ ∘ actual. (spatialmath.OrientationBetween is the
	// world-frame delta, actual ∘ mapped⁻¹, which is the wrong side for a tool offset.)
	q := spatialmath.Quaternion(quat.Mul(quat.Conj(mapped.Orientation().Quaternion()), actual.Orientation().Quaternion()))
	return Residual{
		Translation: actual.Point().Sub(mapped.Point()),
		Rotation:    &q,
	}
}

// Apply returns p corrected by the residual.
func (r Residual) Apply(p spatialmath.Pose) spatialmath.Pose {
	ori := p.Orientation()
	if r.Rotation != nil {
		ori = spatialmath.Compose(
			spatialmath.NewPoseFromOrientation(ori),
			spatialmath.NewPoseFromOrientation(r.Rotation),
		).Orientation()
	}
	return spatialmath.NewPose(p.Point().Add(r.Translation), ori)
}

// IsZero reports whether the residual is the identity.
func (r Residual) IsZero() bool {
	return r.Translation.Norm() == 0 && (r.Rotation == nil ||
		spatialmath.OrientationAlmostEqual(r.Rotation, spatialmath.NewZeroOrientation()))
}
