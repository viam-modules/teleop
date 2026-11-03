// Package posemath holds the small pose helpers every teleop engine needs and no rdk package
// exports in quite this form.
package posemath

import (
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"
)

// OrientationError returns the world-frame rotation vector (axis * angle, radians) that takes
// `from` to `to`: R_err = R_to * R_fromᵀ. This is the world-frame delta, which is what a
// filter, a status line and an IK residual want. It is NOT the tool-frame delta a residual
// offset wants; see leaderfollower.ResidualBetween for that side.
func OrientationError(from, to spatialmath.Orientation) r3.Vector {
	qe := quat.Mul(to.Quaternion(), quat.Conj(from.Quaternion()))
	if qe.Real < 0 { // shortest path
		qe = quat.Scale(-1, qe)
	}
	aa := spatialmath.QuatToR4AA(qe)
	return r3.Vector{X: aa.RX, Y: aa.RY, Z: aa.RZ}.Mul(aa.Theta)
}

// AngleBetweenDeg is the magnitude of OrientationError between two poses, in degrees.
func AngleBetweenDeg(a, b spatialmath.Pose) float64 {
	return OrientationError(a.Orientation(), b.Orientation()).Norm() * 180 / math.Pi
}

// Round1 rounds to one decimal, for log lines and status maps.
func Round1(v float64) float64 { return math.Round(v*10) / 10 }

// PoseMap renders p as {x, y, z, ox, oy, oz, theta}, rounded to 0.1 mm / 0.1 deg, using only
// structpb-native types so the same value fits a structured log line and a DoCommand result.
// nil → nil.
func PoseMap(p spatialmath.Pose) map[string]interface{} {
	if p == nil {
		return nil
	}
	pt := p.Point()
	ov := p.Orientation().OrientationVectorDegrees()
	return map[string]interface{}{
		"x": Round1(pt.X), "y": Round1(pt.Y), "z": Round1(pt.Z),
		"ox": Round1(ov.OX), "oy": Round1(ov.OY), "oz": Round1(ov.OZ), "theta": Round1(ov.Theta),
	}
}
