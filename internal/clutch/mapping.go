package clutch

import (
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"
)

// Reference is what engage captured: where the follower was, where the hand was, and the
// rotation that takes the hand's frame into the follower's world.
type Reference struct {
	FollowerPose spatialmath.Pose        // world
	Hand         spatialmath.Pose        // source frame
	SourceRot    spatialmath.Orientation // source frame -> world; nil = identity
	Frame        string

	handWorldOri quat.Number // SourceRot ∘ Hand.Orientation
}

// NewReference captures a reference.
func NewReference(follower, hand spatialmath.Pose, sourceRot spatialmath.Orientation, frame string) *Reference {
	r := &Reference{FollowerPose: follower, Hand: hand, SourceRot: sourceRot, Frame: frame}
	r.handWorldOri = quat.Mul(r.sourceQuat(), hand.Orientation().Quaternion())
	return r
}

func (r *Reference) sourceQuat() quat.Number {
	if r.SourceRot == nil {
		return quat.Number{Real: 1}
	}
	return r.SourceRot.Quaternion()
}

// Target maps the hand's current pose (in the source frame) to the follower's target in
// world:
//
//	position = follower_ref + scale × R_source × (hand − hand_ref)
//	absolute: orientation = R_source ∘ hand_rot ∘ tool
//	relative: orientation = R_source ∘ hand_rot ∘ (hand_ref_world_rot⁻¹ ∘ follower_ref_rot)
//	off:      orientation = follower_ref_rot
//
// It returns false when the input produced a NaN (a tracker glitch), which the caller skips.
func (r *Reference) Target(hand spatialmath.Pose, scale float64, mode RotationMode, tool spatialmath.Orientation) (spatialmath.Pose, bool) {
	src := r.sourceQuat()
	delta := rotate(src, hand.Point().Sub(r.Hand.Point()))
	pt := r.FollowerPose.Point().Add(delta.Mul(scale))

	handWorld := quat.Mul(src, hand.Orientation().Quaternion())
	var ori quat.Number
	switch mode {
	case RotationRelative:
		offset := quat.Mul(quat.Conj(r.handWorldOri), r.FollowerPose.Orientation().Quaternion())
		ori = quat.Mul(handWorld, offset)
	case RotationOff:
		ori = r.FollowerPose.Orientation().Quaternion()
	default: // absolute
		ori = handWorld
		if tool != nil {
			ori = quat.Mul(ori, tool.Quaternion())
		}
	}
	if math.IsNaN(pt.X) || math.IsNaN(pt.Y) || math.IsNaN(pt.Z) || math.IsNaN(ori.Real) || math.IsNaN(ori.Imag) {
		return nil, false
	}
	n := quat.Abs(ori)
	if n == 0 {
		return nil, false
	}
	ori = quat.Scale(1/n, ori)
	q := spatialmath.Quaternion(ori)
	return spatialmath.NewPose(pt, &q), true
}

// rotate applies a unit quaternion to a vector.
func rotate(q quat.Number, v r3.Vector) r3.Vector {
	p := quat.Number{Imag: v.X, Jmag: v.Y, Kmag: v.Z}
	out := quat.Mul(quat.Mul(q, p), quat.Conj(q))
	return r3.Vector{X: out.Imag, Y: out.Jmag, Z: out.Kmag}
}
