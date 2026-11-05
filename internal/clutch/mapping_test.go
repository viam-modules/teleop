package clutch

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func ov(x, y, z, th float64) spatialmath.Orientation {
	return &spatialmath.OrientationVectorDegrees{OX: x, OY: y, OZ: z, Theta: th}
}

func yaw(deg float64) spatialmath.Orientation {
	return &spatialmath.EulerAngles{Yaw: deg * math.Pi / 180}
}

func poseAlmostEqual(t *testing.T, got, want spatialmath.Pose, posTol, oriTolDeg float64) {
	t.Helper()
	test.That(t, got, test.ShouldNotBeNil)
	d := got.Point().Sub(want.Point()).Norm()
	test.That(t, d, test.ShouldBeLessThanOrEqualTo, posTol)
	q := spatialmath.QuatToR4AA(spatialmath.OrientationBetween(got.Orientation(), want.Orientation()).Quaternion())
	test.That(t, math.Abs(q.Theta)*180/math.Pi, test.ShouldBeLessThanOrEqualTo, oriTolDeg)
}

func TestTargetTranslatesTheHandDeltaThroughTheSourceRotationAndScale(t *testing.T) {
	follower := spatialmath.NewPose(r3.Vector{X: 500, Y: 100, Z: 300}, ov(0, 0, -1, 0))
	hand0 := spatialmath.NewPose(r3.Vector{X: 1, Y: 2, Z: 3}, yaw(0))
	// The tracker's frame is yawed 90 deg in world: tracker +X is world +Y.
	ref := NewReference(follower, hand0, yaw(90), "vive")

	hand := spatialmath.NewPose(r3.Vector{X: 11, Y: 2, Z: 3}, yaw(0))
	got, ok := ref.Target(hand, 2, RotationOff, nil)
	test.That(t, ok, test.ShouldBeTrue)
	poseAlmostEqual(t, got, spatialmath.NewPose(r3.Vector{X: 500, Y: 120, Z: 300}, ov(0, 0, -1, 0)), 1e-9, 1e-9)
}

func TestAtEngageEveryModeButAbsoluteSendsTheFollowersOwnPose(t *testing.T) {
	follower := spatialmath.NewPose(r3.Vector{X: 500}, ov(0, 1, 0, 30))
	hand := spatialmath.NewPose(r3.Vector{X: 1}, ov(1, 0, 0, 45)) // held any old way
	ref := NewReference(follower, hand, yaw(90), "vive")

	for _, mode := range []RotationMode{RotationRelative, RotationOff} {
		got, ok := ref.Target(hand, 1, mode, nil)
		test.That(t, ok, test.ShouldBeTrue)
		poseAlmostEqual(t, got, follower, 1e-9, 1e-9)
	}
	// Absolute turns the tool to where the hand points (in world), composed with the tool
	// rotation: the vive wand's grip correction lives there.
	tool := yaw(70)
	got, ok := ref.Target(hand, 1, RotationAbsolute, tool)
	test.That(t, ok, test.ShouldBeTrue)
	want := spatialmath.Compose(spatialmath.Compose(
		spatialmath.NewPoseFromOrientation(yaw(90)), spatialmath.NewPoseFromOrientation(hand.Orientation())),
		spatialmath.NewPoseFromOrientation(tool))
	poseAlmostEqual(t, got, spatialmath.NewPose(r3.Vector{X: 500}, want.Orientation()), 1e-9, 1e-9)
}

func TestRelativeTurnsTheFollowerByHowMuchTheHandTurnedInWorld(t *testing.T) {
	follower := spatialmath.NewPose(r3.Vector{X: 500}, ov(0, 1, 0, 30))
	hand0 := spatialmath.NewPose(r3.Vector{}, ov(1, 0, 0, 45))
	ref := NewReference(follower, hand0, nil, "vive") // source frame is world

	// Turn the hand 20 deg about world Z (left-multiply, a world-frame rotation).
	hand := spatialmath.Compose(spatialmath.NewPoseFromOrientation(yaw(20)), hand0)
	got, ok := ref.Target(hand, 1, RotationRelative, nil)
	test.That(t, ok, test.ShouldBeTrue)
	want := spatialmath.Compose(spatialmath.NewPoseFromOrientation(yaw(20)), spatialmath.NewPoseFromOrientation(follower.Orientation()))
	poseAlmostEqual(t, got, spatialmath.NewPose(r3.Vector{X: 500}, want.Orientation()), 1e-9, 1e-9)
}

func TestNaNInputIsSkipped(t *testing.T) {
	follower := spatialmath.NewPose(r3.Vector{X: 500}, ov(0, 0, 1, 0))
	ref := NewReference(follower, spatialmath.NewZeroPose(), nil, "f")
	_, ok := ref.Target(spatialmath.NewPose(r3.Vector{X: math.NaN()}, ov(0, 0, 1, 0)), 1, RotationOff, nil)
	test.That(t, ok, test.ShouldBeFalse)
}
