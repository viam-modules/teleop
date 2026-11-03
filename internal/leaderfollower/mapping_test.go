package leaderfollower

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"teleop/internal/posemath"
)

func ov(x, y, z, th float64) spatialmath.Orientation {
	return &spatialmath.OrientationVectorDegrees{OX: x, OY: y, OZ: z, Theta: th}
}

func poseAlmostEqual(t *testing.T, a, b spatialmath.Pose, mm, deg float64) {
	t.Helper()
	test.That(t, a.Point().Sub(b.Point()).Norm(), test.ShouldBeLessThan, mm)
	test.That(t, posemath.AngleBetweenDeg(a, b), test.ShouldBeLessThan, deg)
}

func TestForwardAppliesOriginScaleToolOffsetAndToolRotation(t *testing.T) {
	m := Mapping{
		Origin:           spatialmath.NewPose(r3.Vector{X: 1000}, ov(0, 0, 1, 90)),
		Scale:            2,
		LeaderToolOffset: r3.Vector{Z: 100},
		ToolRotation:     ov(0, 0, 1, 180),
	}
	// Leader at (100, 0, 0) pointing along +X: the tool offset (tool-frame +Z) moves the tip
	// to (100, 0, 100); scale doubles it to (200, 0, 200); the origin yaws 90 deg and shifts
	// x by 1000: (1000, 200, 200).
	leader := spatialmath.NewPose(r3.Vector{X: 100}, spatialmath.NewZeroOrientation())
	got := m.Forward(leader)
	test.That(t, got.Point().X, test.ShouldAlmostEqual, 1000, 1e-6)
	test.That(t, got.Point().Y, test.ShouldAlmostEqual, 200, 1e-6)
	test.That(t, got.Point().Z, test.ShouldAlmostEqual, 200, 1e-6)
}

func TestInverseRoundTripsForward(t *testing.T) {
	m := Mapping{
		Origin:           spatialmath.NewPose(r3.Vector{X: 300, Y: -50, Z: 20}, ov(0.3, 0.1, 1, 35)),
		Scale:            2.5,
		LeaderToolOffset: r3.Vector{Z: 96},
		ToolRotation:     ov(1, 0, 0, 90),
	}
	leader := spatialmath.NewPose(r3.Vector{X: 150, Y: 40, Z: 120}, ov(0.2, -0.4, 0.8, 17))
	poseAlmostEqual(t, m.Inverse(m.Forward(leader)), leader, 1e-6, 1e-6)
	// Zero-value mapping is the identity.
	poseAlmostEqual(t, Mapping{}.Forward(leader), leader, 1e-9, 1e-9)
}

func TestResidualMakesTheMappedPoseCoincideWithTheActualOne(t *testing.T) {
	// A non-identity mapped orientation, because the tool-frame and world-frame conventions
	// agree at identity and the test would not tell them apart.
	mapped := spatialmath.NewPose(r3.Vector{X: 100, Y: 20, Z: 5}, ov(0, 1, 0, 40))
	actual := spatialmath.NewPose(r3.Vector{X: 110, Y: 25, Z: -5}, ov(0.5, 0.5, 0.7, 70))
	r := ResidualBetween(mapped, actual)
	poseAlmostEqual(t, r.Apply(mapped), actual, 1e-6, 1e-6)
	test.That(t, r.IsZero(), test.ShouldBeFalse)
	test.That(t, ResidualBetween(actual, actual).IsZero(), test.ShouldBeTrue)

	// The rotation is a TOOL-frame correction: applying it to a differently oriented pose
	// does not rotate it by the same world-frame delta.
	other := spatialmath.NewPose(r3.Vector{}, ov(1, 0, 0, 90))
	worldDelta := posemath.OrientationError(mapped.Orientation(), actual.Orientation())
	appliedDelta := posemath.OrientationError(other.Orientation(), r.Apply(other).Orientation())
	test.That(t, appliedDelta.Norm(), test.ShouldAlmostEqual, worldDelta.Norm(), 1e-6) // same angle
	test.That(t, appliedDelta.Sub(worldDelta).Norm(), test.ShouldBeGreaterThan, 0.1)   // different axis
	_ = math.Pi
}
