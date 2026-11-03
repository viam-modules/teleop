package posemath

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestOrientationErrorIsAWorldFrameRotationVector(t *testing.T) {
	from := spatialmath.NewZeroOrientation()
	to := &spatialmath.EulerAngles{Yaw: math.Pi / 6} // 30 deg about world Z
	v := OrientationError(from, to)
	test.That(t, v.X, test.ShouldAlmostEqual, 0, 1e-9)
	test.That(t, v.Y, test.ShouldAlmostEqual, 0, 1e-9)
	test.That(t, v.Z, test.ShouldAlmostEqual, math.Pi/6, 1e-9)

	// The delta is in the WORLD frame: rotating `from` first does not change which axis the
	// extra yaw is reported about.
	tilted := &spatialmath.EulerAngles{Roll: 1.0}
	both := spatialmath.Compose(
		spatialmath.NewPoseFromOrientation(to),
		spatialmath.NewPoseFromOrientation(tilted),
	).Orientation()
	v = OrientationError(tilted, both)
	test.That(t, v.Z, test.ShouldAlmostEqual, math.Pi/6, 1e-9)
	test.That(t, v.Norm(), test.ShouldAlmostEqual, math.Pi/6, 1e-9)
}

func TestOrientationErrorTakesTheShortPath(t *testing.T) {
	from := spatialmath.NewZeroOrientation()
	to := &spatialmath.EulerAngles{Yaw: 350 * math.Pi / 180}
	v := OrientationError(from, to)
	test.That(t, v.Norm(), test.ShouldAlmostEqual, 10*math.Pi/180, 1e-9)
	test.That(t, v.Z, test.ShouldBeLessThan, 0)
}

func TestAngleBetweenDegAndPoseMap(t *testing.T) {
	a := spatialmath.NewPose(r3.Vector{X: 1.26}, spatialmath.NewZeroOrientation())
	b := spatialmath.NewPose(r3.Vector{}, &spatialmath.EulerAngles{Pitch: math.Pi / 4})
	test.That(t, AngleBetweenDeg(a, b), test.ShouldAlmostEqual, 45, 1e-9)

	m := PoseMap(a)
	test.That(t, m["x"], test.ShouldEqual, 1.3)
	test.That(t, m["theta"], test.ShouldEqual, 0.0)
	test.That(t, PoseMap(nil), test.ShouldBeNil)
}
