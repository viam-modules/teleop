package surface

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

// world is the teleop-dev rig's table and one wall, and a gripper-sized tool whose frame
// origin is at the tool's tip (the box hangs behind it, as right-gripper's does).
func world(t *testing.T) *World {
	t.Helper()
	box := func(center r3.Vector, dims r3.Vector, label string) spatialmath.Geometry {
		g, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(center), dims, label)
		test.That(t, err, test.ShouldBeNil)
		return g
	}
	return &World{
		Tool: []spatialmath.Geometry{box(r3.Vector{Z: -55}, r3.Vector{X: 75, Y: 110, Z: 110}, "case")},
		Obstacles: []spatialmath.Geometry{
			box(r3.Vector{X: 289, Y: 546.5, Z: -5}, r3.Vector{X: 760, Y: 1218, Z: 10}, "table"),
			box(r3.Vector{X: 801, Y: 546.5, Z: 502.5}, r3.Vector{X: 10, Y: 1746, Z: 1005}, "front-wall"),
		},
	}
}

// down is a tool pointing straight down, the way a gripper approaches a table: the tool
// frame's +Z points to world -Z, so the box sits above the tip.
func down(x, y, z float64) spatialmath.Pose {
	return spatialmath.NewPose(r3.Vector{X: x, Y: y, Z: z}, &spatialmath.OrientationVectorDegrees{OZ: -1})
}

func TestNearestAndNormal(t *testing.T) {
	w := world(t)

	// Tip 50 mm above the table top (the table's top face is at z = 0).
	d, i, ok := w.Nearest(down(300, 500, 50))
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, w.Label(i), test.ShouldEqual, "table")
	test.That(t, d, test.ShouldAlmostEqual, 50, 0.5)
	n, ok := w.Normal(down(300, 500, 50), i)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, n.Z, test.ShouldAlmostEqual, 1, 1e-3)
	test.That(t, n.X, test.ShouldAlmostEqual, 0, 1e-3)

	// Tip 4 mm into the table: negative distance, and out is still up.
	d, i, ok = w.Nearest(down(300, 500, -4))
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, w.Label(i), test.ShouldEqual, "table")
	test.That(t, d, test.ShouldBeLessThan, 0)

	// Near the front wall, high above the table: the wall is nearest and out is -X.
	pose := down(750, 500, 400)
	d, i, ok = w.Nearest(pose)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, w.Label(i), test.ShouldEqual, "front-wall")
	test.That(t, d, test.ShouldBeGreaterThan, 0)
	n, ok = w.Normal(pose, i)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, n.X, test.ShouldAlmostEqual, -1, 1e-3)

	// No obstacles: nothing to report.
	empty := &World{Tool: w.Tool}
	_, _, ok = empty.Nearest(pose)
	test.That(t, ok, test.ShouldBeFalse)
	test.That(t, w.Label(99), test.ShouldEqual, "")
}
