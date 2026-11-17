package surface

import (
	"context"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

type fakeReader struct {
	cfg *framesystem.Config
}

func (f *fakeReader) FrameSystemConfig(context.Context) (*framesystem.Config, error) {
	return f.cfg, nil
}

const armJSON = `{
  "name": "arm",
  "links": [{"id": "base", "parent": "world", "translation": {"x": 0, "y": 0, "z": 100},
             "geometry": {"type": "box", "x": 50, "y": 50, "z": 100, "translation": {"x": 0, "y": 0, "z": 50}}}],
  "joints": [{"id": "j", "type": "revolute", "parent": "base", "axis": {"x": 0, "y": 0, "z": 1}, "max": 180, "min": -180}]
}`

// A frame system shaped like the rig's: a fixed table, an arm with a joint, a gripper on
// the arm, and a second gripper on the same arm standing in for anything else that moves.
func TestLoad(t *testing.T) {
	box := func(center, dims r3.Vector, label string) spatialmath.Geometry {
		g, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(center), dims, label)
		test.That(t, err, test.ShouldBeNil)
		return g
	}
	armModel, err := referenceframe.UnmarshalModelJSON([]byte(armJSON), "arm")
	test.That(t, err, test.ShouldBeNil)

	at := func(x, y, z float64) spatialmath.Pose {
		return spatialmath.NewPoseFromPoint(r3.Vector{X: x, Y: y, Z: z})
	}
	parts := []*referenceframe.FrameSystemPart{
		{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, at(289, 546.5, -5), "table",
			box(r3.Vector{}, r3.Vector{X: 760, Y: 1218, Z: 10}, "table"))},
		{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, at(0, 0, 0), "arm", nil), ModelFrame: armModel},
		{FrameConfig: referenceframe.NewLinkInFrame("arm", at(0, 0, 105), "gripper",
			box(r3.Vector{Z: -55}, r3.Vector{X: 75, Y: 110, Z: 110}, "case"))},
		{FrameConfig: referenceframe.NewLinkInFrame("arm", at(200, 0, 0), "other-tool",
			box(r3.Vector{}, r3.Vector{X: 10, Y: 10, Z: 10}, "other"))},
	}
	reader := &fakeReader{cfg: &framesystem.Config{Parts: parts}}
	// The arm's link ends 100 mm up and the gripper frame is 105 mm past that.
	gripperWorld := at(0, 0, 205)

	w, err := Load(context.Background(), reader, "gripper")
	test.That(t, err, test.ShouldBeNil)

	// The tool is the gripper's box, back in the gripper's own frame.
	test.That(t, len(w.Tool), test.ShouldEqual, 1)
	test.That(t, w.Tool[0].Pose().Point().Z, test.ShouldAlmostEqual, -55, 1e-6)
	test.That(t, w.Tool[0].Pose().Point().X, test.ShouldAlmostEqual, 0, 1e-6)

	// Only the table is an obstacle: the arm's own link and the tool on the arm move.
	labels := []string{}
	for i := range w.Obstacles {
		labels = append(labels, w.Label(i))
	}
	test.That(t, labels, test.ShouldResemble, []string{"table"})

	// The gripper box spans z = 40..150 in world (frame at 205, box 110 tall centred 55
	// below... the frame); the table top is at z = 0, off to the side in x and y.
	d, i, ok := w.Nearest(gripperWorld)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, w.Label(i), test.ShouldEqual, "table")
	test.That(t, d, test.ShouldBeGreaterThan, 0)

	// A follower with no geometry cannot be guarded.
	_, err = Load(context.Background(), reader, "nope")
	test.That(t, err, test.ShouldNotBeNil)
}
