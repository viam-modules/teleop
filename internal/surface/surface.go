// Package surface answers one question for a teleop loop: how close is the follower's tool
// to the nearest fixed obstacle, and which way is out. It uses the frame system's own
// geometry and rdk's own distance function, so it sees the same shapes the motion planner
// does. It only checks the tool against obstacles that never move; the planner still checks
// everything else.
package surface

import (
	"context"
	"fmt"
	"math"
	"strings"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
)

// originSuffix is what rdk appends to a component's name for the frame its frame block
// defines.
const originSuffix = "_origin"

// Reader is the slice of the frame system service Load needs. framesystem.Service
// satisfies it.
type Reader interface {
	FrameSystemConfig(ctx context.Context) (*framesystem.Config, error)
}

// World is the tool's shape and the fixed obstacles around it.
type World struct {
	// Tool is the follower component's geometry in the follower component's own frame.
	Tool []spatialmath.Geometry
	// Obstacles are the geometries of frames that cannot move, in world.
	Obstacles []spatialmath.Geometry
}

// Load reads the frame system's configuration once. It needs no joint positions: the
// obstacles it keeps are the ones no joint can move, and the tool's shape in its own frame
// is the same whatever the arm is doing. (A module cannot read current inputs through the
// frame system service in any case.)
func Load(ctx context.Context, r Reader, follower string) (*World, error) {
	cfg, err := r.FrameSystemConfig(ctx)
	if err != nil {
		return nil, fmt.Errorf("read frame system config: %w", err)
	}
	fs, err := referenceframe.NewFrameSystem("teleop-surfaces", cfg.Parts, nil)
	if err != nil {
		return nil, fmt.Errorf("build frame system: %w", err)
	}
	// Every joint at zero: wrong for the arms, which are not used, and irrelevant to
	// everything that is.
	zero := referenceframe.NewZeroInputs(fs)
	// Geometries come back in world. A partial result with an error still has the frames
	// that could be placed, which is all that is needed.
	geoms, gerr := referenceframe.FrameSystemGeometries(fs, zero)
	if len(geoms) == 0 {
		return nil, fmt.Errorf("no geometry in the frame system: %v", gerr)
	}
	if fs.Frame(follower) == nil {
		return nil, fmt.Errorf("%q is not in the frame system", follower)
	}
	tf, err := fs.Transform(zero.ToLinearInputs(),
		referenceframe.NewPoseInFrame(follower, spatialmath.NewZeroPose()), referenceframe.World)
	if err != nil {
		return nil, fmt.Errorf("place %q: %w", follower, err)
	}
	followerPose := tf.(*referenceframe.PoseInFrame).Pose()

	w := &World{}
	toTool := spatialmath.PoseInverse(followerPose)
	for name, gif := range geoms {
		if gif == nil {
			continue
		}
		// A component's geometry is under its own name when it comes from the component's
		// model, and under "<name>_origin" when it comes from the frame block.
		component := strings.TrimSuffix(name, originSuffix)
		if component == follower {
			for _, g := range gif.Geometries() {
				w.Tool = append(w.Tool, g.Transform(toTool))
			}
			continue
		}
		if !fixed(fs, name, follower) {
			continue
		}
		for _, g := range gif.Geometries() {
			if g.Label() == "" {
				g.SetLabel(component)
			}
			w.Obstacles = append(w.Obstacles, g)
		}
	}
	if len(w.Tool) == 0 {
		return nil, fmt.Errorf("%q has no geometry in the frame system", follower)
	}
	return w, nil
}

// fixed reports whether a frame and every frame above it has no joints, so it never moves,
// and is not attached to the follower.
func fixed(fs *referenceframe.FrameSystem, name, follower string) bool {
	f := fs.Frame(name)
	for depth := 0; f != nil && depth < 64; depth++ {
		if strings.TrimSuffix(f.Name(), originSuffix) == follower || len(f.DoF()) > 0 {
			return false
		}
		if f.Name() == referenceframe.World {
			return true
		}
		parent, err := fs.Parent(f)
		if err != nil || parent == nil {
			return true
		}
		f = parent
	}
	return false
}

// Nearest returns the signed distance in millimetres from the tool, placed at pose, to the
// closest obstacle, and that obstacle's index. Negative means they overlap by that much.
// ok is false when there are no obstacles or the distance cannot be computed.
func (w *World) Nearest(pose spatialmath.Pose) (dist float64, obstacle int, ok bool) {
	dist, obstacle = math.Inf(1), -1
	for _, t := range w.Tool {
		placed := t.Transform(pose)
		for i, o := range w.Obstacles {
			d, err := placed.DistanceFrom(o)
			if err != nil || math.IsNaN(d) {
				continue
			}
			if d < dist {
				dist, obstacle = d, i
			}
		}
	}
	return dist, obstacle, obstacle >= 0
}

// Distance is the signed distance in millimetres from the tool at pose to one obstacle.
func (w *World) Distance(pose spatialmath.Pose, obstacle int) (float64, bool) {
	best, ok := math.Inf(1), false
	for _, t := range w.Tool {
		d, err := t.Transform(pose).DistanceFrom(w.Obstacles[obstacle])
		if err != nil || math.IsNaN(d) {
			continue
		}
		if d < best {
			best, ok = d, true
		}
	}
	return best, ok
}

// Normal is the unit direction in world in which moving the tool (without turning it)
// increases its distance from the obstacle fastest: the way out. It is measured by nudging
// the tool a millimetre each way along each axis. ok is false when the distance does not
// change, which leaves no direction to report.
func (w *World) Normal(pose spatialmath.Pose, obstacle int) (r3.Vector, bool) {
	const step = 1.0
	at := func(offset r3.Vector) (float64, bool) {
		moved := spatialmath.NewPose(pose.Point().Add(offset), pose.Orientation())
		return w.Distance(moved, obstacle)
	}
	var grad [3]float64
	for i, axis := range []r3.Vector{{X: step}, {Y: step}, {Z: step}} {
		plus, ok1 := at(axis)
		minus, ok2 := at(axis.Mul(-1))
		if !ok1 || !ok2 {
			return r3.Vector{}, false
		}
		grad[i] = (plus - minus) / (2 * step)
	}
	n := r3.Vector{X: grad[0], Y: grad[1], Z: grad[2]}
	if n.Norm() < 1e-6 {
		return r3.Vector{}, false
	}
	return n.Normalize(), true
}

// Label names an obstacle for logs and status.
func (w *World) Label(obstacle int) string {
	if obstacle < 0 || obstacle >= len(w.Obstacles) {
		return ""
	}
	return strings.TrimSuffix(w.Obstacles[obstacle].Label(), originSuffix)
}
