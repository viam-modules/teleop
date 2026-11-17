package clutch

import (
	"context"
	"errors"
	"math"
	"sync"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"teleop/internal/surface"
)

// tableWorld is a 20 mm cube tool centred on the follower frame, over a table whose top
// face is at z = 250. The table is 10 mm thick, like the one on the teleop-dev rig.
func tableWorld(t *testing.T) *surface.World {
	t.Helper()
	box := func(center, dims r3.Vector, label string) spatialmath.Geometry {
		g, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(center), dims, label)
		test.That(t, err, test.ShouldBeNil)
		return g
	}
	return &surface.World{
		Tool:      []spatialmath.Geometry{box(r3.Vector{}, r3.Vector{X: 20, Y: 20, Z: 20}, "tool")},
		Obstacles: []spatialmath.Geometry{box(r3.Vector{X: 500, Y: 100, Z: 245}, r3.Vector{X: 1000, Y: 1000, Z: 10}, "table")},
	}
}

func at(x, y, z float64) spatialmath.Pose {
	return spatialmath.NewPose(r3.Vector{X: x, Y: y, Z: z}, ov(0, 0, -1, 0))
}

func TestGuardHoldsTheSurfaceMeasuredFromOutside(t *testing.T) {
	g := newGuard(tableWorld(t))

	// Far above: nothing in range.
	test.That(t, g.update(at(500, 100, 500)), test.ShouldBeNil)
	test.That(t, g.project(at(500, 100, 500)).Point().Z, test.ShouldEqual, 500)

	// 40 mm of clearance (tool bottom at 290): the plane is where the tool would be 5 mm
	// off the table, which is the follower frame at z = 265, facing up.
	p := g.update(at(500, 100, 300))
	test.That(t, p, test.ShouldNotBeNil)
	test.That(t, p.label, test.ShouldEqual, "table")
	test.That(t, p.normal.Z, test.ShouldAlmostEqual, 1, 1e-6)
	test.That(t, p.point.Z, test.ShouldAlmostEqual, 265, 1e-6)
	test.That(t, g.status().Contact, test.ShouldBeFalse)

	// The hand goes 25 mm past it: the plane stays, the command is held on it, and moving
	// sideways is untouched.
	p = g.update(at(520, 130, 240))
	test.That(t, p.point.Z, test.ShouldAlmostEqual, 265, 1e-6)
	test.That(t, g.status().Contact, test.ShouldBeTrue)
	held := g.project(at(520, 130, 240))
	test.That(t, held.Point().Z, test.ShouldAlmostEqual, 265, 1e-6)
	test.That(t, held.Point().X, test.ShouldAlmostEqual, 520, 1e-6)
	test.That(t, held.Point().Y, test.ShouldAlmostEqual, 130, 1e-6)

	// Far past the thin table, out the other side: the plane still faces up. Measured from
	// in here, the nearest way out would be down.
	p = g.update(at(500, 100, 150))
	test.That(t, p.normal.Z, test.ShouldAlmostEqual, 1, 1e-6)
	test.That(t, g.project(at(500, 100, 150)).Point().Z, test.ShouldAlmostEqual, 265, 1e-6)
	test.That(t, g.status().Held, test.ShouldEqual, uint64(2))

	// Back out above the margin: free again, and a command above the plane is left alone.
	p = g.update(at(500, 100, 280))
	test.That(t, g.status().Contact, test.ShouldBeFalse)
	test.That(t, p.point.Z, test.ShouldAlmostEqual, 265, 1e-6)
	test.That(t, g.project(at(500, 100, 280)).Point().Z, test.ShouldEqual, 280)

	// Out of range: no plane.
	test.That(t, g.update(at(500, 100, 400)), test.ShouldBeNil)
	test.That(t, g.status().Obstacle, test.ShouldEqual, "")
}

func TestGuardStartingInsideAnObstacleDoesNothing(t *testing.T) {
	g := newGuard(tableWorld(t))
	// Never seen from outside: there is no plane to trust, so nothing is enforced.
	test.That(t, g.update(at(500, 100, 252)), test.ShouldBeNil)
	test.That(t, g.project(at(500, 100, 252)).Point().Z, test.ShouldEqual, 252)
	test.That(t, g.status().Contact, test.ShouldBeFalse)
}

func TestHandPlaneUndoesTheMapping(t *testing.T) {
	// Tracker frame yawed 90 degrees in world, scale 3.
	ref := NewReference(followerStart, hand0, yaw(90), trackerName)
	const scale = 3.0
	pointW, normalW := r3.Vector{X: 500, Y: 100, Z: 265}, r3.Vector{Z: 1}
	pointH, normalH := ref.HandPlane(pointW, normalW, scale)

	// World up is tracker up, and 35 mm down at the follower is 35/3 mm down at the hand.
	test.That(t, normalH.Z, test.ShouldAlmostEqual, 1, 1e-9)
	test.That(t, pointH.Z, test.ShouldAlmostEqual, 3-35.0/3, 1e-9)

	// For any hand position, its distance past the plane at the hand, times the scale, is
	// the target's distance past the plane in world.
	for _, hand := range []r3.Vector{{X: 1, Y: 2, Z: 3}, {X: 20, Y: -7, Z: -15}, {X: -4, Y: 9, Z: -40}} {
		target, ok := ref.Target(spatialmath.NewPose(hand, yaw(0)), scale, RotationOff, nil)
		test.That(t, ok, test.ShouldBeTrue)
		world := target.Point().Sub(pointW).Dot(normalW)
		atHand := hand.Sub(pointH).Dot(normalH)
		test.That(t, atHand*scale, test.ShouldAlmostEqual, world, 1e-9)
	}

	// A wall facing world -X is felt along the tracker's +Y (tracker +X is world +Y).
	_, n := ref.HandPlane(r3.Vector{X: 700}, r3.Vector{X: -1}, scale)
	test.That(t, n.Y, test.ShouldAlmostEqual, 1, 1e-9)
}

type recordingDevice struct {
	mu    sync.Mutex
	cmds  []map[string]interface{}
	block chan struct{}
}

func (d *recordingDevice) DoCommand(_ context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if d.block != nil {
		<-d.block
	}
	d.mu.Lock()
	defer d.mu.Unlock()
	d.cmds = append(d.cmds, cmd)
	return nil, nil
}

func (d *recordingDevice) got() []map[string]interface{} {
	d.mu.Lock()
	defer d.mu.Unlock()
	return append([]map[string]interface{}(nil), d.cmds...)
}

func TestHapticSenderKeepsOnlyTheNewestAndClears(t *testing.T) {
	dev := &recordingDevice{block: make(chan struct{})}
	s := &hapticSender{dev: dev, g: newGuard(tableWorld(t))}
	cmd := func(z float64) map[string]interface{} {
		return constraintCommand(r3.Vector{Z: z}, r3.Vector{Z: 1})
	}
	// The first goes out and blocks; the next three pile up and only the last survives.
	s.send(cmd(1))
	s.send(cmd(2))
	s.send(cmd(3))
	s.send(cmd(4))
	close(dev.block)
	waitFor(t, "the newest constraint", func() bool { return len(dev.got()) == 2 })
	s.clear()

	got := dev.got()
	test.That(t, len(got), test.ShouldEqual, 3)
	z := func(c map[string]interface{}) float64 { return c["point_mm"].(map[string]interface{})["z"].(float64) }
	test.That(t, z(got[0]), test.ShouldEqual, 1)
	test.That(t, z(got[1]), test.ShouldEqual, 4)
	test.That(t, got[2]["command"], test.ShouldEqual, "clear_constraint")

	c := got[0]
	test.That(t, c["command"], test.ShouldEqual, "constraint")
	test.That(t, c["type"], test.ShouldEqual, "plane")
	test.That(t, c["stiffness_n_per_m"], test.ShouldEqual, 400.0)
	test.That(t, c["max_force_n"], test.ShouldEqual, 2.5)
	test.That(t, c["hold_ms"], test.ShouldEqual, 300.0)
}

func TestTheFollowerSlidesOnASurfaceAndTheDeviceIsToldWhereItIs(t *testing.T) {
	r := newTrackerRig(t)
	r.start(t)
	dev := &recordingDevice{}
	g := newGuard(tableWorld(t))
	r.engine.mu.Lock()
	r.engine.guard = g
	r.engine.haptic = &hapticSender{dev: dev, g: g}
	r.engine.mu.Unlock()

	// A few ticks at the start pose, 40 mm clear of the table, measure the surface.
	waitFor(t, "the surface", func() bool { return r.engine.Status().Surface.Obstacle == "table" })
	waitFor(t, "a constraint at the device", func() bool { return len(dev.got()) > 0 })
	first := dev.got()[0]
	// The hand is at z = 3 with the follower at 300; the surface is at follower z = 265, so
	// 35 mm below the hand at scale 1, facing up.
	test.That(t, first["point_mm"].(map[string]interface{})["z"], test.ShouldAlmostEqual, 3-35.0, 1e-6)
	test.That(t, first["normal"].(map[string]interface{})["z"], test.ShouldAlmostEqual, 1, 1e-6)

	// The hand drops 60 mm, through the table: the follower stops on the surface.
	r.tracker.SetPose(body, spatialmath.NewPose(r3.Vector{X: 1, Y: 2, Z: 3 - 60}, yaw(0)))
	r.waitForPose(t, at(500, 100, 265))
	test.That(t, r.engine.Status().Surface.Contact, test.ShouldBeTrue)

	// Sliding the hand along the tracker's X moves the follower along world Y, still on
	// the surface.
	r.tracker.SetPose(body, spatialmath.NewPose(r3.Vector{X: 11, Y: 2, Z: 3 - 60}, yaw(0)))
	r.waitForPose(t, at(500, 110, 265))

	// Lifting the hand back frees it.
	r.tracker.SetPose(body, spatialmath.NewPose(r3.Vector{X: 11, Y: 2, Z: 3}, yaw(0)))
	r.waitForPose(t, at(500, 110, 300))
	test.That(t, r.engine.Status().Surface.Contact, test.ShouldBeFalse)
	test.That(t, r.engine.Status().Surface.Held, test.ShouldBeGreaterThan, uint64(0))

	// Stopping removes the constraint from the device.
	r.stop(t)
	got := dev.got()
	test.That(t, got[len(got)-1]["command"], test.ShouldEqual, "clear_constraint")
	_ = time.Now
}

// tilted is the tool at a position, rolled 45 degrees about world X. The 20 mm cube then
// reaches 10·√2 below its centre instead of 10.
func tilted(x, y, z float64) spatialmath.Pose {
	return spatialmath.NewPose(r3.Vector{X: x, Y: y, Z: z}, &spatialmath.EulerAngles{Roll: math.Pi / 4})
}

func TestGuardFollowsTheToolsOrientation(t *testing.T) {
	reach := 10 * math.Sqrt2 // the tilted cube's reach below its centre
	w := tableWorld(t)

	t.Run("turning while in contact lifts the surface", func(t *testing.T) {
		g := newGuard(w)
		g.update(at(500, 100, 300))
		g.update(at(500, 100, 240)) // through the table: held at 265
		test.That(t, g.project(at(500, 100, 240)).Point().Z, test.ShouldAlmostEqual, 265, 1e-6)

		// Still pushed through, now tilted: the corner would dig in, so the surface rises to
		// keep the corner 5 mm off the table top at z = 250.
		p := g.update(tilted(500, 100, 240))
		test.That(t, p.normal.Z, test.ShouldAlmostEqual, 1, 1e-6)
		test.That(t, p.point.Z, test.ShouldAlmostEqual, 250+5+reach, 1e-3)
		held := g.project(tilted(500, 100, 240))
		test.That(t, held.Point().Z, test.ShouldAlmostEqual, 250+5+reach, 1e-3)
		d, _, _ := w.Nearest(held)
		test.That(t, d, test.ShouldAlmostEqual, 5, 1e-3)

		// Turned back flat: the surface comes back down.
		p = g.update(at(500, 100, 240))
		test.That(t, p.point.Z, test.ShouldAlmostEqual, 265, 1e-3)
	})

	t.Run("turning into the obstacle from just above it makes contact", func(t *testing.T) {
		g := newGuard(w)
		// 1 mm above the surface, flat: free.
		g.update(at(500, 100, 266))
		test.That(t, g.status().Contact, test.ShouldBeFalse)
		// Tilt in place: the corner is now inside the margin though the position never moved.
		p := g.update(tilted(500, 100, 266))
		test.That(t, g.status().Contact, test.ShouldBeTrue)
		test.That(t, p.point.Z, test.ShouldAlmostEqual, 250+5+reach, 1e-3)
		test.That(t, g.project(tilted(500, 100, 266)).Point().Z, test.ShouldAlmostEqual, 250+5+reach, 1e-3)
	})

	t.Run("a command turned further than the hand is still kept out", func(t *testing.T) {
		g := newGuard(w)
		g.update(at(500, 100, 300))
		g.update(at(500, 100, 240)) // surface at 265 for a flat tool
		// The command arrives tilted while the hand's target is flat (the two can differ by
		// the smoothing lag): it is lifted to its own clearance.
		held := g.project(tilted(500, 100, 240))
		d, _, _ := w.Nearest(held)
		test.That(t, d, test.ShouldAlmostEqual, 5, 1e-3)
	})
}

type refusingDevice struct {
	mu    sync.Mutex
	calls int
}

func (d *refusingDevice) DoCommand(context.Context, map[string]interface{}) (map[string]interface{}, error) {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.calls++
	return nil, errors.New("unknown command")
}

func TestADeviceThatCannotRenderASurfaceIsLeftAlone(t *testing.T) {
	dev := &refusingDevice{}
	g := newGuard(tableWorld(t))
	s := &hapticSender{dev: dev, g: g}
	for i := 0; i < 50; i++ {
		s.send(constraintCommand(r3.Vector{}, r3.Vector{Z: 1}))
		time.Sleep(time.Millisecond)
	}
	s.clear()
	dev.mu.Lock()
	calls := dev.calls
	dev.mu.Unlock()
	// Five refusals, then nothing more: no constraint each tick, and no clear at the end.
	test.That(t, calls, test.ShouldEqual, deviceGiveUpAfter)
	st := g.status()
	test.That(t, st.Device, test.ShouldBeFalse)
	test.That(t, st.DeviceError, test.ShouldContainSubstring, "unknown command")
}
