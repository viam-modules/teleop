package clutch

import (
	"context"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"

	"teleop/internal/surface"
)

// Haptic surface defaults.
const (
	DefaultHapticMarginMM  = 5.0   // the felt surface sits this far outside the real one
	DefaultHapticRangeMM   = 80.0  // obstacles nearer than this are sent to the device
	DefaultHapticStiffness = 400.0 // N/m at the hand
	DefaultHapticMaxForceN = 2.5
	hapticHold             = 300 * time.Millisecond // each constraint lasts this long; renewed every tick
	hapticCommandTimeout   = 200 * time.Millisecond
	hapticClearTimeout     = time.Second
	// projectCheckMM: a command within this distance of the surface has its tool checked at
	// the command's own orientation.
	projectCheckMM = 60.0
	// projectToleranceMM: how far inside the margin the tool may be before it is moved.
	projectToleranceMM = 0.5
)

// HapticsConfig turns on surfaces: the follower's target is kept out of fixed obstacles, and
// the hand device is told where that surface is so the operator feels it.
type HapticsConfig struct {
	Enabled        bool
	MarginMM       float64
	RangeMM        float64
	StiffnessNPerM float64
	MaxForceN      float64
}

// plane is a half-space the follower's target position must stay in, in world.
type plane struct {
	point    r3.Vector // a point on the plane
	normal   r3.Vector // unit, pointing to the allowed side
	label    string    // the obstacle behind it
	obstacle int       // its index in the world
}

// guard keeps the follower out of the nearest fixed obstacle.
//
// While the tool is outside the obstacle's margin, the plane is recomputed every tick from
// the tool's actual shape and pose. Once the hand's target crosses it, the plane is kept
// as it is until the target comes back to the allowed side: the way out of an obstacle is
// only measured from outside it, never from inside, where a thin obstacle (a 10 mm table
// top) would report the far side as the nearest exit. The plane is unbounded, so a hand
// that crossed it has to come back across it; it cannot go around the obstacle's edge.
//
// The plane constrains the follower frame's position, but how far that frame must stay
// from the obstacle depends on which way the tool is turned: tilt a gripper and a corner
// swings toward the table. So while in contact the plane keeps its direction and is moved
// along it every tick to where the tool, at its current orientation, sits exactly at the
// margin.
type guard struct {
	world *surface.World
	cfg   HapticsConfig

	mu        sync.Mutex
	current   *plane
	contact   bool
	projected uint64
	lastErr   string
}

func newGuard(w *surface.World, cfg HapticsConfig) *guard {
	return &guard{world: w, cfg: cfg}
}

// update takes the hand's unfiltered target and returns the plane to enforce, or nil when
// no obstacle is within range.
func (g *guard) update(target spatialmath.Pose) *plane {
	g.mu.Lock()
	defer g.mu.Unlock()
	// While the hand is past the current surface, that surface stands, however far past it
	// goes. Distance alone would call a hand that has gone clean through a thin obstacle
	// "clear" again, on the wrong side of it.
	if g.current != nil && g.behindLocked(target.Point()) {
		g.refitLocked(target)
		g.contact = true
		return g.copyLocked()
	}
	dist, obstacle, ok := g.world.Nearest(target)
	if !ok {
		g.current, g.contact = nil, false
		return nil
	}
	if dist >= g.cfg.MarginMM {
		g.contact = false
		if dist >= g.cfg.RangeMM {
			g.current = nil
			return nil
		}
		n, ok := g.world.Normal(target, obstacle)
		if !ok {
			// No measurable way out here: keep whatever plane there was.
			return g.copyLocked()
		}
		// Moving the target back along the normal by (dist - margin) puts the tool exactly
		// at the margin. That is where the plane is.
		g.current = &plane{
			point:    target.Point().Sub(n.Mul(dist - g.cfg.MarginMM)),
			normal:   n,
			label:    g.world.Label(obstacle),
			obstacle: obstacle,
		}
		return g.copyLocked()
	}
	// Inside the margin of the obstacle the surface belongs to, without the position having
	// crossed it: the tool turned toward the obstacle. Move the surface out to match.
	if g.current != nil && g.current.obstacle == obstacle {
		g.refitLocked(target)
		g.contact = g.behindLocked(target.Point())
		return g.copyLocked()
	}
	// Inside the margin with no surface crossed: either there is none yet (the session
	// began here) or the hand came in around the side of the last one. Nothing is enforced;
	// the planner still refuses a real collision.
	g.contact = false
	return g.copyLocked()
}

func (g *guard) behindLocked(point r3.Vector) bool {
	return point.Sub(g.current.point).Dot(g.current.normal) < 0
}

// onPlaneLocked is point, moved onto the plane if it is behind it.
func (g *guard) onPlaneLocked(point r3.Vector) r3.Vector {
	behind := point.Sub(g.current.point).Dot(g.current.normal)
	if behind >= 0 {
		return point
	}
	return point.Sub(g.current.normal.Mul(behind))
}

// refitLocked moves the current plane along its normal to where the tool, turned as the
// target is, sits exactly at the margin. The distance is measured with the tool on the
// plane, not at the hand's target, which may be far inside the obstacle.
func (g *guard) refitLocked(target spatialmath.Pose) {
	at := g.onPlaneLocked(target.Point())
	d, ok := g.world.Distance(spatialmath.NewPose(at, target.Orientation()), g.current.obstacle)
	if !ok {
		return
	}
	g.current.point = at.Add(g.current.normal.Mul(g.cfg.MarginMM - d))
}

func (g *guard) copyLocked() *plane {
	if g.current == nil {
		return nil
	}
	p := *g.current
	return &p
}

// project moves a command pose back onto the allowed side of the current plane. The
// follower slides along the surface while the hand is past it. The command's orientation
// lags the hand's by the smoothing, so the tool is checked once more as the command has it
// turned, and moved out further if it would still be inside the margin.
func (g *guard) project(cmd spatialmath.Pose) spatialmath.Pose {
	g.mu.Lock()
	defer g.mu.Unlock()
	if g.current == nil {
		return cmd
	}
	at := g.onPlaneLocked(cmd.Point())
	moved := at != cmd.Point()
	// Only near the surface: far from it the tool cannot be inside the margin.
	if at.Sub(g.current.point).Dot(g.current.normal) < projectCheckMM {
		if d, ok := g.world.Distance(spatialmath.NewPose(at, cmd.Orientation()), g.current.obstacle); ok &&
			d < g.cfg.MarginMM-projectToleranceMM {
			at = at.Add(g.current.normal.Mul(g.cfg.MarginMM - d))
			moved = true
		}
	}
	if !moved {
		return cmd
	}
	g.projected++
	return spatialmath.NewPose(at, cmd.Orientation())
}

// HapticStatus is the guard's part of the engine status.
type HapticStatus struct {
	Enabled   bool   `json:"enabled"`
	Obstacle  string `json:"obstacle"`  // the obstacle the current surface belongs to
	Contact   bool   `json:"contact"`   // the hand is past the surface right now
	Projected uint64 `json:"projected"` // ticks where the follower's target was held on a surface
	LastError string `json:"last_error"`
}

func (g *guard) status() HapticStatus {
	if g == nil {
		return HapticStatus{}
	}
	g.mu.Lock()
	defer g.mu.Unlock()
	st := HapticStatus{Enabled: true, Contact: g.contact, Projected: g.projected, LastError: g.lastErr}
	if g.current != nil {
		st.Obstacle = g.current.label
	}
	return st
}

func (g *guard) noteErr(err error) {
	g.mu.Lock()
	defer g.mu.Unlock()
	if err == nil {
		g.lastErr = ""
		return
	}
	g.lastErr = err.Error()
}

// HandPlane converts a plane on the follower's target position (world) into the hand's
// frame, by undoing the mapping
//
//	target = follower_ref + scale × R_source × (hand − hand_ref)
//
// so a device can hold the hand on the matching side.
func (r *Reference) HandPlane(pointWorld, normalWorld r3.Vector, scale float64) (point, normal r3.Vector) {
	inv := quat.Conj(r.sourceQuat())
	offset := rotate(inv, pointWorld.Sub(r.FollowerPose.Point())).Mul(1 / scale)
	return r.Hand.Point().Add(offset), rotate(inv, normalWorld)
}

// constraintCommand is the DoCommand a hand device takes to hold the hand on one side of a
// plane. Positions are millimetres in the device's own frame.
func constraintCommand(point, normal r3.Vector, cfg HapticsConfig) map[string]interface{} {
	return map[string]interface{}{
		"command":           "constraint",
		"type":              "plane",
		"point_mm":          map[string]interface{}{"x": point.X, "y": point.Y, "z": point.Z},
		"normal":            map[string]interface{}{"x": normal.X, "y": normal.Y, "z": normal.Z},
		"stiffness_n_per_m": cfg.StiffnessNPerM,
		"max_force_n":       cfg.MaxForceN,
		"hold_ms":           float64(hapticHold / time.Millisecond),
	}
}

// hapticSender sends constraints to the device without holding up the loop: one call in
// flight at a time, and only the newest waiting one is kept.
type hapticSender struct {
	dev Commander
	g   *guard

	mu      sync.Mutex
	busy    bool
	pending map[string]interface{}
	wg      sync.WaitGroup
}

func (s *hapticSender) send(cmd map[string]interface{}) {
	s.mu.Lock()
	if s.busy {
		s.pending = cmd
		s.mu.Unlock()
		return
	}
	s.busy = true
	s.wg.Add(1)
	s.mu.Unlock()
	go func() {
		defer s.wg.Done()
		for cmd != nil {
			ctx, cancel := context.WithTimeout(context.Background(), hapticCommandTimeout)
			_, err := s.dev.DoCommand(ctx, cmd)
			cancel()
			s.g.noteErr(err)
			s.mu.Lock()
			cmd, s.pending = s.pending, nil
			if cmd == nil {
				s.busy = false
			}
			s.mu.Unlock()
		}
	}()
}

// clear waits for anything in flight and removes the constraint from the device.
func (s *hapticSender) clear() {
	s.mu.Lock()
	s.pending = nil
	s.mu.Unlock()
	s.wg.Wait()
	ctx, cancel := context.WithTimeout(context.Background(), hapticClearTimeout)
	defer cancel()
	_, _ = s.dev.DoCommand(ctx, map[string]interface{}{"command": "clear_constraint"})
}
