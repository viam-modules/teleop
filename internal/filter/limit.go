package filter

import (
	"math"
	"time"

	"go.viam.com/rdk/spatialmath"

	"teleop/internal/posemath"
)

// Default follower speed caps. They bound how fast the commanded follower pose may move, in
// the follower's world, whatever the hand does and whatever the scale. The uFactory xArm
// faults above 1000 mm/s TCP speed in servo mode (C60); 250 mm/s leaves the motion planner's
// own lag and overshoot well clear of that.
const (
	DefaultMaxSpeedMMPerSec = 250.0
	DefaultMaxRotDegPerSec  = 180.0
	limiterMaxDt            = 100 * time.Millisecond
	limiterChangeEpsilonMM  = 1e-3
	limiterChangeEpsilonDeg = 1e-3
)

// Limiter moves a commanded pose toward a goal no faster than a linear and an angular speed.
// It is the last stage before the pipeline: the goal is the filtered target, the output is
// what is sent. Step runs every tick, also when the goal has not changed, so a follower that
// fell behind a fast hand catches up at the capped speed and then stops.
//
// The step is capped by the time since the previous Step, clamped to limiterMaxDt, so a
// stalled loop never turns into one large jump.
type Limiter struct {
	maxMMPerSec, maxDegPerSec float64

	cmd      spatialmath.Pose
	cmdTime  time.Time
	sentOnce bool
	limited  uint64
}

// NewLimiter returns a Limiter. A zero speed disables that axis's cap.
func NewLimiter(maxMMPerSec, maxDegPerSec float64) *Limiter {
	return &Limiter{maxMMPerSec: maxMMPerSec, maxDegPerSec: maxDegPerSec}
}

// Reset starts a session with the commanded pose at pose, normally the follower's own pose
// at registration.
func (l *Limiter) Reset(pose spatialmath.Pose, now time.Time) {
	l.cmd, l.cmdTime, l.sentOnce = pose, now, false
}

// Limited is how many steps were cut short by a cap since construction.
func (l *Limiter) Limited() uint64 { return l.limited }

// Step moves the commanded pose toward goal and returns it, with whether it should be sent:
// true when it moved, and once right after Reset so the pipeline gets a first target.
func (l *Limiter) Step(now time.Time, goal spatialmath.Pose) (spatialmath.Pose, bool) {
	if l.cmd == nil {
		l.cmd, l.cmdTime, l.sentOnce = goal, now, true
		return goal, true
	}
	dt := now.Sub(l.cmdTime)
	if dt > limiterMaxDt {
		dt = limiterMaxDt
	}
	if dt < 0 {
		dt = 0
	}
	l.cmdTime = now

	dist := goal.Point().Sub(l.cmd.Point()).Norm()
	angle := posemath.AngleBetweenDeg(l.cmd, goal)
	fPos, fRot := 1.0, 1.0
	if l.maxMMPerSec > 0 && dist > 0 {
		fPos = math.Min(1, l.maxMMPerSec*dt.Seconds()/dist)
	}
	if l.maxDegPerSec > 0 && angle > 0 {
		fRot = math.Min(1, l.maxDegPerSec*dt.Seconds()/angle)
	}
	if fPos < 1 || fRot < 1 {
		l.limited++
	}
	next := spatialmath.NewPose(
		l.cmd.Point().Add(goal.Point().Sub(l.cmd.Point()).Mul(fPos)),
		spatialmath.Interpolate(l.cmd, goal, fRot).Orientation(),
	)
	moved := next.Point().Sub(l.cmd.Point()).Norm() > limiterChangeEpsilonMM ||
		posemath.AngleBetweenDeg(l.cmd, next) > limiterChangeEpsilonDeg
	l.cmd = next
	if !l.sentOnce {
		l.sentOnce = true
		return next, true
	}
	return next, moved
}
