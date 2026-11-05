package filter

import (
	"math"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"teleop/internal/posemath"
)

func yawPose(x, deg float64) spatialmath.Pose {
	return spatialmath.NewPose(r3.Vector{X: x}, &spatialmath.EulerAngles{Yaw: deg * math.Pi / 180})
}

func TestLimiterCapsLinearSpeedAndCatchesUpWhileTheGoalHolds(t *testing.T) {
	l := NewLimiter(250, 0)
	t0 := time.Unix(1000, 0)
	l.Reset(yawPose(0, 0), t0)

	// First step after reset sends, even standing still.
	p, send := l.Step(t0, yawPose(0, 0))
	test.That(t, send, test.ShouldBeTrue)
	test.That(t, p.Point().X, test.ShouldEqual, 0)
	_, send = l.Step(t0.Add(10*time.Millisecond), yawPose(0, 0))
	test.That(t, send, test.ShouldBeFalse) // converged, nothing to send

	// The hand jumps 100 mm. At 250 mm/s each 20 ms tick may move 5 mm.
	goal := yawPose(100, 0)
	now := t0.Add(10 * time.Millisecond)
	for i := 1; i <= 20; i++ {
		now = now.Add(20 * time.Millisecond)
		p, send = l.Step(now, goal)
		test.That(t, send, test.ShouldBeTrue)
		test.That(t, p.Point().X, test.ShouldAlmostEqual, float64(5*i), 1e-9)
	}
	test.That(t, l.Limited(), test.ShouldEqual, 19) // the 20th step lands exactly
	now = now.Add(20 * time.Millisecond)
	_, send = l.Step(now, goal)
	test.That(t, send, test.ShouldBeFalse)
}

func TestLimiterNeverTurnsAStallIntoAJump(t *testing.T) {
	l := NewLimiter(250, 0)
	t0 := time.Unix(1000, 0)
	l.Reset(yawPose(0, 0), t0)
	l.Step(t0, yawPose(0, 0))
	// Two seconds without a step: still only limiterMaxDt worth of motion.
	p, _ := l.Step(t0.Add(2*time.Second), yawPose(1000, 0))
	test.That(t, p.Point().X, test.ShouldAlmostEqual, 25, 1e-9)
}

func TestLimiterCapsRotationIndependently(t *testing.T) {
	l := NewLimiter(250, 90)
	t0 := time.Unix(1000, 0)
	l.Reset(yawPose(0, 0), t0)
	l.Step(t0, yawPose(0, 0))
	// 1 mm and 90 deg away: position arrives at once, rotation at 90 deg/s.
	p, _ := l.Step(t0.Add(100*time.Millisecond), yawPose(1, 90))
	test.That(t, p.Point().X, test.ShouldAlmostEqual, 1, 1e-9)
	test.That(t, posemath.AngleBetweenDeg(yawPose(0, 0), p), test.ShouldAlmostEqual, 9, 1e-6)
}

func TestLimiterWithoutCapsPassesTheGoal(t *testing.T) {
	l := NewLimiter(0, 0)
	t0 := time.Unix(1000, 0)
	l.Reset(yawPose(0, 0), t0)
	l.Step(t0, yawPose(0, 0))
	p, send := l.Step(t0.Add(time.Millisecond), yawPose(500, 170))
	test.That(t, send, test.ShouldBeTrue)
	test.That(t, p.Point().X, test.ShouldAlmostEqual, 500, 1e-9)
	test.That(t, posemath.AngleBetweenDeg(yawPose(500, 170), p), test.ShouldAlmostEqual, 0, 1e-6)
	test.That(t, l.Limited(), test.ShouldEqual, 0)
}
