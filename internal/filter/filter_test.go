package filter

import (
	"math"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func at(x float64) spatialmath.Pose {
	return spatialmath.NewPose(r3.Vector{X: x}, spatialmath.NewZeroOrientation())
}

func TestFirstSampleSeedsAndPasses(t *testing.T) {
	f := New(Config{SmoothTau: time.Second, DeadzonePosMM: 5, OutlierPosMMPerSec: 10})
	p, ok := f.Step(time.Unix(0, 0), at(100))
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, p.Point().X, test.ShouldEqual, 100)
}

func TestDeadzoneSkipsSmallMovesAgainstTheLastPassedPose(t *testing.T) {
	f := New(Config{DeadzonePosMM: 2, DeadzoneRotDeg: 2})
	t0 := time.Unix(0, 0)
	f.Step(t0, at(0))
	_, ok := f.Step(t0.Add(10*time.Millisecond), at(1))
	test.That(t, ok, test.ShouldBeFalse)
	_, ok = f.Step(t0.Add(20*time.Millisecond), at(1.5))
	test.That(t, ok, test.ShouldBeFalse) // 1.5 from the last PASSED pose, not from the previous sample
	p, ok := f.Step(t0.Add(30*time.Millisecond), at(2.5))
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, p.Point().X, test.ShouldEqual, 2.5)
	test.That(t, f.Counters().Deadzoned, test.ShouldEqual, 2)

	// A zero threshold ignores that axis rather than blocking everything.
	g := New(Config{DeadzonePosMM: 2})
	g.Step(t0, at(0))
	_, ok = g.Step(t0.Add(10*time.Millisecond), spatialmath.NewPose(r3.Vector{X: 0.5}, &spatialmath.EulerAngles{Yaw: 1}))
	test.That(t, ok, test.ShouldBeFalse)
}

func TestOutliersAreRejectedThenAcceptedAfterAStreak(t *testing.T) {
	f := New(Config{OutlierPosMMPerSec: 1000, OutlierAcceptAfter: 3})
	t0 := time.Unix(0, 0)
	f.Step(t0, at(0))
	// 500 mm in 10 ms is 50 m/s: an outlier.
	for i := 1; i <= 2; i++ {
		_, ok := f.Step(t0.Add(time.Duration(i)*10*time.Millisecond), at(500))
		test.That(t, ok, test.ShouldBeFalse)
	}
	p, ok := f.Step(t0.Add(30*time.Millisecond), at(500))
	test.That(t, ok, test.ShouldBeTrue) // third in a row: the jump is real
	test.That(t, p.Point().X, test.ShouldEqual, 500)
	test.That(t, f.Counters().Rejected, test.ShouldEqual, 2)
	// Re-seeded: the next nearby sample passes without being dragged by the old pose.
	p, ok = f.Step(t0.Add(40*time.Millisecond), at(501))
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, p.Point().X, test.ShouldEqual, 501)
}

func TestSmoothingIsByTimeConstantNotByFrame(t *testing.T) {
	// Two filters sampled at different rates reach the same fraction of a step after the
	// same elapsed time.
	tau := 100 * time.Millisecond
	run := func(period time.Duration, total time.Duration) float64 {
		f := New(Config{SmoothTau: tau})
		t0 := time.Unix(0, 0)
		f.Step(t0, at(0))
		var last spatialmath.Pose
		for ts := period; ts <= total; ts += period {
			last, _ = f.Step(t0.Add(ts), at(100))
		}
		return last.Point().X
	}
	fast := run(10*time.Millisecond, 100*time.Millisecond)
	slow := run(25*time.Millisecond, 100*time.Millisecond)
	// After one time constant a continuous EMA sits at 1 - 1/e ≈ 63 %.
	test.That(t, fast, test.ShouldAlmostEqual, 100*(1-math.Exp(-1)), 1.0)
	test.That(t, slow, test.ShouldAlmostEqual, 100*(1-math.Exp(-1)), 1.0)
}

func TestResetForgetsTheSessionButKeepsCounters(t *testing.T) {
	f := New(Config{DeadzonePosMM: 5})
	t0 := time.Unix(0, 0)
	f.Step(t0, at(0))
	f.Step(t0.Add(time.Millisecond), at(1))
	test.That(t, f.Counters().Deadzoned, test.ShouldEqual, 1)
	f.Reset()
	_, ok := f.Step(t0.Add(2*time.Millisecond), at(1))
	test.That(t, ok, test.ShouldBeTrue) // seeds again
	test.That(t, f.Counters().Deadzoned, test.ShouldEqual, 1)
}
