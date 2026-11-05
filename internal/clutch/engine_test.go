package clutch

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"teleop/internal/gripper"
	"teleop/internal/hold"
	"teleop/internal/pipeline"
	"teleop/internal/posemath"
	"teleop/internal/testfake"
)

// The engine tests run the loop fast with the filter off, so every tick is a send candidate
// and the assertions are about start/stop and the mapping, not the smoothing.

const (
	trackerName = "vive"
	body        = "left"
)

var (
	followerStart = spatialmath.NewPose(r3.Vector{X: 500, Y: 100, Z: 300}, ov(0, 0, -1, 0))
	hand0         = spatialmath.NewPose(r3.Vector{X: 1, Y: 2, Z: 3}, yaw(0))
	jHome         = []referenceframe.Input{0, 0, 0, 0, 0}
	jA            = []referenceframe.Input{0.3, -0.5, 0.8, 0.2, 0.1}
)

type rig struct {
	tracker *testfake.PoseTracker
	arm     *testfake.Arm
	motion  *testfake.Motion
	frames  *testfake.Frames
	capture *testfake.Commander
	leaderG *testfake.Gripper
	followG *testfake.Gripper
	engine  *Engine
}

type rigOption func(*Config, *rig, *Deps)

func withMode(m RotationMode) rigOption {
	return func(c *Config, _ *rig, _ *Deps) { c.RotationMode = m }
}
func withScale(s float64) rigOption { return func(c *Config, _ *rig, _ *Deps) { c.Scale = s } }
func withNoFrames() rigOption       { return func(_ *Config, _ *rig, d *Deps) { d.Frames = nil } }
func withUnknownFrame() rigOption {
	return func(_ *Config, r *rig, _ *Deps) { r.frames.Rotations = map[string]spatialmath.Orientation{} }
}
func withCapture() rigOption {
	return func(_ *Config, r *rig, d *Deps) { r.capture = testfake.NewCommander(); d.Capture = r.capture }
}
func withHold(h hold.Config) rigOption { return func(c *Config, _ *rig, _ *Deps) { c.Hold = h } }

// withLeaderGripper adds a leader gripper (arm rig) answering readName/readKey and a
// follower gripper in command mode.
func withLeaderGripper(readName, readKey string, pct float64) rigOption {
	return func(c *Config, r *rig, d *Deps) {
		r.leaderG = testfake.NewGripper(readName, readKey, pct)
		r.followG = testfake.NewGripper("", "", 0)
		d.LeaderGripper, d.FollowerGripper = r.leaderG, r.followG
		c.Gripper = gripper.OutputConfig{Mode: gripper.ModeCommand, Template: map[string]interface{}{"set": gripper.TokenSteps}, Range: 850, DeadbandPct: 3}
		c.GripperRead = GripperReadConfig{Command: map[string]interface{}{"command": readName}, Key: readKey}
	}
}

// baseConfig lifts the speed caps far out of the way, so the mapping tests see the mapping;
// the caps have their own tests below.
func baseConfig() Config {
	return Config{
		MaxSpeedMMPerSec:     1e9,
		MaxRotDegPerSec:      1e9,
		Scale:                1,
		RotationMode:         RotationOff,
		RateHz:               200,
		MoveTimeout:          time.Second,
		MaxConsecutiveErrors: 3,
	}
}

// newTrackerRig builds an engine on a pose tracker whose frame is yawed 90 deg in world
// (tracker +X is world +Y), with the hand tracked at hand0 and the follower at followerStart.
func newTrackerRig(t *testing.T, opts ...rigOption) *rig {
	t.Helper()
	r := &rig{
		tracker: testfake.NewPoseTracker(trackerName),
		motion:  testfake.NewMotion(followerStart),
		frames:  testfake.NewFrames(map[string]spatialmath.Orientation{trackerName: yaw(90)}),
	}
	r.tracker.SetPose(body, hand0)
	cfg := baseConfig()
	deps := Deps{
		Source:   &TrackerSource{Tracker: r.tracker, Name: trackerName, Body: body},
		Frames:   r.frames,
		Pipeline: pipeline.New(r.motion, "builtin", "follower-gripper"),
		Logger:   logging.NewTestLogger(t),
	}
	for _, o := range opts {
		o(&cfg, r, &deps)
	}
	var err error
	r.engine, err = New(deps, cfg)
	test.That(t, err, test.ShouldBeNil)
	t.Cleanup(func() { _ = r.engine.Stop(context.Background()) })
	return r
}

// newArmRig builds an engine on a leader arm (the test model at jHome) whose base frame is
// identity in world.
func newArmRig(t *testing.T, opts ...rigOption) *rig {
	t.Helper()
	model := testfake.Model(t)
	r := &rig{
		arm:    testfake.NewArm(model, jHome),
		motion: testfake.NewMotion(followerStart),
		frames: testfake.NewFrames(map[string]spatialmath.Orientation{"leader-arm_origin": yaw(0)}),
	}
	cfg := baseConfig()
	deps := Deps{
		Source:   &ArmSource{Arm: r.arm, Name: "leader-arm"},
		Frames:   r.frames,
		Pipeline: pipeline.New(r.motion, "builtin", "follower-gripper"),
		Leader:   r.arm,
		Logger:   logging.NewTestLogger(t),
	}
	for _, o := range opts {
		o(&cfg, r, &deps)
	}
	var err error
	r.engine, err = New(deps, cfg)
	test.That(t, err, test.ShouldBeNil)
	t.Cleanup(func() { _ = r.engine.Stop(context.Background()) })
	return r
}

func waitFor(t *testing.T, what string, cond func() bool) {
	t.Helper()
	deadline := time.Now().Add(3 * time.Second)
	for time.Now().Before(deadline) {
		if cond() {
			return
		}
		time.Sleep(2 * time.Millisecond)
	}
	t.Fatalf("timed out waiting for %s", what)
}

func (r *rig) waitForMoves(t *testing.T, n int) {
	t.Helper()
	waitFor(t, "moves", func() bool { return r.motion.MoveCount() >= n })
}

// waitForPose waits until the latest teleop_move is want, within 1e-6 mm and degrees. The
// limiter sends only while the commanded pose moves, so tests wait for arrival, not counts.
func (r *rig) waitForPose(t *testing.T, want spatialmath.Pose) {
	t.Helper()
	waitFor(t, "the follower target", func() bool {
		m := r.motion.LastMove()
		return m != nil && m.Point().Sub(want.Point()).Norm() < 1e-6 && posemath.AngleBetweenDeg(m, want) < 1e-6
	})
}

func (r *rig) start(t *testing.T) {
	t.Helper()
	test.That(t, r.engine.Start(context.Background()), test.ShouldBeNil)
}

func (r *rig) stop(t *testing.T) {
	t.Helper()
	test.That(t, r.engine.Stop(context.Background()), test.ShouldBeNil)
}

func TestNothingIsSentBeforeStartAndStartMovesNothing(t *testing.T) {
	r := newTrackerRig(t)
	time.Sleep(20 * time.Millisecond)
	test.That(t, r.motion.StartCount(), test.ShouldEqual, 0)
	test.That(t, r.tracker.ReadCount(), test.ShouldEqual, 0) // no loop before start

	r.start(t)
	test.That(t, r.motion.StartCount(), test.ShouldEqual, 1)
	regAt := referenceframe.ProtobufToPoseInFrame(r.motion.Starts[0].Destination).Pose()
	poseAlmostEqual(t, regAt, followerStart, 1e-9, 1e-9)
	r.waitForMoves(t, 1)
	poseAlmostEqual(t, r.motion.LastMove(), followerStart, 1e-6, 1e-6)
	test.That(t, r.frames.CallCount(), test.ShouldEqual, 1) // once per start, not per tick
	st := r.engine.Status()
	test.That(t, st.Running, test.ShouldBeTrue)
	test.That(t, st.Tracked, test.ShouldBeTrue)
	test.That(t, st.Sessions, test.ShouldEqual, 1)
}

func TestMotionIsAScaledDeltaThroughTheSourceFrameRotation(t *testing.T) {
	r := newTrackerRig(t, withScale(2))
	r.start(t)
	r.waitForMoves(t, 1)
	// +10 along the tracker's X is +10 along world Y, times the scale.
	r.tracker.SetPose(body, spatialmath.NewPose(r3.Vector{X: 11, Y: 2, Z: 3}, yaw(0)))
	r.waitForPose(t, spatialmath.NewPose(r3.Vector{X: 500, Y: 120, Z: 300}, ov(0, 0, -1, 0)))
}

func TestStopHoldsTheFollowerAndEachStartCapturesFreshReferences(t *testing.T) {
	r := newTrackerRig(t)
	r.start(t)
	r.waitForMoves(t, 1)
	r.start(t) // repeated start: no re-capture
	test.That(t, r.motion.StartCount(), test.ShouldEqual, 1)

	r.stop(t)
	test.That(t, r.motion.StopCount(), test.ShouldEqual, 1)
	st := r.engine.Status()
	test.That(t, st.Running, test.ShouldBeFalse)
	test.That(t, st.StoppedReason, test.ShouldEqual, StopReasonStopped)
	r.stop(t) // idempotent
	test.That(t, r.motion.StopCount(), test.ShouldEqual, 1)

	reads := r.tracker.ReadCount()
	r.tracker.SetPose(body, spatialmath.NewPose(r3.Vector{X: 100, Y: 50, Z: 3}, yaw(0)))
	elsewhere := spatialmath.NewPose(r3.Vector{X: 700, Y: 0, Z: 200}, ov(0, 0, -1, 0))
	r.motion.SetPose(elsewhere)
	n := r.motion.MoveCount()
	time.Sleep(30 * time.Millisecond)
	test.That(t, r.motion.MoveCount(), test.ShouldEqual, n)
	test.That(t, r.tracker.ReadCount(), test.ShouldEqual, reads) // no loop while stopped

	r.start(t)
	test.That(t, r.motion.StartCount(), test.ShouldEqual, 2)
	regAt := referenceframe.ProtobufToPoseInFrame(r.motion.Starts[1].Destination).Pose()
	poseAlmostEqual(t, regAt, elsewhere, 1e-9, 1e-9)
	r.waitForMoves(t, n+1)
	poseAlmostEqual(t, r.motion.LastMove(), elsewhere, 1e-6, 1e-6)
	test.That(t, r.engine.Status().Sessions, test.ShouldEqual, 2)
}

func TestStartRefusesAnUntrackedHand(t *testing.T) {
	r := newTrackerRig(t)
	r.tracker.Lose(body)
	err := r.engine.Start(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "not tracked")
	test.That(t, r.motion.StartCount(), test.ShouldEqual, 0)
	test.That(t, r.engine.Status().Running, test.ShouldBeFalse)
}

func TestLosingTrackingEndsTheSessionUntilTheNextStart(t *testing.T) {
	r := newTrackerRig(t)
	r.start(t)
	r.waitForMoves(t, 1)
	r.tracker.Lose(body)
	waitFor(t, "loop end", func() bool { return !r.engine.Status().Running })
	st := r.engine.Status()
	test.That(t, st.StoppedReason, test.ShouldEqual, StopReasonUntracked)
	test.That(t, st.Tracked, test.ShouldBeFalse)
	test.That(t, r.motion.StopCount(), test.ShouldEqual, 1)

	r.tracker.SetPose(body, hand0)
	time.Sleep(20 * time.Millisecond)
	test.That(t, r.motion.StartCount(), test.ShouldEqual, 1)
	r.start(t)
	test.That(t, r.motion.StartCount(), test.ShouldEqual, 2)
	r.stop(t)
	test.That(t, r.motion.StopCount(), test.ShouldEqual, 2) // teleop_stop once per session
}

func TestRotationModesAgainstTheFollower(t *testing.T) {
	handTurned := spatialmath.NewPose(hand0.Point(), yaw(30))
	for _, tc := range []struct {
		mode RotationMode
		want spatialmath.Orientation
	}{
		{RotationOff, followerStart.Orientation()},
		{RotationRelative, followerStart.Orientation()}, // no jump at start
		{RotationAbsolute, yaw(120)},
	} {
		t.Run(string(tc.mode), func(t *testing.T) {
			r := newTrackerRig(t, withMode(tc.mode))
			r.tracker.SetPose(body, handTurned)
			r.start(t)
			r.waitForMoves(t, 1)
			poseAlmostEqual(t, r.motion.LastMove(), spatialmath.NewPose(followerStart.Point(), tc.want), 1e-6, 1e-6)
		})
	}
}

func TestSetRotationModeTakesEffectMidSession(t *testing.T) {
	r := newTrackerRig(t, withMode(RotationOff))
	r.tracker.SetPose(body, spatialmath.NewPose(hand0.Point(), yaw(30)))
	r.start(t)
	r.waitForMoves(t, 1)
	poseAlmostEqual(t, r.motion.LastMove(), followerStart, 1e-6, 1e-6)

	test.That(t, r.engine.SetRotationMode("sideways"), test.ShouldNotBeNil)
	test.That(t, r.engine.SetRotationMode(RotationAbsolute), test.ShouldBeNil)
	r.waitForPose(t, spatialmath.NewPose(followerStart.Point(), yaw(120)))
}

func TestAnUnknownSourceFrameFallsBackToWorld(t *testing.T) {
	for name, opt := range map[string]rigOption{"no frame system": withNoFrames(), "frame not configured": withUnknownFrame()} {
		t.Run(name, func(t *testing.T) {
			r := newTrackerRig(t, opt)
			r.start(t)
			r.waitForMoves(t, 1)
			r.tracker.SetPose(body, spatialmath.NewPose(r3.Vector{X: 11, Y: 2, Z: 3}, yaw(0)))
			r.waitForPose(t, spatialmath.NewPose(r3.Vector{X: 510, Y: 100, Z: 300}, ov(0, 0, -1, 0)))
		})
	}
}

func TestCaptureSensorBracketsEverySession(t *testing.T) {
	r := newTrackerRig(t, withCapture())
	r.start(t)
	r.stop(t)
	test.That(t, r.capture.Keys(), test.ShouldResemble, []string{"start-capture", "stop-capture"})
}

func TestAFailedRegistrationIsReportedAndTheNextStartWorks(t *testing.T) {
	r := newArmRig(t)
	r.motion.SetStartError(errors.New("planner busy"))
	err := r.engine.Start(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "planner busy")
	test.That(t, r.engine.Status().Running, test.ShouldBeFalse)

	r.motion.SetStartError(nil)
	r.start(t)
	test.That(t, r.engine.Status().Running, test.ShouldBeTrue)
}

func TestASessionLostOnTheServerEndsTheLoop(t *testing.T) {
	r := newTrackerRig(t)
	r.start(t)
	r.waitForMoves(t, 1)
	r.motion.SetMoveError(errors.New("teleop pipeline is not running"))
	r.tracker.SetPose(body, spatialmath.NewPose(r3.Vector{X: 11, Y: 2, Z: 3}, yaw(0)))
	waitFor(t, "loop end", func() bool { return !r.engine.Status().Running })
	test.That(t, r.engine.Status().StoppedReason, test.ShouldEqual, StopReasonSessionLost)
}

func TestRepeatedSendFailuresEndTheLoop(t *testing.T) {
	// A slow cap keeps the follower catching up for many ticks, so every tick sends.
	r := newArmRig(t, withCaps(1, 1e9))
	r.start(t)
	r.waitForMoves(t, 1)
	r.motion.SetMoveError(errors.New("bad things"))
	r.arm.SetJoints(jA)
	waitFor(t, "loop end", func() bool { return !r.engine.Status().Running })
	test.That(t, r.engine.Status().StoppedReason, test.ShouldEqual, StopReasonMaxErrors)
	test.That(t, r.motion.StopCount(), test.ShouldEqual, 1)
	r.stop(t)
	test.That(t, r.motion.StopCount(), test.ShouldEqual, 1)
}

// TestTheLeaderStaysLimpAcrossStartsAndStops is the point of a clutch: the hand drags a
// limp leader to a new place between drags, so start and stop must never stiffen it.
func TestTheLeaderStaysLimpAcrossStartsAndStops(t *testing.T) {
	r := newArmRig(t, withHold(hold.Config{Mode: hold.Manual}))
	ctx := context.Background()
	test.That(t, r.arm.InManualMode(), test.ShouldBeFalse) // New does no I/O
	test.That(t, r.engine.HoldLeader(ctx), test.ShouldBeNil)
	test.That(t, r.arm.InManualMode(), test.ShouldBeTrue)
	for i := 0; i < 2; i++ {
		r.start(t)
		test.That(t, r.arm.InManualMode(), test.ShouldBeTrue)
		r.stop(t)
		test.That(t, r.arm.InManualMode(), test.ShouldBeTrue)
	}
	test.That(t, r.engine.Status().LeaderHeld, test.ShouldBeTrue)
	r.engine.ReleaseLeader(ctx)
	test.That(t, r.arm.InManualMode(), test.ShouldBeFalse)
	test.That(t, r.engine.Status().LeaderHeld, test.ShouldBeFalse)
}

func TestHoldLeaderRefusesAnArmWithoutManualMode(t *testing.T) {
	r := newArmRig(t, withHold(hold.Config{Mode: hold.Manual}))
	r.arm.SupportsManual = false
	err := r.engine.HoldLeader(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "does not support manual mode")
}

func TestArmSourceMirrorsTheLeaderAndItsGripper(t *testing.T) {
	r := newArmRig(t, withLeaderGripper("get_position", "pct", 40))
	r.start(t)
	r.waitForMoves(t, 1)
	poseAlmostEqual(t, r.motion.LastMove(), followerStart, 1e-6, 1e-6)
	waitFor(t, "a gripper write", func() bool { return r.followG.WriteCount() >= 1 })
	test.That(t, r.followG.LastWrite()["set"], test.ShouldEqual, 340)

	before, _ := r.arm.Model().Transform(jHome)
	after, _ := r.arm.Model().Transform(jA)
	r.arm.SetJoints(jA)
	wantPt := followerStart.Point().Add(after.Point().Sub(before.Point()))
	waitFor(t, "the follower target", func() bool {
		m := r.motion.LastMove()
		return m != nil && m.Point().Sub(wantPt).Norm() < 1e-6
	})

	r.stop(t)
}

func TestArmSourceReadFailuresEndTheLoopAfterTheLimit(t *testing.T) {
	r := newArmRig(t)
	r.start(t)
	r.arm.SetFailRead(errors.New("serial timeout"))
	waitFor(t, "loop end", func() bool { return !r.engine.Status().Running })
	st := r.engine.Status()
	test.That(t, st.StoppedReason, test.ShouldEqual, StopReasonMaxErrors)
	test.That(t, st.LastError, test.ShouldContainSubstring, "serial timeout")
}

func TestAHoldWithoutALeaderIsRefused(t *testing.T) {
	r := newTrackerRig(t)
	_, err := New(Deps{Source: r.engine.deps.Source, Pipeline: r.engine.deps.Pipeline},
		Config{Hold: hold.Config{Mode: hold.Manual}})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "needs a leader arm")
}

func TestArmSourceToolOffsetMirrorsTheTipNotTheWrist(t *testing.T) {
	model := testfake.Model(t)
	a := testfake.NewArm(model, jA)
	wrist, err := model.Transform(jA)
	test.That(t, err, test.ShouldBeNil)
	src := &ArmSource{Arm: a, Name: "leader", ToolOffset: r3.Vector{Z: 100}}
	got, err := src.Read(context.Background())
	test.That(t, err, test.ShouldBeNil)
	want := spatialmath.Compose(wrist, spatialmath.NewPoseFromPoint(r3.Vector{Z: 100}))
	poseAlmostEqual(t, got.Pose, want, 1e-9, 1e-9)
	test.That(t, got.Pose.Point().Sub(wrist.Point()).Norm(), test.ShouldAlmostEqual, 100, 1e-9)
	test.That(t, got.Frame, test.ShouldEqual, "leader_origin")
}

func withCaps(mm, deg float64) rigOption {
	return func(c *Config, _ *rig, _ *Deps) { c.MaxSpeedMMPerSec, c.MaxRotDegPerSec = mm, deg }
}

// TestAFastHandIsFollowedAtTheCappedSpeed is the xArm incident: a quick hand motion at a
// large scale asked for over 1 m/s. Every commanded step is now bounded by the cap times the
// tick, and the follower still arrives once the hand stops.
func TestAFastHandIsFollowedAtTheCappedSpeed(t *testing.T) {
	r := newTrackerRig(t, withCaps(250, 180), withScale(2.5))
	r.start(t)
	r.waitForMoves(t, 1)
	// The hand jumps 100 mm in one tick: a 250 mm jump of the target in world (+Y, through
	// the tracker's 90 deg frame rotation).
	r.tracker.SetPose(body, spatialmath.NewPose(r3.Vector{X: 101, Y: 2, Z: 3}, yaw(0)))
	want := spatialmath.NewPose(r3.Vector{X: 500, Y: 350, Z: 300}, ov(0, 0, -1, 0))
	waitFor(t, "arrival", func() bool {
		m := r.motion.LastMove()
		return m != nil && m.Point().Sub(want.Point()).Norm() < 1e-6
	})
	// No consecutive commands further apart than the cap allows for the loop's longest gap
	// (the limiter clamps a stalled tick to 100 ms).
	moves := r.motion.MovePoses()
	for i := 1; i < len(moves); i++ {
		test.That(t, moves[i].Point().Sub(moves[i-1].Point()).Norm(), test.ShouldBeLessThanOrEqualTo, 250*0.1+1e-6)
	}
	test.That(t, len(moves), test.ShouldBeGreaterThan, 10) // it took many capped steps
	test.That(t, r.engine.Status().SpeedLimited, test.ShouldBeGreaterThan, 0)
}

func TestASessionEndsWhenThePipelineReportsAnError(t *testing.T) {
	r := newArmRig(t)
	r.start(t)
	r.waitForMoves(t, 1)
	r.motion.SetStatusError("xArm: Emergency Stop Button Pushed In")
	waitFor(t, "loop end", func() bool { return !r.engine.Status().Running })
	st := r.engine.Status()
	test.That(t, st.StoppedReason, test.ShouldEqual, StopReasonPipelineError)
	test.That(t, st.LastError, test.ShouldContainSubstring, "Emergency Stop")
	test.That(t, r.motion.StopCount(), test.ShouldEqual, 1)

	// While the pipeline still reports it, start is refused and registers nothing.
	err := r.engine.Start(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "Emergency Stop")
	test.That(t, r.motion.StartCount(), test.ShouldEqual, 1)

	// Cleared: start works again, and the stale fault from the last session is forgotten.
	r.motion.SetStatusError("")
	r.start(t)
	time.Sleep(50 * time.Millisecond)
	test.That(t, r.engine.Status().Running, test.ShouldBeTrue)
}
