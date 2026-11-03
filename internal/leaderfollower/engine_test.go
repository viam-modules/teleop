package leaderfollower

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
	"teleop/internal/pipeline"
	"teleop/internal/posemath"
	"teleop/internal/testfake"
)

// The engine tests run the loop fast and with the filter off, so every tick is a send
// candidate and assertions are about the mapping, not the smoothing.

var (
	jHome = []referenceframe.Input{0, 0, 0, 0, 0}
	jA    = []referenceframe.Input{0.3, -0.5, 0.8, 0.2, 0.1}
	jB    = []referenceframe.Input{-0.2, -0.3, 0.6, 0.4, -0.3}
)

type rig struct {
	leader   *testfake.Arm
	motion   *testfake.Motion
	leaderG  *testfake.Gripper
	followG  *testfake.Gripper
	engine   *Engine
	mapping  Mapping
	follower spatialmath.Pose
}

type rigOption func(*Config, *rig)

func withCapture() rigOption { return func(c *Config, _ *rig) { c.AlignOnStart = false } }
func withHold(h HoldConfig) rigOption {
	return func(c *Config, _ *rig) { c.Hold = h }
}
func withRotationOff() rigOption { return func(c *Config, _ *rig) { c.RotationEnabled = false } }
func withAlignLimit(mm float64) rigOption {
	return func(c *Config, _ *rig) { c.AlignMaxPositionErrorMM = mm }
}

// withGrippers adds a leader gripper answering readName/readKey and a follower in command
// mode with a {"set": $steps} template.
func withGrippers(readName, readKey string, pct float64) rigOption {
	return func(c *Config, r *rig) {
		r.leaderG = testfake.NewGripper(readName, readKey, pct)
		r.followG = testfake.NewGripper("", "", 0)
		c.Gripper = gripper.OutputConfig{Mode: gripper.ModeCommand, Template: map[string]interface{}{"set": gripper.TokenSteps}, Range: 850, DeadbandPct: 3}
		c.GripperRead = GripperReadConfig{Command: map[string]interface{}{"command": readName}, Key: readKey}
	}
}

// newRig builds an engine whose follower sits exactly where the leader at followerJoints
// would map, so an align is exact unless a test perturbs it. The leader starts at jHome.
func newRig(t *testing.T, followerJoints []referenceframe.Input, opts ...rigOption) *rig {
	t.Helper()
	model := testfake.Model(t)
	r := &rig{leader: testfake.NewArm(model, jHome)}
	r.mapping = Mapping{Scale: 2, Origin: spatialmath.NewPose(r3.Vector{X: 300}, ov(0, 0, 1, 90))}
	p, err := model.Transform(followerJoints)
	test.That(t, err, test.ShouldBeNil)
	r.follower = r.mapping.Forward(p)
	r.motion = testfake.NewMotion(r.follower)
	cfg := Config{
		Mapping:              r.mapping,
		RateHz:               200,
		RotationEnabled:      true,
		AlignOnStart:         true,
		Hold:                 HoldConfig{Mode: HoldManual},
		MoveTimeout:          time.Second,
		MaxConsecutiveErrors: 10,
	}
	for _, o := range opts {
		o(&cfg, r)
	}
	deps := Deps{
		Leader:   r.leader,
		Pipeline: pipeline.New(r.motion, "builtin", "follower-gripper"),
		Logger:   logging.NewTestLogger(t),
	}
	if r.leaderG != nil {
		deps.LeaderGripper = r.leaderG
		deps.FollowerGripper = r.followG
	}
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

func TestStartWithAlignMovesTheLeaderToTheFollowerAndNotTheOtherWayRound(t *testing.T) {
	r := newRig(t, jA)
	ctx := context.Background()
	test.That(t, r.engine.Start(ctx), test.ShouldBeNil)

	// The leader was driven (one blocking move, no vendor extras) to where it maps onto the
	// follower; the follower was registered at its own pose and asked to go nowhere else.
	test.That(t, r.leader.Moves, test.ShouldHaveLength, 1)
	test.That(t, r.leader.Moves[0], test.ShouldBeNil)
	al := r.engine.Align()
	test.That(t, al.Aligned, test.ShouldBeTrue)
	test.That(t, al.PositionErrorMM, test.ShouldBeLessThan, 1)
	test.That(t, r.motion.StartCount(), test.ShouldEqual, 1)
	regAt := referenceframe.ProtobufToPoseInFrame(r.motion.Starts[0].Destination).Pose()
	poseAlmostEqual(t, regAt, r.follower, 1e-6, 1e-6)

	r.waitForMoves(t, 1)
	poseAlmostEqual(t, r.motion.LastMove(), r.follower, 1, 1)
	st := r.engine.Status()
	test.That(t, st.Running, test.ShouldBeTrue)
	test.That(t, st.SessionID, test.ShouldNotBeEmpty)
	test.That(t, st.LeaderHeld, test.ShouldBeTrue)
}

func TestFirstMirroredTargetEqualsTheFollowersPoseEvenWhenTheLeaderCannotReachIt(t *testing.T) {
	r := newRig(t, jA)
	// Twist the follower about an axis the 5-DoF leader cannot follow: the align is now
	// best effort and the gap must be absorbed, not sent.
	r.follower = spatialmath.Compose(r.follower, spatialmath.NewPoseFromOrientation(ov(1, 0, 0, 25)))
	r.motion.SetPose(r.follower)
	test.That(t, r.engine.Start(context.Background()), test.ShouldBeNil)
	al := r.engine.Align()
	test.That(t, al.OrientationErrorDeg, test.ShouldBeGreaterThan, 1)
	test.That(t, al.Residual.IsZero(), test.ShouldBeFalse)
	r.waitForMoves(t, 1)
	poseAlmostEqual(t, r.motion.LastMove(), r.follower, 1e-3, 1e-3)
}

func TestCaptureStartMovesNothingAndMirrorsDeltas(t *testing.T) {
	// The follower is somewhere unrelated to where the leader is.
	r := newRig(t, jB, withCapture())
	far := spatialmath.NewPose(r3.Vector{X: 900, Y: 400, Z: 300}, ov(0, 0, -1, 0))
	r.motion.SetPose(far)
	test.That(t, r.engine.Start(context.Background()), test.ShouldBeNil)
	test.That(t, r.leader.Moves, test.ShouldBeEmpty) // nothing drove the leader
	al := r.engine.Align()
	test.That(t, al.Aligned, test.ShouldBeFalse)
	test.That(t, al.PositionErrorMM, test.ShouldBeGreaterThan, 50) // the gap is real...
	r.waitForMoves(t, 1)
	poseAlmostEqual(t, r.motion.LastMove(), far, 1e-3, 1e-3) // ...and absorbed

	// Now the hand moves the leader: the follower target moves by scale × the leader's
	// tip delta, from where the follower was.
	before, _ := r.leader.Model().Transform(jHome)
	after, _ := r.leader.Model().Transform(jA)
	r.leader.SetJoints(jA)
	n := r.motion.MoveCount()
	r.waitForMoves(t, n+2)
	wantDelta := r.mapping.Forward(after).Point().Sub(r.mapping.Forward(before).Point())
	got := r.motion.LastMove().Point().Sub(far.Point())
	test.That(t, got.Sub(wantDelta).Norm(), test.ShouldBeLessThan, 1e-3)
}

func TestAlignRefusesWhenTheFollowerIsTooFarAwayAndTouchesNothing(t *testing.T) {
	r := newRig(t, jA, withAlignLimit(5))
	r.motion.SetPose(spatialmath.NewPose(r3.Vector{X: 5000}, spatialmath.NewZeroOrientation()))
	err := r.engine.Start(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "cannot get within 5 mm")
	test.That(t, r.leader.Moves, test.ShouldBeEmpty)
	test.That(t, r.leader.ManualCalls, test.ShouldBeEmpty)
	test.That(t, r.motion.StartCount(), test.ShouldEqual, 0)
	test.That(t, r.engine.Status().Running, test.ShouldBeFalse)
}

func TestAlignAcceptsAJointStoppingShortAndCompensates(t *testing.T) {
	r := newRig(t, jA)
	// The wrist pitch clamps well short of what the IK will ask for (it solves to about
	// 38 deg for this pose), as an arm clamping to its calibrated range does. The move
	// "succeeds", the gap is absorbed, the follower does not move on start.
	r.leader.ClampMax = []referenceframe.Input{10, 10, 10, 0.2, 10}
	test.That(t, r.engine.Start(context.Background()), test.ShouldBeNil)
	al := r.engine.Align()
	test.That(t, al.PositionErrorMM, test.ShouldBeGreaterThan, 5)
	r.waitForMoves(t, 1)
	poseAlmostEqual(t, r.motion.LastMove(), r.follower, 1e-3, 1e-3)

	// With a limit configured the same short stop is refused and the leader released.
	r2 := newRig(t, jA, withAlignLimit(2))
	r2.leader.ClampMax = []referenceframe.Input{10, 10, 10, 0.2, 10}
	err := r2.engine.Start(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "stopped short")
	test.That(t, r2.leader.InManualMode(), test.ShouldBeFalse)
	test.That(t, r2.motion.StartCount(), test.ShouldEqual, 0)
}

func TestAlignRefusesALeaderThatAcceptsTheMoveButDoesNotMove(t *testing.T) {
	r := newRig(t, jA, withAlignLimit(5))
	r.leader.Limp = true
	err := r.engine.Start(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "no joint moved")
	test.That(t, r.motion.StartCount(), test.ShouldEqual, 0)

	// Best-effort mode only warns and absorbs the whole gap.
	r2 := newRig(t, jA)
	r2.leader.Limp = true
	test.That(t, r2.engine.Start(context.Background()), test.ShouldBeNil)
	r2.waitForMoves(t, 1)
	poseAlmostEqual(t, r2.motion.LastMove(), r2.follower, 1e-3, 1e-3)
}

func TestHoldManualUsesTheArmAPIAndRefusesAnArmWithoutIt(t *testing.T) {
	r := newRig(t, jA)
	ctx := context.Background()
	test.That(t, r.engine.Start(ctx), test.ShouldBeNil)
	// Exit before the align move (a crashed session may have left it compliant), enter after.
	test.That(t, r.leader.ManualCalls, test.ShouldResemble, []bool{false, true})
	test.That(t, r.leader.InManualMode(), test.ShouldBeTrue)
	test.That(t, r.leader.CommandNames(), test.ShouldBeEmpty) // nothing vendor-specific
	test.That(t, r.engine.Stop(ctx), test.ShouldBeNil)
	test.That(t, r.leader.ManualCalls, test.ShouldResemble, []bool{false, true, false})
	test.That(t, r.motion.StopCount(), test.ShouldEqual, 1)
	st := r.engine.Status()
	test.That(t, st.Running, test.ShouldBeFalse)
	test.That(t, st.LeaderHeld, test.ShouldBeFalse)
	test.That(t, st.StoppedReason, test.ShouldEqual, StopReasonStopped)

	r2 := newRig(t, jA)
	r2.leader.SupportsManual = false
	err := r2.engine.Start(ctx)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "does not support manual mode")
	test.That(t, r2.motion.StartCount(), test.ShouldEqual, 0)

	r3 := newRig(t, jA)
	r3.leader.ManualErr = errors.New("servo 4 refused its gain read")
	err = r3.engine.Start(ctx)
	test.That(t, err.Error(), test.ShouldContainSubstring, "enter leader manual mode")
	test.That(t, r3.engine.Status().Running, test.ShouldBeFalse)
}

func TestHoldNoneSendsTheLeaderNothing(t *testing.T) {
	r := newRig(t, jA, withHold(HoldConfig{Mode: HoldNone}))
	test.That(t, r.engine.Start(context.Background()), test.ShouldBeNil)
	test.That(t, r.engine.Stop(context.Background()), test.ShouldBeNil)
	test.That(t, r.leader.ManualCalls, test.ShouldBeEmpty)
	test.That(t, r.leader.CommandNames(), test.ShouldBeEmpty)
	test.That(t, r.engine.Status().LeaderHeld, test.ShouldBeFalse)
}

func TestHoldCustomSendsTheConfiguredListsInOrderAndAbortsOnAnAfterStartFailure(t *testing.T) {
	hold := HoldConfig{Mode: HoldCustom, Leader: LeaderCommands{
		BeforeStart: []map[string]interface{}{{"command": "exit_hold"}},
		AfterStart:  []map[string]interface{}{{"command": "enter_hold", "gain": 2.0}, {"command": "second"}},
		OnStop:      []map[string]interface{}{{"command": "release"}},
	}, Gripper: GripperCommands{
		OnStart: []map[string]interface{}{{"command": "free_trigger"}},
		OnStop:  []map[string]interface{}{{"command": "lock_trigger"}},
	}}
	r := newRig(t, jA, withHold(hold), withGrippers("get_position", "pct", 50))
	ctx := context.Background()
	test.That(t, r.engine.Start(ctx), test.ShouldBeNil)
	test.That(t, r.leader.CommandNames(), test.ShouldResemble, []string{"exit_hold", "enter_hold", "second"})
	test.That(t, r.leader.Commands[1]["gain"], test.ShouldEqual, 2.0)
	test.That(t, r.leader.ManualCalls, test.ShouldBeEmpty)
	test.That(t, r.leaderG.LastWrite()["command"], test.ShouldEqual, "free_trigger")
	test.That(t, r.engine.Status().LeaderHeld, test.ShouldBeTrue)
	test.That(t, r.engine.Stop(ctx), test.ShouldBeNil)
	test.That(t, r.leader.CommandNames(), test.ShouldResemble, []string{"exit_hold", "enter_hold", "second", "release"})
	test.That(t, r.leaderG.LastWrite()["command"], test.ShouldEqual, "lock_trigger")

	// A failing after_start aborts Start, sends on_stop, and registers nothing.
	r2 := newRig(t, jA, withHold(hold), withGrippers("get_position", "pct", 50))
	r2.leader.FailCommand, r2.leader.FailCommandErr = "second", errors.New("overload")
	err := r2.engine.Start(ctx)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "leader_commands.after_start[1]")
	test.That(t, r2.leader.CommandNames(), test.ShouldResemble, []string{"exit_hold", "enter_hold", "second", "release"})
	test.That(t, r2.motion.StartCount(), test.ShouldEqual, 0)
	test.That(t, r2.engine.Status().LeaderHeld, test.ShouldBeFalse)
}

func TestOnlyOneMoveIsInFlightAndTheRestAreDropped(t *testing.T) {
	r := newRig(t, jA)
	r.motion.MoveWait = make(chan struct{})
	test.That(t, r.engine.Start(context.Background()), test.ShouldBeNil)
	r.leader.SetJoints(jB) // every tick now has a new target
	waitFor(t, "drops", func() bool { return r.engine.Status().Dropped >= 5 })
	st := r.engine.Status()
	test.That(t, st.Sends, test.ShouldEqual, 1)
	test.That(t, st.Running, test.ShouldBeTrue)
	close(r.motion.MoveWait)
	waitFor(t, "more sends", func() bool { return r.engine.Status().Sends >= 2 })
}

func TestALostSessionStopsTheLoopButLeavesTheLeaderHeld(t *testing.T) {
	r := newRig(t, jA)
	ctx := context.Background()
	test.That(t, r.engine.Start(ctx), test.ShouldBeNil)
	r.motion.SetMoveError(errors.New(`component "follower-gripper" not registered in teleop pipeline; registered: []`))
	r.leader.SetJoints(jB)
	waitFor(t, "loop to stop", func() bool { return !r.engine.Status().Running })
	st := r.engine.Status()
	test.That(t, st.StoppedReason, test.ShouldEqual, StopReasonSessionLost)
	test.That(t, st.LeaderHeld, test.ShouldBeTrue) // a hand may be on it
	test.That(t, st.LastError, test.ShouldContainSubstring, "session lost")
	test.That(t, st.ErrorsTotal, test.ShouldBeGreaterThanOrEqualTo, 1)
	test.That(t, st.LastErrorAt, test.ShouldNotBeEmpty)
	test.That(t, r.leader.InManualMode(), test.ShouldBeTrue)
	// Stop releases it; a later Start on the dead session releases first, then re-aligns.
	test.That(t, r.engine.Stop(ctx), test.ShouldBeNil)
	test.That(t, r.leader.InManualMode(), test.ShouldBeFalse)
	test.That(t, r.engine.Status().LeaderHeld, test.ShouldBeFalse)
}

func TestRepeatedReadFailuresStopTheLoopAsMaxErrors(t *testing.T) {
	r := newRig(t, jA)
	test.That(t, r.engine.Start(context.Background()), test.ShouldBeNil)
	r.leader.SetFailRead(errors.New("checksum"))
	waitFor(t, "loop to stop", func() bool { return !r.engine.Status().Running })
	st := r.engine.Status()
	test.That(t, st.StoppedReason, test.ShouldEqual, StopReasonMaxErrors)
	test.That(t, st.ConsecutiveErrors, test.ShouldEqual, 10)
	test.That(t, st.LastError, test.ShouldContainSubstring, "checksum")
	test.That(t, st.LeaderHeld, test.ShouldBeTrue)
}

func TestRotationDisabledHoldsTheFollowersStartOrientation(t *testing.T) {
	r := newRig(t, jA, withRotationOff())
	test.That(t, r.engine.Start(context.Background()), test.ShouldBeNil)
	rolled := append([]referenceframe.Input(nil), jA...)
	rolled[4] += 1.2 // wrist roll: a pure orientation change at the leader
	r.leader.SetJoints(rolled)
	n := r.motion.MoveCount()
	r.waitForMoves(t, n+2)
	last := r.motion.LastMove()
	test.That(t, posemath.AngleBetweenDeg(last, r.follower), test.ShouldBeLessThan, 1e-6)

	// With rotation on, the same roll reaches the follower.
	r2 := newRig(t, jA)
	test.That(t, r2.engine.Start(context.Background()), test.ShouldBeNil)
	r2.leader.SetJoints(rolled)
	n = r2.motion.MoveCount()
	r2.waitForMoves(t, n+2)
	test.That(t, posemath.AngleBetweenDeg(r2.motion.LastMove(), r2.follower), test.ShouldBeGreaterThan, 30)
}

func TestGripperMirrorsOnChangeOnlyThroughTheConfiguredCommands(t *testing.T) {
	r := newRig(t, jA, withGrippers("get_position", "position_percentage", 50))
	test.That(t, r.engine.Start(context.Background()), test.ShouldBeNil)
	waitFor(t, "first gripper write", func() bool { return r.followG.WriteCount() >= 1 })
	test.That(t, r.followG.LastWrite(), test.ShouldResemble, map[string]interface{}{"set": 425.0})
	time.Sleep(30 * time.Millisecond) // many ticks at the same percentage
	test.That(t, r.followG.WriteCount(), test.ShouldEqual, 1)
	r.leaderG.SetPct(51) // inside the deadband
	time.Sleep(30 * time.Millisecond)
	test.That(t, r.followG.WriteCount(), test.ShouldEqual, 1)
	r.leaderG.SetPct(60)
	waitFor(t, "second gripper write", func() bool { return r.followG.WriteCount() >= 2 })
	test.That(t, r.followG.LastWrite()["set"], test.ShouldEqual, 510.0)
	st := r.engine.Status()
	test.That(t, st.GripperPct, test.ShouldEqual, 60)
	test.That(t, st.GripperWrites, test.ShouldEqual, 2)
	test.That(t, st.GripperReadMs, test.ShouldBeGreaterThanOrEqualTo, 0)
}

func TestABlockingFollowerGripperDoesNotStallTheMirrorLoop(t *testing.T) {
	r := newRig(t, jA, withGrippers("get_position", "pct", 50))
	r.followG.Block = make(chan struct{})
	test.That(t, r.engine.Start(context.Background()), test.ShouldBeNil)
	waitFor(t, "gripper write", func() bool { return r.followG.WriteCount() >= 1 })
	c0 := r.engine.Status().Cycles
	waitFor(t, "cycles while the jaw is stuck", func() bool { return r.engine.Status().Cycles > c0+20 })
	r.leaderG.SetPct(70)
	r.leaderG.SetPct(90)
	time.Sleep(20 * time.Millisecond)
	test.That(t, r.followG.WriteCount(), test.ShouldEqual, 1)
	close(r.followG.Block)
	waitFor(t, "pending write", func() bool { return r.followG.WriteCount() >= 2 })
	time.Sleep(20 * time.Millisecond)
	test.That(t, r.followG.LastWrite()["set"], test.ShouldEqual, 765.0) // the latest, not every step
}

func TestAWrongGripperReadKeyFailsStartByName(t *testing.T) {
	r := newRig(t, jA, withGrippers("get_position", "position_percentage", 50))
	r.leaderG.ReadKey = "opening"
	err := r.engine.Start(context.Background())
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, `leader_gripper_read_key "position_percentage" not in response (keys: opening)`)
	test.That(t, r.motion.StartCount(), test.ShouldEqual, 0)
	test.That(t, r.leader.InManualMode(), test.ShouldBeFalse) // released
}

func TestAFailingFollowerGripperWriteDoesNotStopTheLoop(t *testing.T) {
	r := newRig(t, jA, withGrippers("get_position", "pct", 50))
	r.followG.FailWrite = errors.New("jaw stuck")
	test.That(t, r.engine.Start(context.Background()), test.ShouldBeNil)
	waitFor(t, "gripper write", func() bool { return r.followG.WriteCount() >= 1 })
	c0 := r.engine.Status().Cycles
	waitFor(t, "cycles", func() bool { return r.engine.Status().Cycles > c0+20 })
	test.That(t, r.engine.Status().Running, test.ShouldBeTrue)
	test.That(t, r.engine.Status().ConsecutiveErrors, test.ShouldEqual, 0)
}

func TestThePipelinesOwnErrorSurfacesInStatus(t *testing.T) {
	r := newRig(t, jA)
	test.That(t, r.engine.Start(context.Background()), test.ShouldBeNil)
	r.motion.SetStatusError("collision with table")
	waitFor(t, "pipeline error", func() bool { return r.engine.Status().PipelineError == "collision with table" })
	test.That(t, r.engine.Status().PipelinePlanMs, test.ShouldEqual, 12)
	test.That(t, r.engine.Status().Running, test.ShouldBeTrue) // sends still succeed
	r.motion.SetStatusError("")
	waitFor(t, "pipeline error cleared", func() bool { return r.engine.Status().PipelineError == "" })
}

func TestStartIsIdempotentAndStopWithoutStartIsANoOp(t *testing.T) {
	r := newRig(t, jA)
	ctx := context.Background()
	test.That(t, r.engine.Stop(ctx), test.ShouldBeNil)
	test.That(t, r.motion.StopCount(), test.ShouldEqual, 0)
	test.That(t, r.engine.Start(ctx), test.ShouldBeNil)
	test.That(t, r.engine.Start(ctx), test.ShouldBeNil)
	test.That(t, r.motion.StartCount(), test.ShouldEqual, 1)
	test.That(t, r.engine.Align().PosesMap()["aligned"], test.ShouldBeTrue)
}
