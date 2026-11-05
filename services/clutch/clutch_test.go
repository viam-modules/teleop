package clutch

import (
	"context"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
	"go.viam.com/test"

	engine "teleop/internal/clutch"
	"teleop/internal/testfake"
)

// TestAttributesDecodeThroughRDK decodes a real attribute map the way viam-server does
// (mapstructure on the json tag), with a pose-tracker pair and an arm pair in one service.
func TestAttributesDecodeThroughRDK(t *testing.T) {
	attrs := utils.AttributeMap{
		"scale":                    1.5,
		"rotation_mode":            "relative",
		"follower_gripper_mode":    "command",
		"follower_gripper_command": map[string]interface{}{"set": "$steps"},
		"gripper_range":            850.0,
		"pairs": []interface{}{
			map[string]interface{}{"name": "left", "pose_tracker": "vive", "body": "left",
				"follower_component": "right-gripper", "rate_hz": 90.0, "rotation_mode": "absolute"},
			map[string]interface{}{"name": "right", "leader_arm": "so101-right", "leader_gripper": "so101-right-gripper",
				"follower_component": "left-gripper", "follower_gripper": "left-gripper",
				"leader_hold":                 "manual",
				"leader_tool_offset_mm":       map[string]interface{}{"x": 0.0, "y": 0.0, "z": 100.0},
				"leader_gripper_commands":     map[string]interface{}{"on_start": []interface{}{map[string]interface{}{"command": "set_torque", "enable": false}}},
				"leader_gripper_read_command": map[string]interface{}{"command": "get_position"},
				"leader_gripper_read_key":     "position_percentage",
				"capture_control_sensor":      "capture"},
		},
	}
	cfg, err := resource.TransformAttributeMap[*Config](attrs)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.Scale, test.ShouldEqual, 1.5)
	test.That(t, cfg.Pairs, test.ShouldHaveLength, 2)

	deps, _, err := cfg.Validate("services.0")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{
		"vive", "right-gripper", "builtin",
		"so101-right", "left-gripper", "so101-right-gripper", "capture",
	})

	pairs, err := cfg.resolvePairs()
	test.That(t, err, test.ShouldBeNil)
	left, err := pairs[0].Settings.Resolve()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, left.RotationMode, test.ShouldEqual, engine.RotationAbsolute) // pair override
	right, err := pairs[1].Settings.Resolve()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, right.RotationMode, test.ShouldEqual, engine.RotationRelative) // inherited
	test.That(t, right.RateHz, test.ShouldEqual, 60)
	test.That(t, right.Hold.Gripper.OnStart[0]["command"], test.ShouldEqual, "set_torque")
	test.That(t, right.GripperRead.Key, test.ShouldEqual, "position_percentage")
	test.That(t, pairs[1].Settings.ToolOffset().Z, test.ShouldEqual, 100)
}

func TestValidateRejectsAPairWithoutASource(t *testing.T) {
	cfg := &Config{Settings: engine.Settings{FollowerComponent: "g"}}
	_, _, err := cfg.Validate("services.0")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "a source is required")
	test.That(t, err.Error(), test.ShouldContainSubstring, "services.0")
}

var follower = spatialmath.NewPose(r3.Vector{X: 500, Y: 100, Z: 300}, &spatialmath.OrientationVectorDegrees{OZ: -1})

// teleopDevRig is the teleop-dev machine's shape: two SO-101 pairs, plus a vive pair to show
// the sources mix. Nothing is started on configure.
func teleopDevRig(t *testing.T) (resource.Resource, map[string]*testfake.Arm, map[string]*testfake.Motion) {
	t.Helper()
	model := testfake.Model(t)
	home := []referenceframe.Input{0, 0, 0, 0, 0}
	arms := map[string]*testfake.Arm{"left": testfake.NewArm(model, home), "right": testfake.NewArm(model, home)}
	tracker := testfake.NewPoseTracker("vive")
	tracker.SetPose("wand", spatialmath.NewPose(r3.Vector{X: 1}, &spatialmath.OrientationVectorDegrees{OZ: 1}))
	motions := map[string]*testfake.Motion{
		"left": testfake.NewMotion(follower), "right": testfake.NewMotion(follower), "wand": testfake.NewMotion(follower),
	}
	deps := resource.Dependencies{
		arm.Named("left-leader-arm"):  arms["left"],
		arm.Named("right-leader-arm"): arms["right"],
		posetracker.Named("vive"):     tracker,
		motion.Named("left-motion"):   motions["left"],
		motion.Named("right-motion"):  motions["right"],
		motion.Named("wand-motion"):   motions["wand"],
		gripper.Named("left-gripper"): testfake.NewGripper("", "", 0),
	}
	conf := resource.Config{
		Name: "teleop-clutch", API: generic.API, Model: Model,
		ConvertedAttributes: &Config{
			Settings: engine.Settings{LeaderHold: "manual", RateHz: 200, RotationMode: engine.RotationRelative},
			Pairs: []map[string]interface{}{
				{"name": "right", "leader_arm": "right-leader-arm", "follower_component": "right-gripper", "motion_service": "right-motion"},
				{"name": "left", "leader_arm": "left-leader-arm", "follower_component": "left-arm", "motion_service": "left-motion"},
				{"name": "wand", "pose_tracker": "vive", "body": "wand", "leader_hold": "", "follower_component": "x", "motion_service": "wand-motion"},
			},
		},
	}
	res, err := newService(context.Background(), deps, conf, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldBeNil)
	t.Cleanup(func() { _ = res.Close(context.Background()) })
	return res, arms, motions
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

func pairStatus(t *testing.T, res resource.Resource, name string) map[string]interface{} {
	t.Helper()
	out, err := res.DoCommand(context.Background(), map[string]interface{}{"command": "status", "pair": name})
	test.That(t, err, test.ShouldBeNil)
	return out["pairs"].(map[string]interface{})[name].(map[string]interface{})
}

// TestStartAndStopArePerPairAndIdempotent is a footswitch actions service mapping press to
// {"command": "start", "pair": "right"} and release to {"command": "stop"}.
func TestStartAndStopArePerPairAndIdempotent(t *testing.T) {
	res, arms, motions := teleopDevRig(t)
	ctx := context.Background()
	// Leaders are limp from configure; nothing is driven.
	test.That(t, arms["right"].InManualMode(), test.ShouldBeTrue)
	test.That(t, arms["left"].InManualMode(), test.ShouldBeTrue)
	test.That(t, motions["right"].StartCount(), test.ShouldEqual, 0)

	// Press.
	out, err := res.DoCommand(ctx, map[string]interface{}{"command": "start", "pair": "right"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out["running"], test.ShouldBeTrue)
	test.That(t, motions["right"].StartCount(), test.ShouldEqual, 1)
	test.That(t, motions["left"].StartCount(), test.ShouldEqual, 0)
	waitFor(t, "a move", func() bool { return motions["right"].MoveCount() >= 1 })
	_, err = res.DoCommand(ctx, map[string]interface{}{"command": "start", "side": "right"}) // repeat: no re-capture
	test.That(t, err, test.ShouldBeNil)
	test.That(t, motions["right"].StartCount(), test.ShouldEqual, 1)

	// Release, unscoped: stops everything that is running, nothing else.
	_, err = res.DoCommand(ctx, map[string]interface{}{"command": "stop"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pairStatus(t, res, "right")["running"], test.ShouldBeFalse)
	test.That(t, pairStatus(t, res, "right")["stopped_reason"], test.ShouldEqual, "stopped")
	test.That(t, arms["right"].InManualMode(), test.ShouldBeTrue) // still limp between drags
	test.That(t, motions["right"].StopCount(), test.ShouldEqual, 1)
	test.That(t, motions["left"].StopCount(), test.ShouldEqual, 0)

	// Both sides at once, and a pose-tracker pair alongside.
	_, err = res.DoCommand(ctx, map[string]interface{}{"command": "start"})
	test.That(t, err, test.ShouldBeNil)
	for _, p := range []string{"left", "right", "wand"} {
		test.That(t, pairStatus(t, res, p)["running"], test.ShouldBeTrue)
	}

	_, err = res.DoCommand(ctx, map[string]interface{}{"command": "set_rotation_mode", "mode": "off", "pair": "left"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pairStatus(t, res, "left")["rotation_mode"], test.ShouldEqual, "off")
	_, err = res.DoCommand(ctx, map[string]interface{}{"command": "set_rotation_mode", "mode": "nope"})
	test.That(t, err, test.ShouldNotBeNil)
	_, err = res.DoCommand(ctx, map[string]interface{}{"command": "start", "pair": "middle"})
	test.That(t, err, test.ShouldNotBeNil)
	_, err = res.DoCommand(ctx, map[string]interface{}{"command": "status", "paire": "left"})
	test.That(t, err, test.ShouldNotBeNil)
	_, err = res.DoCommand(ctx, map[string]interface{}{"command": "release"})
	test.That(t, err, test.ShouldNotBeNil)

	test.That(t, res.Close(ctx), test.ShouldBeNil)
	test.That(t, arms["left"].InManualMode(), test.ShouldBeFalse)
	test.That(t, arms["right"].InManualMode(), test.ShouldBeFalse)
}

func TestAMissingDependencyFailsConfigure(t *testing.T) {
	conf := resource.Config{
		Name: "clutch", API: generic.API, Model: Model,
		ConvertedAttributes: &Config{Settings: engine.Settings{PoseTracker: "vive", Body: "left", FollowerComponent: "g"}},
	}
	_, err := newService(context.Background(), resource.Dependencies{}, conf, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, `pose_tracker "vive"`)
}

func TestALeaderThatCannotGoLimpFailsConfigureAndReleasesTheOthers(t *testing.T) {
	model := testfake.Model(t)
	home := []referenceframe.Input{0, 0, 0, 0, 0}
	good, stiff := testfake.NewArm(model, home), testfake.NewArm(model, home)
	stiff.SupportsManual = false
	deps := resource.Dependencies{
		arm.Named("good"):       good,
		arm.Named("stiff"):      stiff,
		motion.Named("builtin"): testfake.NewMotion(follower),
	}
	conf := resource.Config{
		Name: "clutch", API: generic.API, Model: Model,
		ConvertedAttributes: &Config{
			Settings: engine.Settings{LeaderHold: "manual"},
			Pairs: []map[string]interface{}{
				{"name": "a", "leader_arm": "good", "follower_component": "f1"},
				{"name": "b", "leader_arm": "stiff", "follower_component": "f2"},
			},
		},
	}
	_, err := newService(context.Background(), deps, conf, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "does not support manual mode")
	test.That(t, good.InManualMode(), test.ShouldBeFalse)
}
