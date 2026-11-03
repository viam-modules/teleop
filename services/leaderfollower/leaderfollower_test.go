package leaderfollower

import (
	"context"
	"errors"
	"testing"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
	"go.viam.com/test"

	engine "teleop/internal/leaderfollower"
	"teleop/internal/testfake"
)

// TestAttributesDecodeThroughRDK decodes a real attribute map the way viam-server does
// (resource.TransformAttributeMap, i.e. mapstructure on the json tag). Building Config as a
// Go literal bypasses that decoder entirely, which is how an embedded Settings that decoded
// to all-zero once shipped with every test green.
func TestAttributesDecodeThroughRDK(t *testing.T) {
	attrs := utils.AttributeMap{
		"leader_arm":            "right-leader-arm",
		"leader_gripper":        "right-leader-gripper",
		"follower_component":    "right-gripper",
		"follower_gripper":      "right-gripper",
		"scale":                 2.5,
		"leader_tool_offset_mm": map[string]interface{}{"x": 0.0, "y": 0.0, "z": 96.0},
		"origin_translation_mm": map[string]interface{}{"x": 0.0, "y": 1003.0, "z": 0.0},
		"origin_orientation": map[string]interface{}{
			"type": "ov_degrees", "value": map[string]interface{}{"x": 0.0, "y": 0.0, "z": 1.0, "th": 0.0},
		},
		"align_on_start": true,
		"leader_hold":    "custom",
		"leader_commands": map[string]interface{}{
			"after_start": []interface{}{map[string]interface{}{"command": "enter_manual_mode", "p_gain": 4.0}},
			"on_stop":     []interface{}{map[string]interface{}{"command": "exit_manual_mode"}},
		},
		"leader_gripper_commands": map[string]interface{}{
			"on_start": []interface{}{map[string]interface{}{"command": "set_torque", "enable": false}},
		},
		"follower_gripper_mode":       "command",
		"follower_gripper_command":    map[string]interface{}{"set": "$steps"},
		"leader_gripper_read_command": map[string]interface{}{"command": "get_position"},
		"leader_gripper_read_key":     "position_percentage",
		"auto_start":                  true,
		"pairs": []interface{}{
			map[string]interface{}{"name": "left", "leader_arm": "left-leader-arm", "leader_gripper": "left-leader-gripper",
				"follower_component": "left-gripper", "follower_gripper": "left-gripper"},
			map[string]interface{}{"name": "right"},
		},
	}
	cfg, err := resource.TransformAttributeMap[*Config](attrs)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.LeaderArm, test.ShouldEqual, "right-leader-arm")
	test.That(t, cfg.Scale, test.ShouldEqual, 2.5)
	test.That(t, cfg.AlignOnStart, test.ShouldBeTrue)
	test.That(t, cfg.AutoStart, test.ShouldBeTrue)
	test.That(t, cfg.LeaderToolOffset.Z, test.ShouldEqual, 96)
	test.That(t, cfg.OriginTranslation.Y, test.ShouldEqual, 1003)
	test.That(t, cfg.OriginOrientation, test.ShouldNotBeNil)
	test.That(t, cfg.LeaderCommands, test.ShouldNotBeNil)
	test.That(t, cfg.LeaderCommands.AfterStart[0]["p_gain"], test.ShouldEqual, 4.0)
	test.That(t, cfg.LeaderGripperCommands.OnStart[0]["enable"], test.ShouldEqual, false)
	test.That(t, cfg.FollowerGripperCommand["set"], test.ShouldEqual, "$steps")
	test.That(t, cfg.Pairs, test.ShouldHaveLength, 2)

	deps, _, err := cfg.Validate("services.0")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{
		"left-leader-arm", "left-gripper", "builtin", "left-leader-gripper",
		"right-leader-arm", "right-gripper", "right-leader-gripper",
	})

	pairs, err := cfg.resolvePairs()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pairs[1].Settings.LeaderArm, test.ShouldEqual, "right-leader-arm") // inherited
	resolved, err := pairs[0].Settings.Resolve()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resolved.Mapping.Origin.Point().Y, test.ShouldEqual, 1003)
	test.That(t, resolved.Hold.Leader.OnStop[0]["command"], test.ShouldEqual, "exit_manual_mode")
}

func TestAStaleSo101ConfigIsRejectedWithTheReplacementNamed(t *testing.T) {
	cfg, err := resource.TransformAttributeMap[*Config](utils.AttributeMap{
		"leader_arm": "l", "follower_component": "f", "leader_hold": "manual", "gripper_in_max_pct": 70.0,
	})
	test.That(t, err, test.ShouldBeNil)
	_, _, err = cfg.Validate("services.0")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "gripper_in_max_pct is no longer supported")
	test.That(t, err.Error(), test.ShouldContainSubstring, "services.0")
}

func TestValidateDeclaresEveryDependency(t *testing.T) {
	cfg := &Config{Settings: engine.Settings{
		LeaderArm: "leader-arm", FollowerComponent: "xarm-gripper",
		LeaderGripper: "leader-gripper", FollowerGripper: "xarm-gripper",
		LeaderGripperReadCommand: map[string]interface{}{"command": "get_position"}, LeaderGripperReadKey: "pct",
	}}
	deps, _, err := cfg.Validate("")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{"leader-arm", "xarm-gripper", "builtin", "leader-gripper"})

	cfg.MotionService = "motion-2"
	deps, _, err = cfg.Validate("")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldContain, "motion-2")

	_, _, err = (&Config{}).Validate("")
	test.That(t, err, test.ShouldNotBeNil)
}

// twoPairRig builds a service with a left and a right pair on manual hold, each with its own
// leader and motion fake, and returns the fakes by pair name.
func twoPairRig(t *testing.T, logger logging.Logger, autoStart bool) (resource.Resource, map[string]*testfake.Arm, map[string]*testfake.Motion, error) {
	t.Helper()
	model := testfake.Model(t)
	joints := []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0}
	pose, err := model.Transform(joints)
	test.That(t, err, test.ShouldBeNil)

	arms := map[string]*testfake.Arm{"left": testfake.NewArm(model, joints), "right": testfake.NewArm(model, joints)}
	motions := map[string]*testfake.Motion{"left": testfake.NewMotion(pose), "right": testfake.NewMotion(pose)}
	deps := resource.Dependencies{
		arm.Named("left-leader"):      arms["left"],
		arm.Named("right-leader"):     arms["right"],
		motion.Named("left-motion"):   motions["left"],
		motion.Named("right-motion"):  motions["right"],
		gripper.Named("left-gripper"): testfake.NewGripper("", "", 0),
	}
	conf := resource.Config{
		Name: "teleop", API: generic.API, Model: Model,
		ConvertedAttributes: &Config{
			Settings:  engine.Settings{LeaderHold: engine.HoldManual},
			AutoStart: autoStart,
			Pairs: []map[string]interface{}{
				{"name": "left", "leader_arm": "left-leader", "follower_component": "left-gripper", "motion_service": "left-motion"},
				{"name": "right", "leader_arm": "right-leader", "follower_component": "right-gripper", "motion_service": "right-motion"},
			},
		},
	}
	res, err := newService(context.Background(), deps, conf, logger)
	return res, arms, motions, err
}

func running(t *testing.T, st map[string]interface{}, pair string) bool {
	t.Helper()
	return st["pairs"].(map[string]interface{})[pair].(map[string]interface{})["running"].(bool)
}

func TestConstructorWiresTheEnginesAndStatusReportsIdle(t *testing.T) {
	res, arms, motions, err := twoPairRig(t, logging.NewTestLogger(t), false)
	test.That(t, err, test.ShouldBeNil)
	defer res.Close(context.Background())
	test.That(t, res.Name().Name, test.ShouldEqual, "teleop")

	st, err := res.DoCommand(context.Background(), map[string]interface{}{"command": "status"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, st["running"], test.ShouldBeFalse)
	test.That(t, running(t, st, "left"), test.ShouldBeFalse)
	left := st["pairs"].(map[string]interface{})["left"].(map[string]interface{})
	test.That(t, left["stopped_reason"], test.ShouldEqual, "")
	test.That(t, left["cycles"], test.ShouldEqual, uint64(0))
	test.That(t, left, test.ShouldNotContainKey, "align_poses")
	test.That(t, arms["left"].ManualCalls, test.ShouldBeEmpty)
	test.That(t, motions["left"].StartCount(), test.ShouldEqual, 0)
}

func TestPairsStartAndStopPerPairAndAll(t *testing.T) {
	ctx := context.Background()
	res, arms, motions, err := twoPairRig(t, logging.NewTestLogger(t), false)
	test.That(t, err, test.ShouldBeNil)
	defer res.Close(ctx)

	st, err := res.DoCommand(ctx, map[string]interface{}{"command": "start", "pair": "left"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, running(t, st, "left"), test.ShouldBeTrue)
	test.That(t, st["pairs"], test.ShouldNotContainKey, "right") // scoped status
	test.That(t, motions["right"].StartCount(), test.ShouldEqual, 0)
	test.That(t, arms["left"].InManualMode(), test.ShouldBeTrue)
	left := st["pairs"].(map[string]interface{})["left"].(map[string]interface{})
	test.That(t, left, test.ShouldContainKey, "align_poses")
	test.That(t, left["leader_held"], test.ShouldBeTrue)

	// "side" is accepted as an alias of "pair".
	st, err = res.DoCommand(ctx, map[string]interface{}{"command": "start", "side": "right"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, running(t, st, "right"), test.ShouldBeTrue)

	st, err = res.DoCommand(ctx, map[string]interface{}{"command": "status"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, st["running"], test.ShouldBeTrue)
	test.That(t, running(t, st, "left") && running(t, st, "right"), test.ShouldBeTrue)

	st, err = res.DoCommand(ctx, map[string]interface{}{"command": "stop", "pair": "left"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, running(t, st, "left"), test.ShouldBeFalse)
	test.That(t, motions["left"].StopCount(), test.ShouldEqual, 1)
	test.That(t, motions["right"].StopCount(), test.ShouldEqual, 0)

	st, err = res.DoCommand(ctx, map[string]interface{}{"command": "stop"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, st["running"], test.ShouldBeFalse)
	test.That(t, motions["right"].StopCount(), test.ShouldEqual, 1)
	test.That(t, arms["right"].InManualMode(), test.ShouldBeFalse)

	_, err = res.DoCommand(ctx, map[string]interface{}{"command": "status", "pair": "middle"})
	test.That(t, err.Error(), test.ShouldContainSubstring, `unknown pair "middle"; configured: [left right]`)
	_, err = res.DoCommand(ctx, map[string]interface{}{"command": "start", "pairs": "left"})
	test.That(t, err.Error(), test.ShouldContainSubstring, `unknown key "pairs"`)
	_, err = res.DoCommand(ctx, map[string]interface{}{"command": "dance"})
	test.That(t, err.Error(), test.ShouldContainSubstring, "expected start, stop or status")
}

func TestStartLogsEachPairsFailureByNameAndStartsTheOthers(t *testing.T) {
	ctx := context.Background()
	logger, logs := logging.NewObservedTestLogger(t)
	res, arms, _, err := twoPairRig(t, logger, false)
	test.That(t, err, test.ShouldBeNil)
	defer res.Close(ctx)
	arms["right"].FailRead = errors.New("checksum")

	_, err = res.DoCommand(ctx, map[string]interface{}{"command": "start"})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "right: ")
	test.That(t, err.Error(), test.ShouldContainSubstring, "checksum")
	test.That(t, logs.FilterMessageSnippet(`pair "right": start failed`).Len(), test.ShouldEqual, 1)
	test.That(t, logs.FilterMessageSnippet(`pair "left"`).Len(), test.ShouldEqual, 0)

	st, err := res.DoCommand(ctx, map[string]interface{}{"command": "status"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, running(t, st, "left"), test.ShouldBeTrue)
	test.That(t, running(t, st, "right"), test.ShouldBeFalse)
}

func TestAFailedAutoStartReleasesWhatDidStart(t *testing.T) {
	// viam-server retries a failing constructor; the pair that started must not be left
	// holding its leader across the retry.
	model := testfake.Model(t)
	joints := []referenceframe.Input{0.1, -0.3, 0.6, 0.2, 0}
	pose, _ := model.Transform(joints)
	left, right := testfake.NewArm(model, joints), testfake.NewArm(model, joints)
	right.FailRead = errors.New("checksum")
	deps := resource.Dependencies{
		arm.Named("left-leader"): left, arm.Named("right-leader"): right,
		motion.Named("builtin"): testfake.NewMotion(pose),
	}
	conf := resource.Config{
		Name: "teleop", API: generic.API, Model: Model,
		ConvertedAttributes: &Config{
			Settings: engine.Settings{LeaderHold: engine.HoldManual}, AutoStart: true,
			Pairs: []map[string]interface{}{
				{"name": "left", "leader_arm": "left-leader", "follower_component": "left-gripper"},
				{"name": "right", "leader_arm": "right-leader", "follower_component": "right-gripper"},
			},
		},
	}
	_, err := newService(context.Background(), deps, conf, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "auto_start failed")
	test.That(t, left.ManualCalls, test.ShouldResemble, []bool{false, true, false})

	// A single-pair config needs no pairs list and is named "main".
	ok := resource.Config{
		Name: "teleop", API: generic.API, Model: Model,
		ConvertedAttributes: &Config{Settings: engine.Settings{LeaderArm: "left-leader", FollowerComponent: "g"}, AutoStart: true},
	}
	res, err := newService(context.Background(), deps, ok, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldBeNil)
	defer res.Close(context.Background())
	st, _ := res.DoCommand(context.Background(), map[string]interface{}{"command": "status"})
	test.That(t, running(t, st, "main"), test.ShouldBeTrue)
	_ = spatialmath.NewZeroPose
}
