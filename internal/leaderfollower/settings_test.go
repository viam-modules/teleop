package leaderfollower

import (
	"encoding/json"
	"testing"
	"time"

	"go.viam.com/test"

	"teleop/internal/gripper"
)

func TestResolveAppliesDefaults(t *testing.T) {
	cfg, err := Settings{LeaderArm: "l", FollowerComponent: "f"}.Resolve()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.RateHz, test.ShouldEqual, 60)
	test.That(t, cfg.RotationEnabled, test.ShouldBeTrue)
	test.That(t, cfg.Filter.SmoothTau, test.ShouldEqual, 100*time.Millisecond)
	test.That(t, cfg.Filter.DeadzonePosMM, test.ShouldEqual, 1)
	test.That(t, cfg.Filter.OutlierPosMMPerSec, test.ShouldEqual, 4500)
	test.That(t, cfg.Hold.Mode, test.ShouldEqual, HoldNone)
	test.That(t, cfg.Gripper.Mode, test.ShouldEqual, gripper.ModeOpenClose)
	test.That(t, cfg.Gripper.DeadbandPct, test.ShouldEqual, 3)
	test.That(t, cfg.MoveTimeout, test.ShouldEqual, 500*time.Millisecond)
	test.That(t, cfg.MaxConsecutiveErrors, test.ShouldEqual, 10)
	test.That(t, cfg.Mapping.Scale, test.ShouldEqual, 0) // Mapping defaults 0 to 1 itself
	test.That(t, cfg.AlignOnStart, test.ShouldBeFalse)

	zero := 0.0
	cfg, err = Settings{LeaderArm: "l", FollowerComponent: "f", SmoothTauMs: &zero, RateHz: 90}.Resolve()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.Filter.SmoothTau, test.ShouldEqual, 0)
	test.That(t, cfg.RateHz, test.ShouldEqual, 90)
}

func TestRemovedKeysAreRejectedByName(t *testing.T) {
	// A stale config from the so-101 service, as JSON, because that is how it arrives.
	stale := `{"leader_arm": "l", "follower_component": "f", "gripper_lock": true}`
	var s Settings
	test.That(t, json.Unmarshal([]byte(stale), &s), test.ShouldBeNil)
	err := s.Validate()
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "gripper_lock is no longer supported")

	for _, key := range []string{"blocked_feedback", "gripper_in_max_pct", "outlier_pos_mm_per_s", "max_consecutive_errors", "compensate_align_residual", "follower_arm"} {
		var s Settings
		test.That(t, json.Unmarshal([]byte(`{"leader_arm": "l", "follower_component": "f", "`+key+`": 1}`), &s), test.ShouldBeNil)
		err := s.Validate()
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, key)
	}

	for _, hold := range []Hold{"stiff", "torque_off"} {
		err := Settings{LeaderArm: "l", FollowerComponent: "f", LeaderHold: hold}.Validate()
		test.That(t, err.Error(), test.ShouldContainSubstring, "no longer supported")
	}
}

func TestHoldValidation(t *testing.T) {
	base := Settings{LeaderArm: "l", FollowerComponent: "f"}

	s := base
	s.LeaderHold = "soft"
	test.That(t, s.Validate(), test.ShouldNotBeNil)

	s = base
	s.ManualMode = map[string]interface{}{"p_gain": 4.0}
	test.That(t, s.Validate().Error(), test.ShouldContainSubstring, "manual_mode needs leader_hold")
	s.LeaderHold = HoldManual
	test.That(t, s.Validate(), test.ShouldBeNil)

	s = base
	s.LeaderCommands = &LeaderCommands{AfterStart: []map[string]interface{}{{"command": "x"}}}
	test.That(t, s.Validate().Error(), test.ShouldContainSubstring, "leader_commands needs leader_hold")
	s.LeaderHold = HoldCustom
	test.That(t, s.Validate(), test.ShouldBeNil)
	s.LeaderCommands.OnStop = []map[string]interface{}{{}}
	test.That(t, s.Validate().Error(), test.ShouldContainSubstring, "leader_commands.on_stop[0]")

	s = base
	s.LeaderGripperCommands = &GripperCommands{OnStart: []map[string]interface{}{{"command": "x"}}}
	test.That(t, s.Validate().Error(), test.ShouldContainSubstring, "needs leader_gripper")

	cfg, err := Settings{LeaderArm: "l", FollowerComponent: "f", LeaderHold: HoldCustom,
		LeaderCommands: &LeaderCommands{OnStop: []map[string]interface{}{{"command": "release"}}}}.Resolve()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.Hold.Leader.OnStop[0]["command"], test.ShouldEqual, "release")
}

func TestGripperValidation(t *testing.T) {
	base := Settings{LeaderArm: "l", FollowerComponent: "f"}

	s := base
	s.LeaderGripper = "lg"
	test.That(t, s.Validate().Error(), test.ShouldContainSubstring, "set together")
	s.FollowerGripper = "fg"
	test.That(t, s.Validate().Error(), test.ShouldContainSubstring, "leader_gripper_read_command is required")
	s.LeaderGripperReadCommand = map[string]interface{}{"command": "get_position"}
	test.That(t, s.Validate().Error(), test.ShouldContainSubstring, "leader_gripper_read_key is required")
	s.LeaderGripperReadKey = "position_percentage"
	test.That(t, s.Validate(), test.ShouldBeNil)

	s.FollowerGripperMode = gripper.ModeCommand
	test.That(t, s.Validate().Error(), test.ShouldContainSubstring, "follower_gripper_command is required")
	s.FollowerGripperCommand = map[string]interface{}{"set": 5.0}
	test.That(t, s.Validate().Error(), test.ShouldContainSubstring, "carries neither")
	s.FollowerGripperCommand = map[string]interface{}{"set": "$steps"}
	test.That(t, s.Validate(), test.ShouldBeNil)

	// The migration alias resolves to command mode.
	s.FollowerGripperMode = "xarm"
	cfg, err := s.Resolve()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.Gripper.Mode, test.ShouldEqual, gripper.ModeCommand)
	test.That(t, cfg.GripperRead.Key, test.ShouldEqual, "position_percentage")

	s.FollowerGripperMode = gripper.ModeOpenClose
	test.That(t, s.Validate().Error(), test.ShouldContainSubstring, "follower_gripper_command needs")

	s = base
	s.LeaderGripperReadKey = "x"
	test.That(t, s.Validate().Error(), test.ShouldContainSubstring, "need leader_gripper")
}
