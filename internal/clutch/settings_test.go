package clutch

import (
	"testing"

	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"teleop/internal/gripper"
	"teleop/internal/hold"
)

func trackerSettings() Settings {
	return Settings{PoseTracker: "vive", Body: "left", FollowerComponent: "right-gripper"}
}

func armSettings() Settings {
	return Settings{LeaderArm: "so101-left", FollowerComponent: "right-gripper"}
}

func TestDefaultsDependOnTheSource(t *testing.T) {
	cfg, err := trackerSettings().Resolve()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.RateHz, test.ShouldEqual, 90)
	test.That(t, cfg.RotationMode, test.ShouldEqual, RotationAbsolute)
	test.That(t, cfg.Scale, test.ShouldEqual, 1)
	test.That(t, cfg.Filter.SmoothTau, test.ShouldEqual, DefaultSmoothTau)
	test.That(t, cfg.Filter.DeadzonePosMM, test.ShouldEqual, 3)
	test.That(t, cfg.MaxSpeedMMPerSec, test.ShouldEqual, 250)
	test.That(t, cfg.MaxRotDegPerSec, test.ShouldEqual, 180)
	test.That(t, cfg.Hold.Mode, test.ShouldEqual, hold.None)
	test.That(t, cfg.Gripper.Mode, test.ShouldEqual, gripper.ModeOpenClose)

	cfg, err = armSettings().Resolve()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.RateHz, test.ShouldEqual, 60)

	s := armSettings()
	s.RateHz = 30
	s.LeaderHold = hold.Manual
	s.ToolOrientation = &spatialmath.OrientationConfig{Type: spatialmath.OrientationVectorDegreesType,
		Value: map[string]interface{}{"x": 0.0, "y": 0.0, "z": 1.0, "th": 70.0}}
	cfg, err = s.Resolve()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.RateHz, test.ShouldEqual, 30)
	test.That(t, cfg.Hold.Mode, test.ShouldEqual, hold.Manual)
	test.That(t, cfg.ToolRotation, test.ShouldNotBeNil)
}

func TestValidateRequiresExactlyOneSource(t *testing.T) {
	bad := func(s Settings, msg string) {
		t.Helper()
		err := s.Validate()
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, msg)
	}
	bad(Settings{FollowerComponent: "f"}, "a source is required")
	bad(Settings{PoseTracker: "v", FollowerComponent: "f"}, "body is required")
	s := trackerSettings()
	s.LeaderArm = "arm"
	bad(s, "exclusive")
	s = trackerSettings()
	s.FollowerComponent = ""
	bad(s, "follower_component is required")
	s = trackerSettings()
	s.RotationMode = "sideways"
	bad(s, "rotation_mode")
	s = trackerSettings()
	s.MaxSpeedMMPerSec = -1
	bad(s, "cannot be turned off")
}

func TestGripperMirroringIsArmOnly(t *testing.T) {
	bad := func(s Settings, msg string) {
		t.Helper()
		err := s.Validate()
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, msg)
	}
	s := trackerSettings()
	s.LeaderGripper = "lg"
	s.FollowerGripper = "g"
	bad(s, "leader_gripper needs leader_arm")

	s = armSettings()
	s.FollowerGripper = "g"
	bad(s, "must be set together")

	s = armSettings()
	s.LeaderGripper = "lg"
	s.FollowerGripper = "g"
	bad(s, "leader_gripper_read_command is required")
	s.LeaderGripperReadCommand = map[string]interface{}{"command": "get_position"}
	s.LeaderGripperReadKey = "pct"
	test.That(t, s.Validate(), test.ShouldBeNil)
	s.FollowerGripperMode = "command"
	bad(s, "follower_gripper_command")
	s.FollowerGripperCommand = map[string]interface{}{"set": "$steps"}
	test.That(t, s.Validate(), test.ShouldBeNil)
}

func TestHoldNeedsAnArmSource(t *testing.T) {
	s := trackerSettings()
	s.LeaderHold = hold.Manual
	err := s.Validate()
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "need leader_arm")

	s = armSettings()
	s.LeaderHold = "torque_off"
	err = s.Validate()
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "no longer supported")

	s = armSettings()
	s.LeaderCommands = &hold.LeaderCommands{AfterStart: []map[string]interface{}{{"command": "x"}}}
	err = s.Validate()
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "leader_commands needs leader_hold")
	s.LeaderHold = hold.Custom
	test.That(t, s.Validate(), test.ShouldBeNil)
}
