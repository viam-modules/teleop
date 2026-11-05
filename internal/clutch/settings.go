package clutch

import (
	"fmt"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"

	"teleop/internal/filter"
	"teleop/internal/gripper"
	"teleop/internal/hold"
)

// RotationMode is how the hand's orientation reaches the follower.
type RotationMode string

const (
	// RotationAbsolute sends the hand's orientation (in world) composed with tool_orientation.
	// The follower's tool points where the hand points; engaging with the hand held
	// differently from the tool turns the tool to match.
	RotationAbsolute RotationMode = "absolute"
	// RotationRelative sends the follower's engage orientation turned by however much the
	// hand has turned since engage. Nothing jumps at engage.
	RotationRelative RotationMode = "relative"
	// RotationOff holds the follower's engage orientation; only position is mirrored.
	RotationOff RotationMode = "off"
)

// Settings is the user-facing configuration of one pair (one hand, one follower).
//
// The source is exactly one of pose_tracker + body, or leader_arm. There is no clutch input:
// whatever owns the clutch (a footswitch actions service, a wand's grip) sends start and
// stop.
type Settings struct {
	// Source.
	PoseTracker      string `json:"pose_tracker,omitempty"` // with Body
	Body             string `json:"body,omitempty"`
	LeaderArm        string `json:"leader_arm,omitempty"`
	LeaderToolOffset *Vec3  `json:"leader_tool_offset_mm,omitempty"` // arm source: tip in the EndPosition frame

	CaptureSensor string `json:"capture_control_sensor,omitempty"` // start-capture / stop-capture DoCommands at start / stop

	// Follower.
	FollowerComponent string `json:"follower_component"` // required: the frame-system name the pipeline positions
	FollowerGripper   string `json:"follower_gripper,omitempty"`
	MotionService     string `json:"motion_service,omitempty"` // default "builtin"

	// Mapping.
	Scale           float64                        `json:"scale,omitempty"`            // default 1
	RotationMode    RotationMode                   `json:"rotation_mode,omitempty"`    // default absolute
	ToolOrientation *spatialmath.OrientationConfig `json:"tool_orientation,omitempty"` // absolute mode: tool frame rotation applied after the hand's

	// Loop and filter.
	RateHz         float64  `json:"rate_hz,omitempty"`          // default 90 (pose tracker) / 60 (arm)
	SmoothTauMs    *float64 `json:"smooth_tau_ms,omitempty"`    // default 200; 0 disables
	PosDeadzoneMM  float64  `json:"pos_deadzone_mm,omitempty"`  // default 3
	RotDeadzoneDeg float64  `json:"rot_deadzone_deg,omitempty"` // default 3

	// Follower speed caps: how fast the commanded follower pose may move, whatever the hand
	// does and whatever the scale. Always on.
	MaxSpeedMMPerSec float64 `json:"max_speed_mm_per_s,omitempty"`     // default 250
	MaxRotDegPerSec  float64 `json:"max_rotation_deg_per_s,omitempty"` // default 180

	// Leader hold (arm source only). Applied on start, released on stop.
	LeaderHold            hold.Mode              `json:"leader_hold,omitempty"`
	ManualMode            map[string]interface{} `json:"manual_mode,omitempty"`
	LeaderCommands        *hold.LeaderCommands   `json:"leader_commands,omitempty"`
	LeaderGripperCommands *hold.GripperCommands  `json:"leader_gripper_commands,omitempty"`

	// Gripper mirroring (arm source): the leader gripper drives the follower gripper while
	// started, as in leader-follower.
	FollowerGripperMode      gripper.Mode           `json:"follower_gripper_mode,omitempty"` // default "open_close"
	FollowerGripperCommand   map[string]interface{} `json:"follower_gripper_command,omitempty"`
	GripperRange             float64                `json:"gripper_range,omitempty"` // default 850
	GripperInvert            bool                   `json:"gripper_invert,omitempty"`
	LeaderGripper            string                 `json:"leader_gripper,omitempty"` // arm source
	LeaderGripperReadCommand map[string]interface{} `json:"leader_gripper_read_command,omitempty"`
	LeaderGripperReadKey     string                 `json:"leader_gripper_read_key,omitempty"`
}

// Vec3 is a JSON-friendly millimetre vector.
type Vec3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// ToolOffset is the leader tool offset as a vector, zero if unset.
func (s Settings) ToolOffset() r3.Vector {
	if s.LeaderToolOffset == nil {
		return r3.Vector{}
	}
	return r3.Vector{X: s.LeaderToolOffset.X, Y: s.LeaderToolOffset.Y, Z: s.LeaderToolOffset.Z}
}

// Fixed loop constants.
const (
	DefaultTrackerRateHz  = 90.0
	DefaultArmRateHz      = 60.0
	DefaultSmoothTau      = 200 * time.Millisecond
	DefaultPosDeadzoneMM  = 3.0
	DefaultRotDeadzoneDeg = 3.0
	OutlierPosMMPerSec    = 4500.0
	OutlierRotDegPerSec   = 2700.0
	MoveTimeout           = 500 * time.Millisecond
	MaxConsecutiveErrors  = 10
	GripperDeadbandPct    = 3.0
)

// HasArmSource reports whether the pair reads a leader arm (else a pose tracker).
func (s Settings) HasArmSource() bool { return s.LeaderArm != "" }

// MotionName returns the motion service name, defaulted.
func (s Settings) MotionName() string {
	if s.MotionService == "" {
		return "builtin"
	}
	return s.MotionService
}

// Validate checks the settings. It does not check that resources exist.
func (s Settings) Validate() error {
	switch {
	case s.LeaderArm != "" && (s.PoseTracker != "" || s.Body != ""):
		return fmt.Errorf("leader_arm and pose_tracker/body are exclusive; a pair has one source")
	case s.LeaderArm == "" && s.PoseTracker == "":
		return fmt.Errorf("a source is required: pose_tracker with body, or leader_arm")
	case s.PoseTracker != "" && s.Body == "":
		return fmt.Errorf("body is required with pose_tracker")
	}
	if s.FollowerComponent == "" {
		return fmt.Errorf("follower_component is required")
	}
	if s.Scale < 0 {
		return fmt.Errorf("scale must be positive, got %v", s.Scale)
	}
	if s.RateHz < 0 || s.RateHz > 1000 {
		return fmt.Errorf("rate_hz must be in (0, 1000], got %v", s.RateHz)
	}
	if s.SmoothTauMs != nil && *s.SmoothTauMs < 0 {
		return fmt.Errorf("smooth_tau_ms must not be negative")
	}
	if s.MaxSpeedMMPerSec < 0 || s.MaxRotDegPerSec < 0 {
		return fmt.Errorf("max_speed_mm_per_s and max_rotation_deg_per_s must be positive; the caps cannot be turned off")
	}
	switch s.RotationMode {
	case "", RotationAbsolute, RotationRelative, RotationOff:
	default:
		return fmt.Errorf("rotation_mode must be %q, %q or %q, got %q",
			RotationAbsolute, RotationRelative, RotationOff, s.RotationMode)
	}
	if s.ToolOrientation != nil {
		if _, err := s.ToolOrientation.ParseConfig(); err != nil {
			return fmt.Errorf("tool_orientation: %w", err)
		}
	}

	// Hold: arm source only.
	if err := hold.ValidateMode(s.LeaderHold); err != nil {
		return err
	}
	if !s.HasArmSource() && s.LeaderToolOffset != nil {
		return fmt.Errorf("leader_tool_offset_mm needs leader_arm")
	}
	if !s.HasArmSource() && (s.LeaderHold != "" || s.ManualMode != nil || s.LeaderCommands != nil || s.LeaderGripperCommands != nil) {
		return fmt.Errorf("leader_hold, manual_mode, leader_commands and leader_gripper_commands need leader_arm")
	}
	if s.ManualMode != nil && s.LeaderHold != hold.Manual {
		return fmt.Errorf("manual_mode needs leader_hold %q", hold.Manual)
	}
	if s.LeaderCommands != nil && s.LeaderHold != hold.Custom {
		return fmt.Errorf("leader_commands needs leader_hold %q", hold.Custom)
	}
	if err := hold.ValidateLeaderCommands("leader_commands", s.LeaderCommands); err != nil {
		return err
	}
	if s.LeaderGripperCommands != nil && s.LeaderGripper == "" {
		return fmt.Errorf("leader_gripper_commands needs leader_gripper")
	}
	if err := hold.ValidateGripperCommands("leader_gripper_commands", s.LeaderGripperCommands); err != nil {
		return err
	}

	// Gripper: arm source only, leader and follower together.
	if s.LeaderGripper != "" && !s.HasArmSource() {
		return fmt.Errorf("leader_gripper needs leader_arm")
	}
	if (s.LeaderGripper == "") != (s.FollowerGripper == "") {
		return fmt.Errorf("leader_gripper and follower_gripper must be set together")
	}
	if s.LeaderGripper != "" {
		if len(s.LeaderGripperReadCommand) == 0 {
			return fmt.Errorf("leader_gripper_read_command is required when leader_gripper is set")
		}
		if s.LeaderGripperReadKey == "" {
			return fmt.Errorf("leader_gripper_read_key is required when leader_gripper is set")
		}
	} else if len(s.LeaderGripperReadCommand) != 0 || s.LeaderGripperReadKey != "" {
		return fmt.Errorf("leader_gripper_read_command and leader_gripper_read_key need leader_gripper")
	}
	mode, _ := gripper.NormalizeMode(s.FollowerGripperMode)
	switch mode {
	case "", gripper.ModeOpenClose, gripper.ModeOff:
	case gripper.ModeCommand:
		if err := gripper.ValidateTemplate(s.FollowerGripperCommand); err != nil {
			return err
		}
	default:
		return fmt.Errorf("follower_gripper_mode must be %q, %q or %q, got %q",
			gripper.ModeCommand, gripper.ModeOpenClose, gripper.ModeOff, s.FollowerGripperMode)
	}
	if s.FollowerGripperCommand != nil && mode != gripper.ModeCommand {
		return fmt.Errorf("follower_gripper_command needs follower_gripper_mode %q", gripper.ModeCommand)
	}
	return nil
}

// GripperReadConfig is how the leader gripper's percentage is read (arm source).
type GripperReadConfig struct {
	Command map[string]interface{}
	Key     string
}

// Config is the resolved engine configuration. Build it with Settings.Resolve.
type Config struct {
	Scale        float64
	RotationMode RotationMode
	ToolRotation spatialmath.Orientation // nil = identity

	RateHz float64
	Filter filter.Config

	MaxSpeedMMPerSec float64
	MaxRotDegPerSec  float64

	Hold        hold.Config
	Gripper     gripper.OutputConfig
	GripperRead GripperReadConfig

	MoveTimeout          time.Duration
	MaxConsecutiveErrors int
}

// Resolve turns the settings into an engine Config with every default applied.
func (s Settings) Resolve() (Config, error) {
	if err := s.Validate(); err != nil {
		return Config{}, err
	}
	rate := DefaultTrackerRateHz
	if s.HasArmSource() {
		rate = DefaultArmRateHz
	}
	cfg := Config{
		Scale:            pick(s.Scale, 1),
		RotationMode:     s.RotationMode,
		RateHz:           pick(s.RateHz, rate),
		MaxSpeedMMPerSec: pick(s.MaxSpeedMMPerSec, filter.DefaultMaxSpeedMMPerSec),
		MaxRotDegPerSec:  pick(s.MaxRotDegPerSec, filter.DefaultMaxRotDegPerSec),
		Filter: filter.Config{
			SmoothTau:           DefaultSmoothTau,
			OutlierPosMMPerSec:  OutlierPosMMPerSec,
			OutlierRotDegPerSec: OutlierRotDegPerSec,
			DeadzonePosMM:       pick(s.PosDeadzoneMM, DefaultPosDeadzoneMM),
			DeadzoneRotDeg:      pick(s.RotDeadzoneDeg, DefaultRotDeadzoneDeg),
		},
		Hold: hold.Config{Mode: s.LeaderHold, ManualExtra: s.ManualMode},
		Gripper: gripper.OutputConfig{
			Mode:        s.FollowerGripperMode,
			Template:    s.FollowerGripperCommand,
			Range:       s.GripperRange,
			Invert:      s.GripperInvert,
			DeadbandPct: GripperDeadbandPct,
		},
		GripperRead:          GripperReadConfig{Command: s.LeaderGripperReadCommand, Key: s.LeaderGripperReadKey},
		MoveTimeout:          MoveTimeout,
		MaxConsecutiveErrors: MaxConsecutiveErrors,
	}
	if cfg.RotationMode == "" {
		cfg.RotationMode = RotationAbsolute
	}
	if s.ToolOrientation != nil {
		o, err := s.ToolOrientation.ParseConfig()
		if err != nil {
			return Config{}, err
		}
		cfg.ToolRotation = o
	}
	if s.SmoothTauMs != nil {
		cfg.Filter.SmoothTau = time.Duration(*s.SmoothTauMs * float64(time.Millisecond))
	}
	if cfg.Hold.Mode == "" {
		cfg.Hold.Mode = hold.None
	}
	if s.LeaderCommands != nil {
		cfg.Hold.Leader = *s.LeaderCommands
	}
	if s.LeaderGripperCommands != nil {
		cfg.Hold.Gripper = *s.LeaderGripperCommands
	}
	if cfg.Gripper.Mode == "" {
		cfg.Gripper.Mode = gripper.ModeOpenClose
	}
	cfg.Gripper.Mode, _ = gripper.NormalizeMode(cfg.Gripper.Mode)
	return cfg, nil
}

func pick(v, def float64) float64 {
	if v == 0 {
		return def
	}
	return v
}
