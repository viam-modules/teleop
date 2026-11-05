package leaderfollower

import (
	"fmt"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"

	"teleop/internal/filter"
	"teleop/internal/gripper"
	"teleop/internal/hold"
)

// Vec3 is a JSON-friendly millimetre vector.
type Vec3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// Settings is the user-facing configuration, shared verbatim between the module's service
// attributes and the standalone client's config file. Zero values select the defaults noted
// on each field.
type Settings struct {
	// Resource names. The follower component is the frame-system name the pipeline positions
	// (normally the follower's gripper, so its TCP is what tracks the leader).
	LeaderArm         string `json:"leader_arm"`
	LeaderGripper     string `json:"leader_gripper,omitempty"`
	FollowerComponent string `json:"follower_component"`
	FollowerGripper   string `json:"follower_gripper,omitempty"`
	MotionService     string `json:"motion_service,omitempty"` // default "builtin"

	// Mapping. See Mapping.
	Scale             float64                        `json:"scale,omitempty"`                 // default 1
	OriginTranslation *Vec3                          `json:"origin_translation_mm,omitempty"` // leader base in follower world
	OriginOrientation *spatialmath.OrientationConfig `json:"origin_orientation,omitempty"`
	LeaderToolOffset  *Vec3                          `json:"leader_tool_offset_mm,omitempty"` // leader tip in its EndPosition frame, applied before scale
	ToolOrientation   *spatialmath.OrientationConfig `json:"tool_orientation,omitempty"`
	RotationEnabled   *bool                          `json:"rotation_enabled,omitempty"` // default true

	// Loop and filter.
	RateHz         float64  `json:"rate_hz,omitempty"`          // default 60
	SmoothTauMs    *float64 `json:"smooth_tau_ms,omitempty"`    // default 100; 0 disables
	PosDeadzoneMM  float64  `json:"pos_deadzone_mm,omitempty"`  // default 1
	RotDeadzoneDeg float64  `json:"rot_deadzone_deg,omitempty"` // default 1

	// Follower speed caps: how fast the commanded follower pose may move, whatever the
	// leader does and whatever the scale. Always on.
	MaxSpeedMMPerSec float64 `json:"max_speed_mm_per_s,omitempty"`     // default 250
	MaxRotDegPerSec  float64 `json:"max_rotation_deg_per_s,omitempty"` // default 180

	// Alignment.
	AlignOnStart            bool    `json:"align_on_start,omitempty"`              // default false: capture where both arms are, move nothing
	AlignMaxPositionErrorMM float64 `json:"align_max_position_error_mm,omitempty"` // default 0: best effort, any gap is absorbed

	// Leader hold.
	LeaderHold            Hold                   `json:"leader_hold,omitempty"`             // default "none"
	ManualMode            map[string]interface{} `json:"manual_mode,omitempty"`             // extra passed to SetManualMode under "manual"
	LeaderCommands        *LeaderCommands        `json:"leader_commands,omitempty"`         // under "custom"
	LeaderGripperCommands *GripperCommands       `json:"leader_gripper_commands,omitempty"` // any hold

	// Gripper mirroring.
	FollowerGripperMode      gripper.Mode           `json:"follower_gripper_mode,omitempty"`       // default "open_close"
	FollowerGripperCommand   map[string]interface{} `json:"follower_gripper_command,omitempty"`    // template, "command" mode
	GripperRange             float64                `json:"gripper_range,omitempty"`               // $steps at 100 %; default 850
	GripperInvert            bool                   `json:"gripper_invert,omitempty"`              // squeeze-to-close
	LeaderGripperReadCommand map[string]interface{} `json:"leader_gripper_read_command,omitempty"` // required with leader_gripper
	LeaderGripperReadKey     string                 `json:"leader_gripper_read_key,omitempty"`     // dotted path; required with leader_gripper

	removedKeys `json:",squash"`
}

// removedKeys are attributes of the so-101 service this model was carved from that no longer
// exist. They are decoded so a stale config fails with the replacement named, instead of
// running silently without the behaviour it asked for (rdk's attribute decoder ignores
// unknown keys).
type removedKeys struct {
	BlockedFeedback         interface{} `json:"blocked_feedback,omitempty"`
	FeedbackTorqueLimit     interface{} `json:"feedback_torque_limit,omitempty"`
	FeedbackMaxSpeed        interface{} `json:"feedback_max_speed_deg_per_sec,omitempty"`
	FeedbackLagMM           interface{} `json:"feedback_lag_mm,omitempty"`
	FollowerArm             interface{} `json:"follower_arm,omitempty"`
	GripperLock             interface{} `json:"gripper_lock,omitempty"`
	GripperLockMs           interface{} `json:"gripper_lock_ms,omitempty"`
	GripperInMinPct         interface{} `json:"gripper_in_min_pct,omitempty"`
	GripperInMaxPct         interface{} `json:"gripper_in_max_pct,omitempty"`
	GripperDeadbandPct      interface{} `json:"gripper_deadband_pct,omitempty"`
	OutlierPosMMPerSec      interface{} `json:"outlier_pos_mm_per_s,omitempty"`
	OutlierRotDegPerSec     interface{} `json:"outlier_rot_deg_per_s,omitempty"`
	MoveTimeoutMs           interface{} `json:"move_timeout_ms,omitempty"`
	MaxConsecutiveErrors    interface{} `json:"max_consecutive_errors,omitempty"`
	CompensateAlignResidual interface{} `json:"compensate_align_residual,omitempty"`
	AlignOrientationWeight  interface{} `json:"align_orientation_weight_mm_per_rad,omitempty"`
}

// removed maps each removed key to what replaces it.
var removed = []struct {
	key, replacement string
	get              func(removedKeys) interface{}
}{
	{"blocked_feedback", "no replacement; spring feedback was removed with the so-101 manual-hold dependency", func(r removedKeys) interface{} { return r.BlockedFeedback }},
	{"feedback_torque_limit", "no replacement (feedback removed)", func(r removedKeys) interface{} { return r.FeedbackTorqueLimit }},
	{"feedback_max_speed_deg_per_sec", "no replacement (feedback removed)", func(r removedKeys) interface{} { return r.FeedbackMaxSpeed }},
	{"feedback_lag_mm", "no replacement (feedback removed)", func(r removedKeys) interface{} { return r.FeedbackLagMM }},
	{"follower_arm", "no replacement (feedback removed)", func(r removedKeys) interface{} { return r.FollowerArm }},
	{"gripper_lock", "no replacement; gripper lock was removed with the so-101 manual-hold dependency", func(r removedKeys) interface{} { return r.GripperLock }},
	{"gripper_lock_ms", "no replacement (gripper lock removed)", func(r removedKeys) interface{} { return r.GripperLockMs }},
	{"gripper_in_min_pct", "rescale in the leader gripper module, or use gripper_invert", func(r removedKeys) interface{} { return r.GripperInMinPct }},
	{"gripper_in_max_pct", "rescale in the leader gripper module, or use gripper_invert", func(r removedKeys) interface{} { return r.GripperInMaxPct }},
	{"gripper_deadband_pct", "fixed at 3 %", func(r removedKeys) interface{} { return r.GripperDeadbandPct }},
	{"outlier_pos_mm_per_s", "fixed at 4500 mm/s", func(r removedKeys) interface{} { return r.OutlierPosMMPerSec }},
	{"outlier_rot_deg_per_s", "fixed at 1800 deg/s", func(r removedKeys) interface{} { return r.OutlierRotDegPerSec }},
	{"move_timeout_ms", "fixed at 500 ms", func(r removedKeys) interface{} { return r.MoveTimeoutMs }},
	{"max_consecutive_errors", "fixed at 10", func(r removedKeys) interface{} { return r.MaxConsecutiveErrors }},
	{"compensate_align_residual", "always on", func(r removedKeys) interface{} { return r.CompensateAlignResidual }},
	{"align_orientation_weight_mm_per_rad", "fixed at 50", func(r removedKeys) interface{} { return r.AlignOrientationWeight }},
}

// Fixed loop constants that used to be attributes.
const (
	DefaultRateHz         = 60.0
	DefaultSmoothTau      = 100 * time.Millisecond
	DefaultPosDeadzoneMM  = 1.0
	DefaultRotDeadzoneDeg = 1.0
	OutlierPosMMPerSec    = 4500.0
	OutlierRotDegPerSec   = 1800.0
	MoveTimeout           = 500 * time.Millisecond
	MaxConsecutiveErrors  = 10
	GripperDeadbandPct    = 3.0
	IKOrientationWeight   = 50.0
)

// Validate checks the settings. It does not check that resources exist.
func (s Settings) Validate() error {
	for _, r := range removed {
		if r.get(s.removedKeys) != nil {
			return fmt.Errorf("%s is no longer supported by viam:teleop:leader-follower; %s", r.key, r.replacement)
		}
	}
	if s.LeaderArm == "" {
		return fmt.Errorf("leader_arm is required")
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
	if err := hold.ValidateMode(s.LeaderHold); err != nil {
		return err
	}
	if s.ManualMode != nil && s.LeaderHold != HoldManual {
		return fmt.Errorf("manual_mode needs leader_hold %q", HoldManual)
	}
	if s.LeaderCommands != nil && s.LeaderHold != HoldCustom {
		return fmt.Errorf("leader_commands needs leader_hold %q", HoldCustom)
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
	if s.OriginOrientation != nil {
		if _, err := s.OriginOrientation.ParseConfig(); err != nil {
			return fmt.Errorf("origin_orientation: %w", err)
		}
	}
	if s.ToolOrientation != nil {
		if _, err := s.ToolOrientation.ParseConfig(); err != nil {
			return fmt.Errorf("tool_orientation: %w", err)
		}
	}
	return nil
}

// MotionName returns the motion service name, defaulted.
func (s Settings) MotionName() string {
	if s.MotionService == "" {
		return "builtin"
	}
	return s.MotionService
}

// GripperReadConfig is how the leader gripper's percentage is read.
type GripperReadConfig struct {
	Command map[string]interface{}
	Key     string
}

// Config is the resolved engine configuration. Build it with Settings.Resolve.
type Config struct {
	Mapping         Mapping
	RateHz          float64
	RotationEnabled bool
	Filter          filter.Config

	MaxSpeedMMPerSec float64
	MaxRotDegPerSec  float64
	IK               IKOptions

	// AlignOnStart drives the leader to the follower's pose before mirroring (IK + a blocking
	// move). Off by default: the default start captures both arms where they are and mirrors
	// deltas from there, so nothing moves on start and no IK is involved.
	AlignOnStart            bool
	AlignMaxPositionErrorMM float64

	Hold        HoldConfig
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
	m := Mapping{Scale: s.Scale}
	var originOri spatialmath.Orientation = spatialmath.NewZeroOrientation()
	if s.OriginOrientation != nil {
		o, err := s.OriginOrientation.ParseConfig()
		if err != nil {
			return Config{}, err
		}
		originOri = o
	}
	var originPt r3.Vector
	if s.OriginTranslation != nil {
		originPt = r3.Vector{X: s.OriginTranslation.X, Y: s.OriginTranslation.Y, Z: s.OriginTranslation.Z}
	}
	m.Origin = spatialmath.NewPose(originPt, originOri)
	if s.LeaderToolOffset != nil {
		m.LeaderToolOffset = r3.Vector{X: s.LeaderToolOffset.X, Y: s.LeaderToolOffset.Y, Z: s.LeaderToolOffset.Z}
	}
	if s.ToolOrientation != nil {
		o, err := s.ToolOrientation.ParseConfig()
		if err != nil {
			return Config{}, err
		}
		m.ToolRotation = o
	}

	cfg := Config{
		Mapping:          m,
		RateHz:           pick(s.RateHz, DefaultRateHz),
		MaxSpeedMMPerSec: pick(s.MaxSpeedMMPerSec, filter.DefaultMaxSpeedMMPerSec),
		MaxRotDegPerSec:  pick(s.MaxRotDegPerSec, filter.DefaultMaxRotDegPerSec),
		RotationEnabled:  s.RotationEnabled == nil || *s.RotationEnabled,
		Filter: filter.Config{
			SmoothTau:           DefaultSmoothTau,
			OutlierPosMMPerSec:  OutlierPosMMPerSec,
			OutlierRotDegPerSec: OutlierRotDegPerSec,
			DeadzonePosMM:       pick(s.PosDeadzoneMM, DefaultPosDeadzoneMM),
			DeadzoneRotDeg:      pick(s.RotDeadzoneDeg, DefaultRotDeadzoneDeg),
		},
		IK:                      IKOptions{OrientationWeightMMPerRad: IKOrientationWeight},
		AlignOnStart:            s.AlignOnStart,
		AlignMaxPositionErrorMM: s.AlignMaxPositionErrorMM, // 0 = best effort, never refuse
		Hold: HoldConfig{
			Mode:        s.LeaderHold,
			ManualExtra: s.ManualMode,
		},
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
	if s.SmoothTauMs != nil {
		cfg.Filter.SmoothTau = time.Duration(*s.SmoothTauMs * float64(time.Millisecond))
	}
	if cfg.Hold.Mode == "" {
		cfg.Hold.Mode = HoldNone
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
