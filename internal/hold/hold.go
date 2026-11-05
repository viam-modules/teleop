// Package hold is how a leader arm is presented to the operator's hand while a teleop
// session is open: nothing, the arm API's manual mode, or configured DoCommand lists. It is
// shared by every engine that reads a leader arm.
package hold

import (
	"context"
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
)

// Mode is how the leader arm is held.
type Mode string

const (
	// None sends the leader nothing. The default: the arm stays however it was.
	None Mode = "none"
	// Manual uses the arm API's manual mode (arm.SetManualMode): exit it before the session
	// is set up, enter it afterwards, exit it on release. Refused on an arm whose Properties
	// do not report SupportManualMode.
	Manual Mode = "manual"
	// Custom sends the configured DoCommand lists at the same three points, for a leader
	// whose module has no manual-mode API. No command name is defaulted.
	Custom Mode = "custom"
)

// LeaderCommands are ordered DoCommand payloads sent to the leader arm under Custom.
type LeaderCommands struct {
	// BeforeStart runs before the session is set up; a failure is logged at debug level and
	// ignored (its job is to undo a hold a crashed session left behind).
	BeforeStart []map[string]interface{} `json:"before_start,omitempty"`
	// AfterStart runs once the leader is at its session pose; a failure aborts the start and
	// sends OnStop.
	AfterStart []map[string]interface{} `json:"after_start,omitempty"`
	// OnStop runs on release; a failure is warned, not returned.
	OnStop []map[string]interface{} `json:"on_stop,omitempty"`
}

// GripperCommands are ordered DoCommand payloads sent to the leader gripper at start (after
// the leader is at its session pose) and at release, under any mode. The usual use is to
// free a trigger servo so it can be squeezed. Failures are warned, not returned.
type GripperCommands struct {
	OnStart []map[string]interface{} `json:"on_start,omitempty"`
	OnStop  []map[string]interface{} `json:"on_stop,omitempty"`
}

// Config is the resolved hold configuration.
type Config struct {
	Mode        Mode
	ManualExtra map[string]interface{} // extra passed to SetManualMode under Manual
	Leader      LeaderCommands
	Gripper     GripperCommands
}

// ValidateMode checks a configured mode, naming the removed so-101 values.
func ValidateMode(m Mode) error {
	switch m {
	case "", None, Manual, Custom:
		return nil
	case "stiff", "torque_off":
		return fmt.Errorf("leader_hold %q is no longer supported; use %q, %q or %q with leader_commands",
			m, None, Manual, Custom)
	}
	return fmt.Errorf("leader_hold must be %q, %q or %q, got %q", None, Manual, Custom, m)
}

// ValidateLeaderCommands checks every list of c; prefix names the attribute in errors.
func ValidateLeaderCommands(prefix string, c *LeaderCommands) error {
	if c == nil {
		return nil
	}
	if err := ValidateCommandList(prefix+".before_start", c.BeforeStart); err != nil {
		return err
	}
	if err := ValidateCommandList(prefix+".after_start", c.AfterStart); err != nil {
		return err
	}
	return ValidateCommandList(prefix+".on_stop", c.OnStop)
}

// ValidateGripperCommands checks both lists of c.
func ValidateGripperCommands(prefix string, c *GripperCommands) error {
	if c == nil {
		return nil
	}
	if err := ValidateCommandList(prefix+".on_start", c.OnStart); err != nil {
		return err
	}
	return ValidateCommandList(prefix+".on_stop", c.OnStop)
}

// ValidateCommandList rejects empty payloads.
func ValidateCommandList(path string, l []map[string]interface{}) error {
	for i, c := range l {
		if len(c) == 0 {
			return fmt.Errorf("%s[%d]: a DoCommand payload must be a non-empty object", path, i)
		}
	}
	return nil
}

// Leader is the slice of arm.Arm a hold needs.
type Leader interface {
	DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error)
	Properties(ctx context.Context, extra map[string]interface{}) (arm.Properties, error)
	SetManualMode(ctx context.Context, manualMode bool, enabledFor time.Duration, extra map[string]interface{}) error
}

// Commander is the slice of a gripper a hold needs. May be nil.
type Commander interface {
	DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error)
}

// Holder applies and releases one leader's hold.
type Holder struct {
	leader  Leader
	gripper Commander
	cfg     Config
	logger  logging.Logger

	held    atomic.Bool
	mu      sync.Mutex
	lastErr string
}

// New returns a Holder. gripper may be nil; a nil interface stays nil.
func New(leader Leader, gripper Commander, cfg Config, logger logging.Logger) *Holder {
	if cfg.Mode == "" {
		cfg.Mode = None
	}
	return &Holder{leader: leader, gripper: gripper, cfg: cfg, logger: logger}
}

// Mode is the configured mode, defaulted.
func (h *Holder) Mode() Mode { return h.cfg.Mode }

// Held reports whether the leader is currently handed to the operator.
func (h *Holder) Held() bool { return h.held.Load() }

// LastError is the most recent release failure, for status.
func (h *Holder) LastError() string {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.lastErr
}

// BeforeStart undoes whatever hold a previous (possibly crashed) session left on the leader,
// so a compliant arm can be driven for an align move. Best effort: failures are logged at
// debug level and ignored.
func (h *Holder) BeforeStart(ctx context.Context) {
	switch h.cfg.Mode {
	case Manual:
		if err := h.leader.SetManualMode(ctx, false, 0, nil); err != nil {
			h.logger.Debugf("exit manual mode before start: %v", err)
		}
	case Custom:
		for i, cmd := range h.cfg.Leader.BeforeStart {
			if _, err := h.leader.DoCommand(ctx, cmd); err != nil {
				h.logger.Debugf("leader_commands.before_start[%d]: %v", i, err)
			}
		}
	}
}

// Apply hands the leader to the operator once it is at its session pose. A failure means
// the caller should abort and Release. The gripper's on_start list runs first and its
// failures are warned, because a locked trigger is a nuisance and a half-applied arm hold is
// a hazard.
func (h *Holder) Apply(ctx context.Context) error {
	if h.gripper != nil {
		for i, cmd := range h.cfg.Gripper.OnStart {
			if _, err := h.gripper.DoCommand(ctx, cmd); err != nil {
				h.logger.Warnf("leader_gripper_commands.on_start[%d]: %v", i, err)
			}
		}
	}
	switch h.cfg.Mode {
	case Manual:
		props, err := h.leader.Properties(ctx, nil)
		if err != nil {
			return fmt.Errorf("leader properties: %w", err)
		}
		if !props.SupportManualMode {
			return fmt.Errorf("leader_hold %q: the leader arm does not support manual mode; use %q or %q with leader_commands",
				Manual, None, Custom)
		}
		if err := h.leader.SetManualMode(ctx, true, 0, h.cfg.ManualExtra); err != nil {
			return fmt.Errorf("enter leader manual mode: %w", err)
		}
		h.held.Store(true)
	case Custom:
		for i, cmd := range h.cfg.Leader.AfterStart {
			if _, err := h.leader.DoCommand(ctx, cmd); err != nil {
				return fmt.Errorf("leader_commands.after_start[%d]: %w", i, err)
			}
		}
		h.held.Store(len(h.cfg.Leader.AfterStart) > 0)
	}
	return nil
}

// Release undoes Apply. Failures are warned, never returned: by the time this runs the
// session is over and there is nothing the caller can do about a leader that will not let
// go. The last failure is kept for LastError.
func (h *Holder) Release(ctx context.Context) {
	var lastErr error
	switch h.cfg.Mode {
	case Manual:
		if err := h.leader.SetManualMode(ctx, false, 0, nil); err != nil {
			h.logger.Warnf("exit leader manual mode: %v", err)
			lastErr = err
		}
	case Custom:
		for i, cmd := range h.cfg.Leader.OnStop {
			if _, err := h.leader.DoCommand(ctx, cmd); err != nil {
				h.logger.Warnf("leader_commands.on_stop[%d]: %v", i, err)
				lastErr = err
			}
		}
	}
	if h.gripper != nil {
		for i, cmd := range h.cfg.Gripper.OnStop {
			if _, err := h.gripper.DoCommand(ctx, cmd); err != nil {
				h.logger.Warnf("leader_gripper_commands.on_stop[%d]: %v", i, err)
				lastErr = err
			}
		}
	}
	h.held.Store(false)
	if lastErr != nil {
		h.mu.Lock()
		h.lastErr = lastErr.Error()
		h.mu.Unlock()
	}
}
