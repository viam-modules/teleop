package leaderfollower

import (
	"context"
	"fmt"
)

// The leader is spoken to at three points of a session, and the leader gripper at two.
// Under HoldManual the arm API's manual mode is used; under HoldCustom the configured
// DoCommand lists are; under HoldNone nothing is sent to the arm. The gripper lists run under
// any hold, because freeing a trigger servo is independent of how the arm is held.

// beforeStart undoes whatever hold a previous (possibly crashed) session left on the leader,
// so a compliant arm can be driven for the align move. Best effort: failures are logged at
// debug level and ignored.
func (e *Engine) beforeStart(ctx context.Context) {
	switch e.cfg.Hold.Mode {
	case HoldManual:
		if err := e.deps.Leader.SetManualMode(ctx, false, 0, nil); err != nil {
			e.deps.Logger.Debugf("exit manual mode before start: %v", err)
		}
	case HoldCustom:
		for i, cmd := range e.cfg.Hold.Leader.BeforeStart {
			if _, err := e.deps.Leader.DoCommand(ctx, cmd); err != nil {
				e.deps.Logger.Debugf("leader_commands.before_start[%d]: %v", i, err)
			}
		}
	}
}

// applyHold hands the leader to the operator once it is at its session pose. A failure
// aborts Start; the caller releases. The gripper's on_start list runs first and its failures
// are warned, because a locked trigger is a nuisance and a half-applied arm hold is a hazard.
func (e *Engine) applyHold(ctx context.Context) error {
	if e.deps.LeaderGripper != nil {
		for i, cmd := range e.cfg.Hold.Gripper.OnStart {
			if _, err := e.deps.LeaderGripper.DoCommand(ctx, cmd); err != nil {
				e.deps.Logger.Warnf("leader_gripper_commands.on_start[%d]: %v", i, err)
			}
		}
	}
	switch e.cfg.Hold.Mode {
	case HoldManual:
		props, err := e.deps.Leader.Properties(ctx, nil)
		if err != nil {
			return fmt.Errorf("leader properties: %w", err)
		}
		if !props.SupportManualMode {
			return fmt.Errorf("leader_hold %q: the leader arm does not support manual mode; use %q or %q with leader_commands",
				HoldManual, HoldNone, HoldCustom)
		}
		if err := e.deps.Leader.SetManualMode(ctx, true, 0, e.cfg.Hold.ManualExtra); err != nil {
			return fmt.Errorf("enter leader manual mode: %w", err)
		}
		e.leaderHeld.Store(true)
	case HoldCustom:
		for i, cmd := range e.cfg.Hold.Leader.AfterStart {
			if _, err := e.deps.Leader.DoCommand(ctx, cmd); err != nil {
				return fmt.Errorf("leader_commands.after_start[%d]: %w", i, err)
			}
		}
		e.leaderHeld.Store(len(e.cfg.Hold.Leader.AfterStart) > 0)
	}
	return nil
}

// releaseLeader undoes applyHold. Failures are warned, never returned: by the time this runs
// the session is over and there is nothing the caller can do about a leader that will not
// let go. The last failure is kept for status. Caller holds e.mu.
func (e *Engine) releaseLeader(ctx context.Context) {
	var lastErr error
	switch e.cfg.Hold.Mode {
	case HoldManual:
		if err := e.deps.Leader.SetManualMode(ctx, false, 0, nil); err != nil {
			e.deps.Logger.Warnf("exit leader manual mode: %v", err)
			lastErr = err
		}
	case HoldCustom:
		for i, cmd := range e.cfg.Hold.Leader.OnStop {
			if _, err := e.deps.Leader.DoCommand(ctx, cmd); err != nil {
				e.deps.Logger.Warnf("leader_commands.on_stop[%d]: %v", i, err)
				lastErr = err
			}
		}
	}
	if e.deps.LeaderGripper != nil {
		for i, cmd := range e.cfg.Hold.Gripper.OnStop {
			if _, err := e.deps.LeaderGripper.DoCommand(ctx, cmd); err != nil {
				e.deps.Logger.Warnf("leader_gripper_commands.on_stop[%d]: %v", i, err)
				lastErr = err
			}
		}
	}
	e.leaderHeld.Store(false)
	if lastErr != nil {
		e.lastHoldError = lastErr.Error() // caller holds e.mu
	}
}
