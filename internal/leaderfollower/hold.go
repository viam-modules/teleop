package leaderfollower

import (
	"context"

	"teleop/internal/hold"
)

// The leader is spoken to at three points of a session, and the leader gripper at two; the
// mechanics live in internal/hold, shared with the clutch engine. These wrappers keep the
// engine's call sites readable.

// beforeStart undoes whatever hold a previous session left on the leader.
func (e *Engine) beforeStart(ctx context.Context) { e.holder.BeforeStart(ctx) }

// applyHold hands the leader to the operator once it is at its session pose. A failure
// aborts Start; the caller releases.
func (e *Engine) applyHold(ctx context.Context) error { return e.holder.Apply(ctx) }

// releaseLeader undoes applyHold. Failures are warned, never returned.
func (e *Engine) releaseLeader(ctx context.Context) { e.holder.Release(ctx) }

// Re-exported so Settings, the client and the tests keep their names.
type (
	// Hold is how the leader arm is presented to the operator's hand once a session starts.
	Hold = hold.Mode
	// LeaderCommands are ordered DoCommand payloads sent to the leader arm under HoldCustom.
	LeaderCommands = hold.LeaderCommands
	// GripperCommands are ordered DoCommand payloads sent to the leader gripper at start and stop.
	GripperCommands = hold.GripperCommands
	// HoldConfig is the resolved leader-hold configuration.
	HoldConfig = hold.Config
)

const (
	HoldNone   = hold.None
	HoldManual = hold.Manual
	HoldCustom = hold.Custom
)
