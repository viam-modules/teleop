package clutch

import (
	"context"
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"teleop/internal/filter"
	"teleop/internal/gripper"
	"teleop/internal/hold"
	"teleop/internal/pipeline"
	"teleop/internal/posemath"
)

// Frames is the slice of the frame system service the engine needs. framesystem.Service
// satisfies it.
type Frames interface {
	TransformPose(ctx context.Context, pose *referenceframe.PoseInFrame, dst string,
		supplementalTransforms []*referenceframe.LinkInFrame) (*referenceframe.PoseInFrame, error)
}

// Commander is anything the engine only sends DoCommands to.
type Commander interface {
	DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error)
}

// Deps are the resources the engine drives. Frames nil means the source frame is taken to
// be world. Leader is the arm behind an ArmSource, for the hold; nil for a pose tracker.
// LeaderGripper, FollowerGripper and Capture may be nil; Now may be nil for the wall clock.
type Deps struct {
	Source          Source
	Frames          Frames
	Pipeline        *pipeline.Pipeline
	Leader          hold.Leader
	LeaderGripper   gripper.Gripper
	FollowerGripper gripper.Gripper
	Capture         Commander
	Logger          logging.Logger
	Now             func() time.Time
}

// StopReason says why the last session ended.
type StopReason string

const (
	StopReasonNone        StopReason = ""
	StopReasonStopped     StopReason = "stopped"      // Stop was called (the clutch was let go)
	StopReasonUntracked   StopReason = "untracked"    // the hand lost tracking
	StopReasonSessionLost StopReason = "session_lost" // the pipeline no longer knows the follower
	StopReasonMaxErrors   StopReason = "max_errors"   // too many consecutive read or send failures
	// StopReasonPipelineError: the motion service's teleop pipeline reported an error (a
	// planner failure, or an arm fault such as an emergency stop). The sends keep succeeding
	// when this happens, because teleop_move only stores a pose, so the status poll is the
	// only place it shows.
	StopReasonPipelineError StopReason = "pipeline_error"
)

// Status is a snapshot for DoCommand/CLI reporting.
type Status struct {
	Running           bool         `json:"running"` // the follower is being driven
	SessionID         string       `json:"session_id"`
	Sessions          uint64       `json:"sessions"`
	StoppedReason     StopReason   `json:"stopped_reason"`
	Tracked           bool         `json:"tracked"`
	LeaderHeld        bool         `json:"leader_held"`
	LastHoldError     string       `json:"last_hold_error"`
	Cycles            uint64       `json:"cycles"`
	Sends             uint64       `json:"sends"`
	Dropped           uint64       `json:"dropped"`
	SpeedLimited      uint64       `json:"speed_limited"` // ticks where a speed cap held the follower back
	Deadzoned         uint64       `json:"deadzoned"`
	Rejected          uint64       `json:"rejected"`
	TickOverruns      uint64       `json:"tick_overruns"`
	ConsecutiveErrors int          `json:"consecutive_errors"`
	ErrorsTotal       uint64       `json:"errors_total"`
	LastError         string       `json:"last_error"`
	LastErrorAt       string       `json:"last_error_at"`
	LastRPCMs         float64      `json:"last_rpc_ms"`
	SourceReadMs      float64      `json:"source_read_ms"`
	GripperReadMs     float64      `json:"gripper_read_ms"`
	RateHz            float64      `json:"rate_hz"`
	RotationMode      RotationMode `json:"rotation_mode"`
	GripperPct        float64      `json:"gripper_pct"`
	GripperWrites     uint64       `json:"gripper_writes"`
	PipelineError     string       `json:"pipeline_error"`
	PipelinePlanMs    float64      `json:"pipeline_plan_ms"`
}

const (
	statusPollInterval = 200 * time.Millisecond
	poseLogInterval    = time.Second
	stopTimeout        = 5 * time.Second
)

// Engine runs one hand. Start is the clutch going down: it captures where the hand and the
// follower are and mirrors the hand's motion from there. Stop is the clutch coming up: the
// follower stops where it is. The leader hold is separate (HoldLeader / ReleaseLeader): a
// leader arm stays limp between drags, which is the point of a clutch.
type Engine struct {
	deps Deps
	cfg  Config

	sender  *pipeline.Sender
	limiter *filter.Limiter
	out     *gripper.Output
	reader  *gripper.Reader // arm source with a leader gripper
	holder  *hold.Holder    // arm source

	mu sync.Mutex
	// running is true while the loop drives the follower.
	running      bool
	registered   bool // the follower is registered with the pipeline
	cancel       context.CancelFunc
	wg           sync.WaitGroup
	ref          *Reference
	filter       *filter.Filter
	goal         spatialmath.Pose // latest filtered target; the limiter walks toward it
	rotationMode RotationMode
	sessionID    string
	sessions     uint64
	stopReason   StopReason

	cycles, tickOverruns, errorsTotal uint64
	consecutiveErrors                 int
	lastError                         string
	lastErrorAt                       time.Time
	pipelineError                     string
	pipelinePlanMs                    float64

	pipelineFault    atomic.Pointer[string] // set by pollStatus, acted on by tick
	tracked          atomic.Bool
	sourceReadNanos  atomic.Int64
	gripperReadNanos atomic.Int64
	poseLogAt        atomic.Int64
	frameWarned      map[string]bool
}

// New returns an Engine. It does nothing until Start.
func New(deps Deps, cfg Config) (*Engine, error) {
	if deps.Logger == nil {
		deps.Logger = logging.NewLogger("clutch")
	}
	if deps.Now == nil {
		deps.Now = time.Now
	}
	if deps.Source == nil || deps.Pipeline == nil {
		return nil, fmt.Errorf("clutch engine needs a source and a pipeline")
	}
	if cfg.RateHz <= 0 {
		cfg.RateHz = DefaultTrackerRateHz
	}
	if cfg.Scale <= 0 {
		cfg.Scale = 1
	}
	if cfg.RotationMode == "" {
		cfg.RotationMode = RotationAbsolute
	}
	if cfg.MaxConsecutiveErrors <= 0 {
		cfg.MaxConsecutiveErrors = MaxConsecutiveErrors
	}
	if cfg.MoveTimeout <= 0 {
		cfg.MoveTimeout = MoveTimeout
	}
	if cfg.Gripper.Mode == "" {
		cfg.Gripper.Mode = gripper.ModeOpenClose
	}
	if cfg.MaxSpeedMMPerSec <= 0 {
		cfg.MaxSpeedMMPerSec = filter.DefaultMaxSpeedMMPerSec
	}
	if cfg.MaxRotDegPerSec <= 0 {
		cfg.MaxRotDegPerSec = filter.DefaultMaxRotDegPerSec
	}
	if cfg.Hold.Mode != "" && cfg.Hold.Mode != hold.None && deps.Leader == nil {
		return nil, fmt.Errorf("leader_hold %q needs a leader arm", cfg.Hold.Mode)
	}
	e := &Engine{deps: deps, cfg: cfg, rotationMode: cfg.RotationMode, frameWarned: map[string]bool{}}
	e.sender = pipeline.NewSender(deps.Pipeline, cfg.MoveTimeout)
	e.limiter = filter.NewLimiter(cfg.MaxSpeedMMPerSec, cfg.MaxRotDegPerSec)
	e.out = gripper.NewOutput(deps.FollowerGripper, cfg.Gripper)
	if deps.LeaderGripper != nil && e.out.Enabled() {
		r, err := gripper.NewReader(deps.LeaderGripper, cfg.GripperRead.Command, cfg.GripperRead.Key)
		if err != nil {
			return nil, err
		}
		e.reader = r
	}
	if deps.Leader != nil {
		var lg hold.Commander
		if deps.LeaderGripper != nil {
			lg = deps.LeaderGripper
		}
		e.holder = hold.New(deps.Leader, lg, cfg.Hold, deps.Logger)
	}
	return e, nil
}

// HoldLeader applies the leader hold (arm source): the leader goes limp, and stays limp
// across starts and stops until ReleaseLeader. A no-op without a leader or with hold none.
func (e *Engine) HoldLeader(ctx context.Context) error {
	if e.holder == nil {
		return nil
	}
	e.holder.BeforeStart(ctx)
	if err := e.holder.Apply(ctx); err != nil {
		e.holder.Release(ctx)
		return err
	}
	return nil
}

// ReleaseLeader undoes HoldLeader. Failures are logged, not returned.
func (e *Engine) ReleaseLeader(ctx context.Context) {
	if e.holder != nil {
		e.holder.Release(ctx)
	}
}

// Start engages: captures the hand's pose and the follower's pose as this session's
// references, registers the follower with the pipeline at its own pose so nothing moves,
// and starts mirroring. Starting a running engine does nothing, so a repeated start never
// re-captures mid-drag.
func (e *Engine) Start(ctx context.Context) error {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.running {
		return nil
	}
	if e.registered {
		// The loop ended on its own and its teleop_stop has not run yet.
		_ = e.closeSessionLocked(ctx)
	}
	fail := func(err error) error { return err }
	if e.reader != nil {
		// One read now, so a wrong command or key fails Start with a message that names it.
		if _, err := e.reader.Read(ctx); err != nil {
			return fail(err)
		}
	}
	sample, err := e.deps.Source.Read(ctx)
	if err != nil {
		return fail(err)
	}
	e.tracked.Store(sample.Tracked)
	if !sample.Tracked {
		return fail(fmt.Errorf("%s is not tracked", e.deps.Source))
	}
	follower, err := e.deps.Pipeline.CurrentPose(ctx)
	if err != nil {
		return fail(fmt.Errorf("read follower pose: %w", err))
	}
	rot := e.sourceRotationLocked(ctx, sample.Frame)
	// A pipeline that is already failing (another pair's arm faulted, a planner error) is
	// not one to add a follower to.
	if st, err := e.deps.Pipeline.Status(ctx); err == nil {
		if perr, _ := st["error"].(string); perr != "" {
			return fail(fmt.Errorf("the teleop pipeline reports an error; not starting: %s", perr))
		}
	}
	if err := e.deps.Pipeline.Start(ctx, follower); err != nil {
		return fail(err)
	}

	e.sessions++
	e.sessionID = fmt.Sprintf("%d-%d", e.deps.Now().Unix(), e.sessions)
	e.ref = NewReference(follower, sample.Pose, rot, sample.Frame)
	e.filter = filter.New(e.cfg.Filter)
	e.goal = follower
	e.limiter.Reset(follower, e.deps.Now())
	e.pipelineFault.Store(nil)
	e.sender.Reset()
	e.out.Reset()
	e.cycles, e.tickOverruns, e.errorsTotal = 0, 0, 0
	e.consecutiveErrors, e.lastError, e.lastErrorAt = 0, "", time.Time{}
	e.pipelineError, e.pipelinePlanMs = "", 0
	e.stopReason = StopReasonNone
	e.registered, e.running = true, true
	if e.deps.Capture != nil {
		if _, err := e.deps.Capture.DoCommand(ctx, map[string]interface{}{"start-capture": true}); err != nil {
			e.deps.Logger.Warnf("capture start: %v", err)
		}
	}

	loopCtx, cancel := context.WithCancel(context.Background())
	e.cancel = cancel
	e.wg.Add(2)
	go e.runLoop(loopCtx)
	go e.pollStatus(loopCtx)
	e.deps.Logger.Infow("clutch engaged",
		"session", e.sessionID,
		"source", e.deps.Source.String(),
		"follower", posemath.PoseMap(follower),
		"hand", posemath.PoseMap(sample.Pose),
		"frame", sample.Frame,
		"source_rotation", posemath.PoseMap(spatialmath.NewPoseFromOrientation(orDefault(rot))),
		"rotation_mode", string(e.rotationMode),
		"rate_hz", e.cfg.RateHz,
	)
	return nil
}

// Stop releases: the follower stops where it is. The leader hold is untouched. Stopping a
// stopped engine does nothing.
func (e *Engine) Stop(ctx context.Context) error {
	e.mu.Lock()
	defer e.mu.Unlock()
	if !e.running && !e.registered {
		return nil
	}
	err := e.closeSessionLocked(ctx)
	e.stopReason = StopReasonStopped
	return err
}

// closeSessionLocked stops the loop and removes the follower from the pipeline. Caller holds
// e.mu; it is released while the loop drains.
func (e *Engine) closeSessionLocked(ctx context.Context) error {
	if e.cancel != nil {
		e.cancel()
	}
	e.mu.Unlock()
	e.wg.Wait()
	e.sender.Wait()
	e.out.Wait()
	e.mu.Lock()
	err := e.unregisterLocked(ctx)
	e.running = false
	return err
}

// unregisterLocked sends teleop_stop once per session and closes capture. The follower
// holds where it is.
func (e *Engine) unregisterLocked(ctx context.Context) error {
	if !e.registered {
		return nil
	}
	e.registered = false
	var err error
	if perr := e.deps.Pipeline.Stop(ctx); perr != nil && !pipeline.IsSessionLost(perr) {
		err = perr
		e.deps.Logger.Warnf("teleop_stop: %v", perr)
	}
	if e.deps.Capture != nil {
		if _, cerr := e.deps.Capture.DoCommand(ctx, map[string]interface{}{"stop-capture": true}); cerr != nil {
			e.deps.Logger.Warnf("capture stop: %v", cerr)
		}
	}
	snd := e.sender.Snapshot()
	var fc filter.Counters
	if e.filter != nil {
		fc = e.filter.Counters()
	}
	e.deps.Logger.Infow("clutch released",
		"session", e.sessionID,
		"cycles", e.cycles, "sends", snd.Sends, "dropped", snd.Dropped,
		"deadzoned", fc.Deadzoned, "rejected", fc.Rejected,
		"follower_cmd", posemath.PoseMap(e.sender.LastSent()),
	)
	return err
}

// SetRotationMode changes the rotation mode; it takes effect on the next tick, mid-session
// included.
func (e *Engine) SetRotationMode(m RotationMode) error {
	switch m {
	case RotationAbsolute, RotationRelative, RotationOff:
	default:
		return fmt.Errorf("rotation_mode must be %q, %q or %q, got %q", RotationAbsolute, RotationRelative, RotationOff, m)
	}
	e.mu.Lock()
	e.rotationMode = m
	e.mu.Unlock()
	e.deps.Logger.Infof("rotation mode %s", m)
	return nil
}

// Status returns a snapshot.
func (e *Engine) Status() Status {
	snd := e.sender.Snapshot()
	e.mu.Lock()
	defer e.mu.Unlock()
	st := Status{
		Running:           e.running,
		SessionID:         e.sessionID,
		Sessions:          e.sessions,
		StoppedReason:     e.stopReason,
		Tracked:           e.tracked.Load(),
		Cycles:            e.cycles,
		Sends:             snd.Sends,
		Dropped:           snd.Dropped,
		SpeedLimited:      e.limiter.Limited(),
		TickOverruns:      e.tickOverruns,
		ConsecutiveErrors: e.consecutiveErrors,
		ErrorsTotal:       e.errorsTotal,
		LastError:         e.lastError,
		LastRPCMs:         snd.LastRPCMs,
		SourceReadMs:      float64(e.sourceReadNanos.Load()) / 1e6,
		GripperReadMs:     float64(e.gripperReadNanos.Load()) / 1e6,
		RateHz:            e.cfg.RateHz,
		RotationMode:      e.rotationMode,
		GripperPct:        e.out.Pct(),
		GripperWrites:     e.out.Writes(),
		PipelineError:     e.pipelineError,
		PipelinePlanMs:    e.pipelinePlanMs,
	}
	if e.holder != nil {
		st.LeaderHeld, st.LastHoldError = e.holder.Held(), e.holder.LastError()
	}
	if !e.lastErrorAt.IsZero() {
		st.LastErrorAt = e.lastErrorAt.Format(time.RFC3339)
	}
	if e.filter != nil {
		c := e.filter.Counters()
		st.Deadzoned, st.Rejected = c.Deadzoned, c.Rejected
	}
	return st
}

func (e *Engine) runLoop(ctx context.Context) {
	defer e.wg.Done()
	period := time.Duration(float64(time.Second) / e.cfg.RateHz)
	ticker := time.NewTicker(period)
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			start := time.Now()
			reason := e.tick(ctx)
			if time.Since(start) > period {
				e.mu.Lock()
				e.tickOverruns++
				e.mu.Unlock()
			}
			if reason != StopReasonNone {
				e.endOnItsOwn(reason)
				return
			}
		}
	}
}

// endOnItsOwn stops driving the follower after the loop gave up.
func (e *Engine) endOnItsOwn(reason StopReason) {
	e.sender.Wait()
	ctx, cancel := context.WithTimeout(context.Background(), stopTimeout)
	defer cancel()
	e.mu.Lock()
	defer e.mu.Unlock()
	// End the status poller too: a later Start makes a new context, and a later Stop waits
	// for every goroutine of every session.
	if e.cancel != nil {
		e.cancel()
	}
	e.running = false
	e.stopReason = reason
	e.deps.Logger.Warnf("clutch loop ended (%s): %s; send stop, then start to engage again", reason, e.lastError)
	_ = e.unregisterLocked(ctx)
}

// pollStatus copies the pipeline's view into the status while running. A goal the planner
// rejects shows up ONLY here: teleop_move only stores a pose, so the sends keep succeeding.
func (e *Engine) pollStatus(ctx context.Context) {
	defer e.wg.Done()
	ticker := time.NewTicker(statusPollInterval)
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			pollCtx, cancel := context.WithTimeout(ctx, statusPollInterval)
			st, err := e.deps.Pipeline.Status(pollCtx)
			cancel()
			if err != nil {
				continue
			}
			perr, _ := st["error"].(string)
			plan, _ := st["last_plan_ms"].(float64)
			e.mu.Lock()
			prev := e.pipelineError
			e.pipelineError, e.pipelinePlanMs = perr, plan
			e.mu.Unlock()
			if perr != "" {
				e.pipelineFault.Store(&perr)
			}
			if perr != prev {
				if perr != "" {
					e.deps.Logger.Warnf("pipeline error: %s", perr)
				} else {
					e.deps.Logger.Info("pipeline error cleared")
				}
			}
		}
	}
}

// tick is one cycle: read the hand, map, filter, offer to the sender, mirror the gripper.
// It returns a reason when the loop should end.
func (e *Engine) tick(ctx context.Context) StopReason {
	if fault := e.pipelineFault.Load(); fault != nil {
		e.mu.Lock()
		e.noteErrorLocked(fmt.Errorf("teleop pipeline error: %s", *fault))
		e.mu.Unlock()
		return StopReasonPipelineError
	}
	start := time.Now()
	sample, err := e.deps.Source.Read(ctx)
	e.sourceReadNanos.Store(time.Since(start).Nanoseconds())
	if err == nil {
		e.tracked.Store(sample.Tracked)
		if !sample.Tracked {
			e.mu.Lock()
			e.lastError = fmt.Sprintf("%s lost tracking", e.deps.Source)
			e.mu.Unlock()
			return StopReasonUntracked
		}
	}

	if err == nil {
		e.mu.Lock()
		ref, mode, f := e.ref, e.rotationMode, e.filter
		e.mu.Unlock()
		now := e.deps.Now()
		if target, ok := ref.Target(sample.Pose, e.cfg.Scale, mode, e.cfg.ToolRotation); ok {
			e.maybeLogPose(sample.Pose, target)
			if filtered, pass := f.Step(now, target); pass {
				e.goal = filtered
			}
		}
		// The limiter steps every tick toward the latest goal, so a follower that fell behind
		// a fast hand keeps catching up at the capped speed after the hand stops.
		if cmd, send := e.limiter.Step(now, e.goal); send {
			e.sender.TrySend(cmd)
		}
		if gerr := e.mirrorGripper(ctx); gerr != nil {
			e.deps.Logger.Warnf("gripper: %v", gerr)
		}
	}
	if gerr := e.out.TakeErr(); gerr != nil {
		e.deps.Logger.Warnf("follower gripper: %v", gerr)
	}

	e.mu.Lock()
	defer e.mu.Unlock()
	if se := e.sender.TakeErr(); se != nil && err == nil {
		err = se
	}
	if e.sender.Lost() {
		e.noteErrorLocked(fmt.Errorf("teleop pipeline session lost: %w", err))
		return StopReasonSessionLost
	}
	if err != nil {
		e.noteErrorLocked(err)
		e.deps.Logger.Warnf("clutch cycle error (%d/%d): %v", e.consecutiveErrors, e.cfg.MaxConsecutiveErrors, err)
		if e.consecutiveErrors >= e.cfg.MaxConsecutiveErrors {
			return StopReasonMaxErrors
		}
		return StopReasonNone
	}
	e.consecutiveErrors, e.lastError = 0, ""
	e.cycles++
	return StopReasonNone
}

func (e *Engine) noteErrorLocked(err error) {
	e.consecutiveErrors++
	e.errorsTotal++
	e.lastError = err.Error()
	e.lastErrorAt = e.deps.Now()
}

// sourceRotationLocked asks the frame system for the rotation from the source's frame into
// world, once per Start. Without a frame system, or when the frame is unknown to it (a
// tracker or leader with no frame block), the source frame is taken to be world and a
// warning is logged once per frame name. Only the rotation matters: the mapping is a delta.
func (e *Engine) sourceRotationLocked(ctx context.Context, frame string) spatialmath.Orientation {
	if e.deps.Frames == nil || frame == "" {
		return nil
	}
	pif, err := e.deps.Frames.TransformPose(ctx,
		referenceframe.NewPoseInFrame(frame, spatialmath.NewZeroPose()), pipeline.WorldFrame, nil)
	if err != nil {
		if !e.frameWarned[frame] {
			e.frameWarned[frame] = true
			e.deps.Logger.Warnf("frame %q is not in the frame system (%v); treating the source frame as world", frame, err)
		}
		return nil
	}
	return pif.Pose().Orientation()
}

// mirrorGripper reads the leader gripper and offers the percentage to the follower output,
// which writes on change, asynchronously.
func (e *Engine) mirrorGripper(ctx context.Context) error {
	if e.reader == nil {
		return nil
	}
	start := time.Now()
	pct, err := e.reader.Read(ctx)
	e.gripperReadNanos.Store(time.Since(start).Nanoseconds())
	if err != nil {
		return err
	}
	e.out.Set(pct)
	return nil
}

// maybeLogPose writes the run loop's pose line at most once per poseLogInterval.
func (e *Engine) maybeLogPose(hand, target spatialmath.Pose) {
	now := e.deps.Now()
	last := e.poseLogAt.Load()
	if last != 0 && now.Sub(time.Unix(0, last)) < poseLogInterval {
		return
	}
	if !e.poseLogAt.CompareAndSwap(last, now.UnixNano()) {
		return
	}
	e.deps.Logger.Infow("clutch pose",
		"session", e.sessionID,
		"hand", posemath.PoseMap(hand),
		"target", posemath.PoseMap(target),
		"follower_cmd", posemath.PoseMap(e.sender.LastSent()),
		"source_read_ms", posemath.Round1(float64(e.sourceReadNanos.Load())/1e6),
	)
}

func orDefault(o spatialmath.Orientation) spatialmath.Orientation {
	if o == nil {
		return spatialmath.NewZeroOrientation()
	}
	return o
}
