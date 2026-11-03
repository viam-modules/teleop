package leaderfollower

import (
	"context"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"teleop/internal/filter"
	"teleop/internal/gripper"
	"teleop/internal/pipeline"
	"teleop/internal/posemath"
)

// Leader is the slice of arm.Arm the engine reads and aligns. arm.Arm satisfies it, in
// process or as a gRPC client. Which methods are used depends on the mode: EndPosition and
// JointPositions always; Kinematics and MoveToJointPositions with AlignOnStart; Properties
// and SetManualMode with HoldManual; DoCommand with HoldCustom.
type Leader interface {
	EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error)
	JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error)
	MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error
	Kinematics(ctx context.Context) (referenceframe.Model, error)
	DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error)
	Properties(ctx context.Context, extra map[string]interface{}) (arm.Properties, error)
	SetManualMode(ctx context.Context, manualMode bool, enabledFor time.Duration, extra map[string]interface{}) error
}

// alignReachToleranceDeg is how far any joint may be from the IK target after the blocking
// align move before Start concludes the arm did not actually move. Generous: it must clear
// the servos' gravity droop (a few degrees) and the profiled move's own stop tolerance, and
// it only needs to catch "did not move at all".
const alignReachToleranceDeg = 10.0

// alignMovedThresholdDeg is the smallest joint change that counts as "the arm moved" when the
// align move fell short: below it the servos are judged not to be driving at all.
const alignMovedThresholdDeg = 2.0

// statusPollInterval is how often the pipeline's own status is read while mirroring.
const statusPollInterval = 200 * time.Millisecond

// maxJointGapDeg returns the largest per-joint |a-b| in degrees and which joint it was.
func maxJointGapDeg(a, b []referenceframe.Input) (float64, int) {
	worst, idx := 0.0, 0
	for i := range a {
		if i >= len(b) {
			break
		}
		if d := math.Abs(a[i]-b[i]) * 180 / math.Pi; d > worst {
			worst, idx = d, i
		}
	}
	return worst, idx
}

// Deps are the resources the engine drives. LeaderGripper and FollowerGripper may both be
// nil; Now may be nil for the wall clock.
type Deps struct {
	Leader          Leader
	LeaderGripper   gripper.Gripper
	Pipeline        *pipeline.Pipeline
	FollowerGripper gripper.Gripper
	Logger          logging.Logger
	Now             func() time.Time
}

// Alignment is what Start established before mirroring began. With AlignOnStart it records
// the move; without it, it records the capture (LeaderTarget and LeaderInputs are then just
// where the leader was).
type Alignment struct {
	Aligned             bool // true if the leader was driven to the follower's pose
	FollowerPose        spatialmath.Pose
	LeaderTarget        spatialmath.Pose // Mapping.Inverse(FollowerPose)
	LeaderInputs        []referenceframe.Input
	LeaderActual        spatialmath.Pose // the leader's EndPosition after Start's move/capture
	Mapped              spatialmath.Pose // Mapping.Forward(LeaderActual), before the residual
	PositionErrorMM     float64          // gap between the mapped leader pose and the follower, after Start
	OrientationErrorDeg float64
	Residual            Residual
}

// StopReason says why the mirror loop is not running.
type StopReason string

const (
	// StopReasonNone: the loop is running, or never started.
	StopReasonNone StopReason = ""
	// StopReasonStopped: Stop was called.
	StopReasonStopped StopReason = "stopped"
	// StopReasonSessionLost: the pipeline no longer knows this component.
	StopReasonSessionLost StopReason = "session_lost"
	// StopReasonMaxErrors: MaxConsecutiveErrors ticks failed in a row.
	StopReasonMaxErrors StopReason = "max_errors"
)

// Status is a snapshot for DoCommand/CLI reporting.
type Status struct {
	Running           bool       `json:"running"`
	SessionID         string     `json:"session_id"`
	StoppedReason     StopReason `json:"stopped_reason"`
	LeaderHeld        bool       `json:"leader_held"`
	LastHoldError     string     `json:"last_hold_error"`
	Cycles            uint64     `json:"cycles"` // ticks completed without error
	Sends             uint64     `json:"sends"`
	Dropped           uint64     `json:"dropped"` // a send was due but one was already in flight
	Deadzoned         uint64     `json:"deadzoned"`
	Rejected          uint64     `json:"rejected"` // outlier rejections
	TickOverruns      uint64     `json:"tick_overruns"`
	ConsecutiveErrors int        `json:"consecutive_errors"`
	ErrorsTotal       uint64     `json:"errors_total"`
	LastError         string     `json:"last_error"`
	LastErrorAt       string     `json:"last_error_at"` // RFC 3339, "" if none
	LastRPCMs         float64    `json:"last_rpc_ms"`
	LeaderReadMs      float64    `json:"leader_read_ms"`
	GripperReadMs     float64    `json:"gripper_read_ms"`
	RateHz            float64    `json:"rate_hz"`
	AlignPosErrMM     float64    `json:"align_position_error_mm"`
	AlignOriErrDeg    float64    `json:"align_orientation_error_deg"`
	GripperPct        float64    `json:"gripper_pct"`
	GripperWrites     uint64     `json:"gripper_writes"`
	// PipelineError and PipelinePlanMs are copied from the motion service's teleop_status:
	// a goal the planner rejects shows up ONLY there, the sends keep succeeding.
	PipelineError  string  `json:"pipeline_error"`
	PipelinePlanMs float64 `json:"pipeline_plan_ms"`
}

// Engine runs one leader→follower Cartesian mirror session.
type Engine struct {
	deps Deps
	cfg  Config

	sender *pipeline.Sender
	out    *gripper.Output
	reader *gripper.Reader // nil without a leader gripper

	mu      sync.Mutex
	running bool // the mirror loop is alive
	// sessionOpen is true from a successful alignment until Stop: the leader is held and the
	// follower is registered with the pipeline. It outlives running, because a loop that
	// stops itself on errors deliberately leaves the leader held (a hand may be on it).
	sessionOpen bool
	cancel      context.CancelFunc
	wg          sync.WaitGroup
	align       *Alignment
	startOri    spatialmath.Orientation
	filter      *filter.Filter
	sessionID   string
	sessions    uint64

	cycles, tickOverruns, errorsTotal uint64
	consecutiveErrors                 int
	lastError, lastHoldError          string
	lastErrorAt                       time.Time
	stoppedReason                     StopReason
	pipelineError                     string
	pipelinePlanMs                    float64

	leaderHeld       atomic.Bool
	leaderReadNanos  atomic.Int64
	gripperReadNanos atomic.Int64
	poseLogAt        atomic.Int64 // poselog.go
}

// New returns an Engine. It does nothing until Start.
func New(deps Deps, cfg Config) (*Engine, error) {
	if deps.Logger == nil {
		deps.Logger = logging.NewLogger("leader-follower")
	}
	if deps.Now == nil {
		deps.Now = time.Now
	}
	if deps.Leader == nil || deps.Pipeline == nil {
		return nil, fmt.Errorf("leader-follower engine needs a leader and a pipeline")
	}
	if cfg.RateHz <= 0 {
		cfg.RateHz = DefaultRateHz
	}
	if cfg.MaxConsecutiveErrors <= 0 {
		cfg.MaxConsecutiveErrors = MaxConsecutiveErrors
	}
	if cfg.MoveTimeout <= 0 {
		cfg.MoveTimeout = MoveTimeout
	}
	if cfg.Hold.Mode == "" {
		cfg.Hold.Mode = HoldNone
	}
	if cfg.Gripper.Mode == "" {
		cfg.Gripper.Mode = gripper.ModeOpenClose
	}
	e := &Engine{deps: deps, cfg: cfg}
	e.sender = pipeline.NewSender(deps.Pipeline, cfg.MoveTimeout)
	e.out = gripper.NewOutput(deps.FollowerGripper, cfg.Gripper)
	if deps.LeaderGripper != nil && e.out.Enabled() {
		r, err := gripper.NewReader(deps.LeaderGripper, cfg.GripperRead.Command, cfg.GripperRead.Key)
		if err != nil {
			return nil, err
		}
		e.reader = r
	}
	return e, nil
}

// Start aligns (or captures) the leader, hands it to the operator, registers the follower
// with the pipeline at its current pose, and begins mirroring. Nothing on the follower side
// moves during Start; with AlignOnStart the leader does.
func (e *Engine) Start(ctx context.Context) error {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.running {
		return nil
	}
	if e.sessionOpen {
		// The loop died on errors and nobody called Stop: clean up before re-aligning.
		e.deps.Logger.Warn("Previous session ended on errors; releasing it before restarting")
		e.closeSessionLocked(ctx)
	}
	e.sessions++
	e.sessionID = fmt.Sprintf("%d-%d", e.deps.Now().Unix(), e.sessions)

	var al *Alignment
	var err error
	if e.cfg.AlignOnStart {
		al, err = e.alignLeader(ctx)
	} else {
		al, err = e.captureLeader(ctx)
	}
	if err != nil {
		return err
	}
	if e.reader != nil {
		// One read now, so a wrong command or key fails Start with a message that names it,
		// instead of failing every tick until the loop gives up.
		if _, err := e.reader.Read(ctx); err != nil {
			e.releaseLeader(ctx)
			return err
		}
	}
	if err := e.deps.Pipeline.Start(ctx, al.FollowerPose); err != nil {
		e.releaseLeader(ctx)
		return err
	}

	e.align = al
	e.sessionOpen = true
	e.startOri = al.FollowerPose.Orientation()
	e.filter = filter.New(e.cfg.Filter)
	e.sender.Reset()
	e.out.Reset()
	e.cycles, e.tickOverruns, e.errorsTotal = 0, 0, 0
	e.consecutiveErrors, e.lastError, e.lastErrorAt = 0, "", time.Time{}
	e.stoppedReason = StopReasonNone
	e.pipelineError, e.pipelinePlanMs = "", 0

	loopCtx, cancel := context.WithCancel(context.Background())
	e.cancel = cancel
	e.running = true
	e.wg.Add(2)
	go e.runLoop(loopCtx)
	go e.pollStatus(loopCtx)
	if al.Aligned {
		e.deps.Logger.Infof("leader-follower started at %.0f Hz; leader aligned to %.1f mm / %.1f deg of the follower",
			e.cfg.RateHz, al.PositionErrorMM, al.OrientationErrorDeg)
	} else {
		e.deps.Logger.Infof("leader-follower started at %.0f Hz; mirroring deltas from the leader's current pose "+
			"(%.0f mm / %.0f deg offset absorbed)", e.cfg.RateHz, al.PositionErrorMM, al.OrientationErrorDeg)
	}
	return nil
}

// Align is the alignment performed by the last Start, or nil.
func (e *Engine) Align() *Alignment {
	e.mu.Lock()
	defer e.mu.Unlock()
	return e.align
}

// captureLeader is the default start: both arms stay where they are. The gap between the
// mapped leader pose and the follower becomes the session's Residual, so the first target
// equals the follower's current pose and the leader's motion from here is mirrored as deltas.
// Caller holds e.mu.
func (e *Engine) captureLeader(ctx context.Context) (*Alignment, error) {
	followerPose, err := e.deps.Pipeline.CurrentPose(ctx)
	if err != nil {
		return nil, err
	}
	joints, err := e.deps.Leader.JointPositions(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("read leader joints: %w", err)
	}
	e.beforeStart(ctx)
	if err := e.applyHold(ctx); err != nil {
		e.releaseLeader(ctx)
		return nil, err
	}
	actual, err := e.deps.Leader.EndPosition(ctx, nil)
	if err != nil {
		e.releaseLeader(ctx)
		return nil, fmt.Errorf("read leader pose: %w", err)
	}
	mapped := e.cfg.Mapping.Forward(actual)
	al := &Alignment{
		FollowerPose:        followerPose,
		LeaderTarget:        e.cfg.Mapping.Inverse(followerPose),
		LeaderInputs:        joints,
		LeaderActual:        actual,
		Mapped:              mapped,
		PositionErrorMM:     followerPose.Point().Sub(mapped.Point()).Norm(),
		OrientationErrorDeg: posemath.AngleBetweenDeg(mapped, followerPose),
		Residual:            ResidualBetween(mapped, followerPose),
	}
	e.logAlign(al)
	return al, nil
}

// alignLeader drives the leader as close as it can get to the follower's current pose and
// applies the configured hold. Caller holds e.mu.
func (e *Engine) alignLeader(ctx context.Context) (*Alignment, error) {
	model, err := e.deps.Leader.Kinematics(ctx)
	if err != nil {
		return nil, fmt.Errorf("leader kinematics: %w", err)
	}
	followerPose, err := e.deps.Pipeline.CurrentPose(ctx)
	if err != nil {
		return nil, err
	}
	want := e.cfg.Mapping.Inverse(followerPose)
	seed, err := e.deps.Leader.JointPositions(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("read leader joints: %w", err)
	}
	res, err := SolveBestEffort(ctx, model, seed, want, e.cfg.IK)
	if err != nil {
		return nil, fmt.Errorf("leader IK: %w", err)
	}
	if e.cfg.AlignMaxPositionErrorMM > 0 && res.PositionErrorMM > e.cfg.AlignMaxPositionErrorMM {
		return nil, fmt.Errorf("leader cannot get within %.0f mm of the follower's pose (best %.1f mm); "+
			"move the follower nearer the mapped workspace or adjust scale/origin",
			e.cfg.AlignMaxPositionErrorMM, res.PositionErrorMM)
	}

	// A previous session may have left the leader compliant; a compliant arm cannot be
	// driven to a pose.
	e.beforeStart(ctx)
	if err := e.deps.Leader.MoveToJointPositions(ctx, res.Inputs, nil); err != nil {
		return nil, fmt.Errorf("move leader to the follower's pose: %w", err)
	}
	// A blocking move that returns nil is not proof the arm went anywhere: a servo with torque
	// off, or tripped into overload protection, accepts the goal, reports "not moving", and
	// stays put. Seen on hardware: the engine announced a 0.4 mm alignment while the arm had
	// not moved a degree. So verify from the position read, not the move's return value.
	after, err := e.deps.Leader.JointPositions(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("read leader joints after align move: %w", err)
	}
	if worst, joint := maxJointGapDeg(after, res.Inputs); worst > alignReachToleranceDeg {
		// Two different situations end here. If NO joint moved, the servos are not driving
		// (torque off, overload trip): refuse. If the arm moved but a joint stopped short, it
		// hit a limit the model does not know about (an arm clamps to its CALIBRATED range,
		// the IK only sees Kinematics()' static limits) or it drooped under load. That is what
		// the residual compensation exists for, so accept it and let the position-residual
		// check decide whether the leftover gap is usable.
		if moved, _ := maxJointGapDeg(after, seed); moved < alignMovedThresholdDeg {
			// Either the servos are not driving or every joint the IK wanted to move was
			// already parked at its calibrated limit. We cannot tell the two apart from here,
			// so this is strict only when a residual limit is configured.
			msg := fmt.Sprintf("leader align: no joint moved although joint %d is %.1f deg from its target. "+
				"Either torque is off / a servo tripped overload, or the arm is already at its calibrated "+
				"limits for this pose", joint+1, worst)
			if e.cfg.AlignMaxPositionErrorMM > 0 {
				return nil, fmt.Errorf("%s", msg)
			}
			e.deps.Logger.Warn(msg)
		} else {
			e.deps.Logger.Warnf("leader align: joint %d stopped %.1f deg short of its target (calibrated limit or "+
				"droop); compensating with the residual", joint+1, worst)
		}
		res.Inputs = after
	}
	if err := e.applyHold(ctx); err != nil {
		// The move left the leader stiff at the aligned pose and the hold may be half
		// applied. Release so it is not stranded in whatever state the failure left it, and
		// let the caller log the reason.
		e.releaseLeader(ctx)
		return nil, err
	}

	al := &Alignment{
		Aligned:      true,
		FollowerPose: followerPose,
		LeaderTarget: want,
		LeaderInputs: res.Inputs,
	}
	actual, err := e.deps.Leader.EndPosition(ctx, nil)
	if err != nil {
		e.releaseLeader(ctx)
		return nil, fmt.Errorf("read leader pose after align: %w", err)
	}
	mapped := e.cfg.Mapping.Forward(actual)
	al.LeaderActual, al.Mapped = actual, mapped
	al.PositionErrorMM = followerPose.Point().Sub(mapped.Point()).Norm()
	al.OrientationErrorDeg = posemath.AngleBetweenDeg(mapped, followerPose)
	if e.cfg.AlignMaxPositionErrorMM > 0 && al.PositionErrorMM > e.cfg.AlignMaxPositionErrorMM {
		e.releaseLeader(ctx)
		return nil, fmt.Errorf("after the align move the leader maps %.0f mm from the follower's pose "+
			"(limit %.0f): a joint stopped short of what the IK asked for. Move the follower nearer the "+
			"leader's reachable workspace or adjust scale/origin", al.PositionErrorMM, e.cfg.AlignMaxPositionErrorMM)
	}
	if al.PositionErrorMM > 25 || al.OrientationErrorDeg > 10 {
		e.deps.Logger.Warnf("leader align is best effort: it maps %.0f mm / %.0f deg from the follower's pose; "+
			"the gap is absorbed as a constant offset", al.PositionErrorMM, al.OrientationErrorDeg)
	}
	al.Residual = ResidualBetween(mapped, followerPose)
	e.logAlign(al)
	return al, nil
}

// Stop ends the session: the loop stops, the follower is removed from the pipeline and holds
// where it is, and the leader is released from its hold.
func (e *Engine) Stop(ctx context.Context) error {
	e.mu.Lock()
	defer e.mu.Unlock()
	if !e.running && !e.sessionOpen {
		return nil
	}
	err := e.closeSessionLocked(ctx)
	e.stoppedReason = StopReasonStopped
	e.deps.Logger.Info("leader-follower stopped")
	return err
}

// closeSessionLocked stops the loop if it is alive, removes the follower from the pipeline
// and releases the leader. Caller holds e.mu; the loop never takes e.mu while it is being
// waited on except inside tick, which returns promptly.
func (e *Engine) closeSessionLocked(ctx context.Context) error {
	if e.cancel != nil {
		e.cancel()
	}
	e.mu.Unlock()
	e.wg.Wait()
	e.sender.Wait()
	e.out.Wait()
	e.mu.Lock()

	var firstErr error
	if err := e.deps.Pipeline.Stop(ctx); err != nil && !pipeline.IsSessionLost(err) {
		firstErr = err
		e.deps.Logger.Warnf("%v", err)
	}
	e.releaseLeader(ctx)
	e.running, e.sessionOpen = false, false
	return firstErr
}

// Status returns a snapshot of the session.
func (e *Engine) Status() Status {
	snd := e.sender.Snapshot()
	e.mu.Lock()
	defer e.mu.Unlock()
	st := Status{
		Running:           e.running,
		SessionID:         e.sessionID,
		StoppedReason:     e.stoppedReason,
		LeaderHeld:        e.leaderHeld.Load(),
		LastHoldError:     e.lastHoldError,
		Cycles:            e.cycles,
		Sends:             snd.Sends,
		Dropped:           snd.Dropped,
		TickOverruns:      e.tickOverruns,
		ConsecutiveErrors: e.consecutiveErrors,
		ErrorsTotal:       e.errorsTotal,
		LastError:         e.lastError,
		LastRPCMs:         snd.LastRPCMs,
		LeaderReadMs:      float64(e.leaderReadNanos.Load()) / 1e6,
		GripperReadMs:     float64(e.gripperReadNanos.Load()) / 1e6,
		RateHz:            e.cfg.RateHz,
		GripperPct:        e.out.Pct(),
		GripperWrites:     e.out.Writes(),
		PipelineError:     e.pipelineError,
		PipelinePlanMs:    e.pipelinePlanMs,
	}
	if !e.lastErrorAt.IsZero() {
		st.LastErrorAt = e.lastErrorAt.Format(time.RFC3339)
	}
	if e.filter != nil {
		c := e.filter.Counters()
		st.Deadzoned, st.Rejected = c.Deadzoned, c.Rejected
	}
	if e.align != nil {
		st.AlignPosErrMM, st.AlignOriErrDeg = e.align.PositionErrorMM, e.align.OrientationErrorDeg
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
			stop := e.tick(ctx)
			if time.Since(start) > period {
				e.mu.Lock()
				e.tickOverruns++
				e.mu.Unlock()
			}
			if stop {
				return
			}
		}
	}
}

// pollStatus copies the pipeline's own view into the status map while mirroring, and logs
// its error transitions. A goal the planner rejects, or a collision with a world obstacle,
// shows up ONLY here: the sends keep succeeding, because teleop_move only stores a pose.
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

// tick runs one cycle and reports whether the loop should stop. Failed sends and gripper
// writes are asynchronous, so they are folded into the error of the tick that first sees
// them. A gripper write failure is logged but does not count toward stopping the loop: the
// arm keeps mirroring without its gripper.
func (e *Engine) tick(ctx context.Context) bool {
	var err error
	if !e.sender.Lost() {
		err = e.syncOnce(ctx)
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
		err = fmt.Errorf("teleop pipeline session lost: %w", err)
	}
	if err != nil {
		e.consecutiveErrors++
		e.errorsTotal++
		e.lastError = err.Error()
		e.lastErrorAt = e.deps.Now()
		e.deps.Logger.Warnf("leader-follower cycle error (%d/%d): %v", e.consecutiveErrors, e.cfg.MaxConsecutiveErrors, err)
		if e.sender.Lost() || e.consecutiveErrors >= e.cfg.MaxConsecutiveErrors {
			e.stoppedReason = StopReasonMaxErrors
			if e.sender.Lost() {
				e.stoppedReason = StopReasonSessionLost
			}
			e.deps.Logger.Errorf("leader-follower loop stopping (%s): %v", e.stoppedReason, err)
			e.running = false
			// The leader is still held; a Stop from the caller releases it. We do not
			// release here because the hand may be on the arm.
			return true
		}
		return false
	}
	e.consecutiveErrors, e.lastError = 0, ""
	e.cycles++
	return false
}

// syncOnce reads the leader, maps and filters the target, and offers it to the sender (at
// most one move in flight, latest wins), then mirrors the gripper.
func (e *Engine) syncOnce(ctx context.Context) error {
	start := time.Now()
	pose, err := e.deps.Leader.EndPosition(ctx, nil)
	e.leaderReadNanos.Store(time.Since(start).Nanoseconds())
	if err != nil {
		return fmt.Errorf("read leader pose: %w", err)
	}
	target := e.MapTarget(pose)
	e.maybeLogPose(pose, target)

	e.mu.Lock()
	f := e.filter
	e.mu.Unlock()
	if send, ok := f.Step(e.deps.Now(), target); ok {
		e.sender.TrySend(send)
	}
	return e.mirrorGripper(ctx)
}

// MapTarget turns a leader end-effector pose into the follower target for this session:
// mapping, alignment residual, and the rotation lock when rotation is disabled.
func (e *Engine) MapTarget(leader spatialmath.Pose) spatialmath.Pose {
	e.mu.Lock()
	al, startOri := e.align, e.startOri
	e.mu.Unlock()
	target := e.cfg.Mapping.Forward(leader)
	if al != nil {
		target = al.Residual.Apply(target)
	}
	if !e.cfg.RotationEnabled && startOri != nil {
		target = spatialmath.NewPose(target.Point(), startOri)
	}
	return target
}

// mirrorGripper reads the leader gripper and offers the percentage to the follower output,
// which writes on change, asynchronously. The read is synchronous (one bus transaction).
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
