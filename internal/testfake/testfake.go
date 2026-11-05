// Package testfake holds the fakes the module's tests drive engines with: a leader arm with
// real forward kinematics on a stock rdk model, a motion service that records the teleop
// DoCommands, and a gripper that answers a configurable read and records writes. They stand
// in for rdk's testutils/inject, which pulls a cgo-tagged motion fake and a media stack this
// module has no use for.
package testfake

import (
	"context"
	"errors"
	"fmt"
	"math"
	"sync"
	"testing"
	"time"

	commonpb "go.viam.com/api/common/v1"
	pb "go.viam.com/api/service/motion/v1"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/arm/kinematics"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"google.golang.org/protobuf/encoding/protojson"
)

// Model returns the 5-DoF leader model the tests use: rdk's embedded Yahboom DOFBOT, chosen
// because it is five revolute joints with a wrist roll and about 250 mm of reach, which is
// the shape of arm the leader-follower engine was written for. Nothing in this module ships
// or depends on any particular leader's kinematics.
func Model(t *testing.T) referenceframe.Model {
	t.Helper()
	m, err := kinematics.ModelFromName(kinematics.Dofbot, "leader")
	if err != nil {
		t.Fatalf("test model: %v", err)
	}
	return m
}

// Arm is a leader whose joints are what they were last commanded to; EndPosition is real
// forward kinematics on its model, so alignment is exercised end to end.
type Arm struct {
	arm.Arm // nil; only the methods below are real

	mu     sync.Mutex
	model  referenceframe.Model
	joints []referenceframe.Input

	// Commands records every DoCommand; Moves records the extras of every
	// MoveToJointPositions; ManualCalls records every SetManualMode (true = enter).
	Commands    []map[string]interface{}
	Moves       []map[string]interface{}
	ManualCalls []bool

	// FailRead fails EndPosition and JointPositions.
	FailRead error
	// ClampMax, if set, caps each commanded joint like a real arm clamping to its calibrated
	// range: the move "succeeds" but stops short.
	ClampMax []referenceframe.Input
	// Limp makes MoveToJointPositions accept the goal and return nil without changing the
	// joints, which is what a torque-off or overload-tripped servo bus does.
	Limp bool
	// FailCommand makes DoCommand fail for that one "command" value with FailCommandErr.
	FailCommand    string
	FailCommandErr error
	// SupportsManual is what Properties reports; ManualErr fails SetManualMode.
	SupportsManual bool
	ManualErr      error
	manual         bool
}

// NewArm returns an Arm at joints on model.
func NewArm(model referenceframe.Model, joints []referenceframe.Input) *Arm {
	return &Arm{model: model, joints: append([]referenceframe.Input(nil), joints...), SupportsManual: true}
}

// Model is the arm's kinematic model.
func (a *Arm) Model() referenceframe.Model { return a.model }

// Joints returns a copy of the current joints.
func (a *Arm) Joints() []referenceframe.Input {
	a.mu.Lock()
	defer a.mu.Unlock()
	return append([]referenceframe.Input(nil), a.joints...)
}

// SetJoints moves the arm without a command, as a hand would.
func (a *Arm) SetJoints(j []referenceframe.Input) {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.joints = append([]referenceframe.Input(nil), j...)
}

// SetFailRead makes EndPosition and JointPositions fail from now on; safe while a loop reads.
func (a *Arm) SetFailRead(err error) {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.FailRead = err
}

// InManualMode reports the last SetManualMode state.
func (a *Arm) InManualMode() bool {
	a.mu.Lock()
	defer a.mu.Unlock()
	return a.manual
}

// CommandNames lists the "command" value of every recorded DoCommand.
func (a *Arm) CommandNames() []string {
	a.mu.Lock()
	defer a.mu.Unlock()
	var out []string
	for _, c := range a.Commands {
		s, _ := c["command"].(string)
		out = append(out, s)
	}
	return out
}

// Transform is forward kinematics with inputs clamped into the model's limits first, as a
// real arm's EndPosition reports a joint parked slightly past its limit.
func Transform(model referenceframe.Model, inputs []referenceframe.Input) (spatialmath.Pose, error) {
	limits := model.DoF()
	safe := make([]referenceframe.Input, len(inputs))
	for i, v := range inputs {
		if i < len(limits) {
			v = math.Max(limits[i].Min, math.Min(limits[i].Max, v))
		}
		safe[i] = v
	}
	return model.Transform(safe)
}

func (a *Arm) Name() resource.Name { return arm.Named("leader-arm") }

func (a *Arm) EndPosition(context.Context, map[string]interface{}) (spatialmath.Pose, error) {
	a.mu.Lock()
	defer a.mu.Unlock()
	if a.FailRead != nil {
		return nil, a.FailRead
	}
	return Transform(a.model, a.joints)
}

func (a *Arm) JointPositions(context.Context, map[string]interface{}) ([]referenceframe.Input, error) {
	a.mu.Lock()
	defer a.mu.Unlock()
	if a.FailRead != nil {
		return nil, a.FailRead
	}
	return append([]referenceframe.Input(nil), a.joints...), nil
}

func (a *Arm) MoveToJointPositions(_ context.Context, p []referenceframe.Input, extra map[string]interface{}) error {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.Moves = append(a.Moves, extra)
	if a.Limp {
		return nil
	}
	q := append([]referenceframe.Input(nil), p...)
	for i := range q {
		if a.ClampMax != nil && i < len(a.ClampMax) && q[i] > a.ClampMax[i] {
			q[i] = a.ClampMax[i]
		}
	}
	a.joints = q
	return nil
}

func (a *Arm) Kinematics(context.Context) (referenceframe.Model, error) { return a.model, nil }

func (a *Arm) DoCommand(_ context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.Commands = append(a.Commands, cmd)
	if a.FailCommand != "" && cmd["command"] == a.FailCommand {
		return nil, a.FailCommandErr
	}
	return map[string]interface{}{}, nil
}

func (a *Arm) Properties(context.Context, map[string]interface{}) (arm.Properties, error) {
	return arm.Properties{SupportManualMode: a.SupportsManual}, nil
}

func (a *Arm) SetManualMode(_ context.Context, on bool, _ time.Duration, _ map[string]interface{}) error {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.ManualCalls = append(a.ManualCalls, on)
	if a.ManualErr != nil {
		return a.ManualErr
	}
	a.manual = on
	return nil
}

func (a *Arm) ManualMode(context.Context, map[string]interface{}) (bool, error) {
	return a.InManualMode(), nil
}

// Motion records the teleop DoCommands and serves a fixed follower pose. It satisfies
// motion.Service for a resource.Dependencies map; only the methods below are real.
type Motion struct {
	motion.Service

	mu     sync.Mutex
	pose   spatialmath.Pose
	Starts []*pb.MoveRequest
	Moves  []*commonpb.PoseInFrame
	Stops  int
	// MoveErr fails every teleop_move; MoveWait, if set, blocks teleop_move until closed.
	MoveErr  error
	MoveWait chan struct{}
	// StartErr fails every teleop_start.
	StartErr error
	// StatusErr is what teleop_status reports as the pipeline's error.
	StatusErr string
}

// NewMotion returns a Motion whose component sits at pose.
func NewMotion(pose spatialmath.Pose) *Motion { return &Motion{pose: pose} }

func (m *Motion) Name() resource.Name { return motion.Named("builtin") }

func (m *Motion) GetPose(_ context.Context, _, dst string, _ []*referenceframe.LinkInFrame, _ map[string]interface{}) (*referenceframe.PoseInFrame, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	return referenceframe.NewPoseInFrame(dst, m.pose), nil
}

// SetPose moves the follower, as the pipeline would after a move.
func (m *Motion) SetPose(p spatialmath.Pose) {
	m.mu.Lock()
	m.pose = p
	m.mu.Unlock()
}

// SetStatusError sets the pipeline error teleop_status reports.
func (m *Motion) SetStatusError(e string) {
	m.mu.Lock()
	m.StatusErr = e
	m.mu.Unlock()
}

// SetStartError makes every subsequent teleop_start fail with err.
func (m *Motion) SetStartError(err error) {
	m.mu.Lock()
	m.StartErr = err
	m.mu.Unlock()
}

// SetMoveError makes every subsequent teleop_move fail with err.
func (m *Motion) SetMoveError(err error) {
	m.mu.Lock()
	m.MoveErr = err
	m.mu.Unlock()
}

func (m *Motion) DoCommand(_ context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if s, ok := cmd["teleop_start"].(string); ok {
		m.mu.Lock()
		startErr := m.StartErr
		m.mu.Unlock()
		if startErr != nil {
			return nil, startErr
		}
		var req pb.MoveRequest
		if err := protojson.Unmarshal([]byte(s), &req); err != nil {
			return nil, err
		}
		m.mu.Lock()
		m.Starts = append(m.Starts, &req)
		m.mu.Unlock()
		return map[string]interface{}{"teleop_start": true}, nil
	}
	if s, ok := cmd["teleop_move"].(string); ok {
		m.mu.Lock()
		wait, moveErr := m.MoveWait, m.MoveErr
		m.mu.Unlock()
		if wait != nil {
			<-wait
		}
		if moveErr != nil {
			return nil, moveErr
		}
		var pif commonpb.PoseInFrame
		if err := protojson.Unmarshal([]byte(s), &pif); err != nil {
			return nil, err
		}
		if _, ok := cmd["component_name"].(string); !ok {
			return nil, errors.New("component_name missing")
		}
		m.mu.Lock()
		m.Moves = append(m.Moves, &pif)
		m.mu.Unlock()
		return map[string]interface{}{"teleop_move": true}, nil
	}
	if _, ok := cmd["teleop_stop"]; ok {
		m.mu.Lock()
		m.Stops++
		m.mu.Unlock()
		return map[string]interface{}{"teleop_stop": true}, nil
	}
	if _, ok := cmd["teleop_status"]; ok {
		m.mu.Lock()
		defer m.mu.Unlock()
		st := map[string]interface{}{"running": true, "last_plan_ms": 12.0}
		if m.StatusErr != "" {
			st["error"] = m.StatusErr
		}
		return map[string]interface{}{"teleop_status": st}, nil
	}
	return nil, errors.New("unknown command")
}

// LastMove is the pose of the latest teleop_move, or nil.
func (m *Motion) LastMove() spatialmath.Pose {
	m.mu.Lock()
	defer m.mu.Unlock()
	if len(m.Moves) == 0 {
		return nil
	}
	return referenceframe.ProtobufToPoseInFrame(m.Moves[len(m.Moves)-1]).Pose()
}

// MovePoses returns every accepted teleop_move pose, in order.
func (m *Motion) MovePoses() []spatialmath.Pose {
	m.mu.Lock()
	defer m.mu.Unlock()
	out := make([]spatialmath.Pose, len(m.Moves))
	for i, pif := range m.Moves {
		out[i] = referenceframe.ProtobufToPoseInFrame(pif).Pose()
	}
	return out
}

// MoveCount is how many teleop_moves were accepted.
func (m *Motion) MoveCount() int {
	m.mu.Lock()
	defer m.mu.Unlock()
	return len(m.Moves)
}

// StartCount is how many teleop_starts were received.
func (m *Motion) StartCount() int {
	m.mu.Lock()
	defer m.mu.Unlock()
	return len(m.Starts)
}

// StopCount is how many teleop_stops were received.
func (m *Motion) StopCount() int {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.Stops
}

// Gripper answers one configured read command and records everything else as a write. It
// satisfies gripper.Gripper for a resource.Dependencies map; only the methods below are real.
type Gripper struct {
	gripper.Gripper

	mu sync.Mutex
	// ReadName is the "command" value that is the read; Pct is what it reports under ReadKey.
	ReadName string
	ReadKey  string
	Pct      float64
	// Writes records every other DoCommand; Opens records Open (true) and Grab (false).
	Writes []map[string]interface{}
	Opens  []bool
	// Block, if set, blocks every write until closed (an xArm jaw in motion).
	Block chan struct{}
	// FailWrite fails every write.
	FailWrite error
}

// NewGripper returns a Gripper answering read command name with pct under key.
func NewGripper(readName, readKey string, pct float64) *Gripper {
	return &Gripper{ReadName: readName, ReadKey: readKey, Pct: pct}
}

func (g *Gripper) Name() resource.Name { return gripper.Named("gripper") }

// SetPct changes what the read reports.
func (g *Gripper) SetPct(p float64) {
	g.mu.Lock()
	g.Pct = p
	g.mu.Unlock()
}

func (g *Gripper) Open(context.Context, map[string]interface{}) error {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.Opens = append(g.Opens, true)
	return nil
}

func (g *Gripper) Grab(context.Context, map[string]interface{}) (bool, error) {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.Opens = append(g.Opens, false)
	return true, nil
}

func (g *Gripper) DoCommand(_ context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	g.mu.Lock()
	if g.ReadName != "" && cmd["command"] == g.ReadName {
		resp := map[string]interface{}{g.ReadKey: g.Pct}
		g.mu.Unlock()
		return resp, nil
	}
	g.Writes = append(g.Writes, cmd)
	block, failWrite := g.Block, g.FailWrite
	g.mu.Unlock()
	if block != nil {
		<-block
	}
	if failWrite != nil {
		return nil, failWrite
	}
	return map[string]interface{}{}, nil
}

// WriteCount is how many writes were received.
func (g *Gripper) WriteCount() int {
	g.mu.Lock()
	defer g.mu.Unlock()
	return len(g.Writes)
}

// LastWrite is the latest write, or nil.
func (g *Gripper) LastWrite() map[string]interface{} {
	g.mu.Lock()
	defer g.mu.Unlock()
	if len(g.Writes) == 0 {
		return nil
	}
	return g.Writes[len(g.Writes)-1]
}

// OpenCount is how many Open/Grab calls were received.
func (g *Gripper) OpenCount() int {
	g.mu.Lock()
	defer g.mu.Unlock()
	return len(g.Opens)
}

// PoseTracker reports configured bodies; a body not set is "not tracked".
type PoseTracker struct {
	posetracker.PoseTracker

	mu    sync.Mutex
	frame string
	poses map[string]spatialmath.Pose
	err   error
	reads int
}

// NewPoseTracker returns a tracker whose poses are reported in frame.
func NewPoseTracker(frame string) *PoseTracker {
	return &PoseTracker{frame: frame, poses: map[string]spatialmath.Pose{}}
}

func (p *PoseTracker) Name() resource.Name { return posetracker.Named("tracker") }

// SetPose sets (and tracks) a body.
func (p *PoseTracker) SetPose(body string, pose spatialmath.Pose) {
	p.mu.Lock()
	p.poses[body] = pose
	p.mu.Unlock()
}

// Lose drops a body from the map, as the vive tracker does when a controller's data freezes.
func (p *PoseTracker) Lose(body string) {
	p.mu.Lock()
	delete(p.poses, body)
	p.mu.Unlock()
}

// SetErr makes every read fail with err (nil restores).
func (p *PoseTracker) SetErr(err error) {
	p.mu.Lock()
	p.err = err
	p.mu.Unlock()
}

func (p *PoseTracker) Poses(_ context.Context, bodies []string, _ map[string]interface{}) (referenceframe.FrameSystemPoses, error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.reads++
	if p.err != nil {
		return nil, p.err
	}
	out := referenceframe.FrameSystemPoses{}
	for _, b := range bodies {
		if pose, ok := p.poses[b]; ok {
			out[b] = referenceframe.NewPoseInFrame(p.frame, pose)
		}
	}
	return out, nil
}

// ReadCount is how many times Poses was called.
func (p *PoseTracker) ReadCount() int {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.reads
}

// Frames answers TransformPose for a zero pose in a known frame with that frame's rotation
// into world; an unknown frame is an error, as the frame system reports it.
type Frames struct {
	mu        sync.Mutex
	Rotations map[string]spatialmath.Orientation
	Calls     int
}

func NewFrames(rotations map[string]spatialmath.Orientation) *Frames {
	return &Frames{Rotations: rotations}
}

func (f *Frames) TransformPose(_ context.Context, pif *referenceframe.PoseInFrame, dst string,
	_ []*referenceframe.LinkInFrame,
) (*referenceframe.PoseInFrame, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.Calls++
	rot, ok := f.Rotations[pif.Parent()]
	if !ok {
		return nil, fmt.Errorf("frame %q not found in frame system", pif.Parent())
	}
	return referenceframe.NewPoseInFrame(dst, spatialmath.Compose(spatialmath.NewPoseFromOrientation(rot), pif.Pose())), nil
}

// CallCount is how many transforms were asked for.
func (f *Frames) CallCount() int {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.Calls
}

// Commander records DoCommands (a capture-control sensor).
type Commander struct {
	sensor.Sensor

	mu       sync.Mutex
	Commands []map[string]interface{}
	Err      error
}

func NewCommander() *Commander { return &Commander{} }

func (c *Commander) Name() resource.Name { return sensor.Named("capture") }

func (c *Commander) DoCommand(_ context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.Err != nil {
		return nil, c.Err
	}
	c.Commands = append(c.Commands, cmd)
	return map[string]interface{}{}, nil
}

// Keys returns the first key of every recorded command, in order.
func (c *Commander) Keys() []string {
	c.mu.Lock()
	defer c.mu.Unlock()
	out := make([]string, 0, len(c.Commands))
	for _, cmd := range c.Commands {
		for k := range cmd {
			out = append(out, k)
		}
	}
	return out
}
