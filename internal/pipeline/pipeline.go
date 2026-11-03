// Package pipeline is the client side of viam-server's builtin motion service teleop
// pipeline: the teleop_start / teleop_move / teleop_stop / teleop_status DoCommands, plus the
// latest-wins Sender every teleop service in this module drives it through.
package pipeline

import (
	"context"
	"fmt"
	"strings"
	"sync/atomic"

	commonpb "go.viam.com/api/common/v1"
	pb "go.viam.com/api/service/motion/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"google.golang.org/protobuf/encoding/protojson"
)

// Motion is the slice of motion.Service the pipeline needs. motion.Service satisfies it, and
// so does a gRPC client to a remote machine's motion service.
type Motion interface {
	GetPose(ctx context.Context, componentName, destinationFrame string,
		supplementalTransforms []*referenceframe.LinkInFrame, extra map[string]interface{},
	) (*referenceframe.PoseInFrame, error)
	DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error)
}

// WorldFrame is the frame every pipeline pose is expressed in.
const WorldFrame = "world"

// The DoCommand keys of rdk's builtin motion service teleop pipeline
// (rdk v1.14.4 services/motion/builtin/builtin.go:63-66). They are re-typed here rather than
// imported: the builtin package reaches motionplan's nlopt solver, which needs cgo, and this
// module ships a windows/amd64 target without it.
const (
	cmdStart  = "teleop_start"
	cmdMove   = "teleop_move"
	cmdStop   = "teleop_stop"
	cmdStatus = "teleop_status"
)

// Pipeline is a client for one component registered with the teleop pipeline. Payloads are
// protojson of the real request messages rather than hand-formatted strings, so a field the
// server adds or renames fails loudly here rather than being silently ignored.
type Pipeline struct {
	motion     Motion
	motionName string // the motion service's resource name, "builtin" for the default
	component  string // the frame-system component the pipeline positions (usually the gripper)
	seq        atomic.Int64
}

// New returns a Pipeline that positions component through m.
func New(m Motion, motionName, component string) *Pipeline {
	return &Pipeline{motion: m, motionName: motionName, component: component}
}

// Component is the frame-system name the pipeline positions.
func (p *Pipeline) Component() string { return p.component }

// CurrentPose reads the component's pose in the world frame.
func (p *Pipeline) CurrentPose(ctx context.Context) (spatialmath.Pose, error) {
	pif, err := p.motion.GetPose(ctx, p.component, WorldFrame, nil, nil)
	if err != nil {
		return nil, fmt.Errorf("get pose of %q: %w", p.component, err)
	}
	return pif.Pose(), nil
}

// Start registers the component with the pipeline, with destination as its initial target.
// Pass the component's current pose so registration itself causes no motion.
func (p *Pipeline) Start(ctx context.Context, destination spatialmath.Pose) error {
	req := &pb.MoveRequest{
		Name:          p.motionName,
		ComponentName: p.component,
		Destination:   referenceframe.PoseInFrameToProtobuf(referenceframe.NewPoseInFrame(WorldFrame, destination)),
	}
	payload, err := protojson.Marshal(req)
	if err != nil {
		return err
	}
	if _, err = p.motion.DoCommand(ctx, map[string]interface{}{cmdStart: string(payload)}); err != nil {
		return fmt.Errorf("%s: %w", cmdStart, err)
	}
	return nil
}

// Move replaces the component's target. The pipeline keeps only the latest pose, so a caller
// should keep at most one Move in flight and send its freshest pose when the previous returns;
// Sender does exactly that.
func (p *Pipeline) Move(ctx context.Context, target spatialmath.Pose) error {
	pif := referenceframe.PoseInFrameToProtobuf(referenceframe.NewPoseInFrame(WorldFrame, target))
	payload, err := protojson.Marshal((*commonpb.PoseInFrame)(pif))
	if err != nil {
		return err
	}
	_, err = p.motion.DoCommand(ctx, map[string]interface{}{
		cmdMove:          string(payload),
		"component_name": p.component,
		"seq":            float64(p.seq.Add(1)),
	})
	if err != nil {
		return fmt.Errorf("%s: %w", cmdMove, err)
	}
	return nil
}

// Stop removes the component from the pipeline. The follower holds where it is.
func (p *Pipeline) Stop(ctx context.Context) error {
	_, err := p.motion.DoCommand(ctx, map[string]interface{}{
		cmdStop:          true,
		"component_name": p.component,
	})
	if err != nil {
		return fmt.Errorf("%s: %w", cmdStop, err)
	}
	return nil
}

// Status returns the pipeline's teleop_status map. rdk v1.14.4 reports running,
// last_inputs_ms, last_plan_ms, last_exec_ms, last_exec_wait_ms, plan_count, exec_count,
// error and components (services/motion/builtin/teleop.go:688-707). It is pipeline-wide: a
// planner error names the component it concerns inside the error string, if at all.
func (p *Pipeline) Status(ctx context.Context) (map[string]interface{}, error) {
	resp, err := p.motion.DoCommand(ctx, map[string]interface{}{cmdStatus: true})
	if err != nil {
		return nil, fmt.Errorf("%s: %w", cmdStatus, err)
	}
	status, _ := resp[cmdStatus].(map[string]interface{})
	return status, nil
}

// IsSessionLost reports whether err means the server-side pipeline no longer knows this
// component, which no retry of Move can fix: the session must be restarted. The two
// substrings are rdk v1.14.4 services/motion/builtin/teleop.go:608 ("teleop pipeline is not
// running") and :642 ("component %q not registered in teleop pipeline").
func IsSessionLost(err error) bool {
	if err == nil {
		return false
	}
	s := err.Error()
	return strings.Contains(s, "not running") || strings.Contains(s, "not registered in teleop pipeline")
}
