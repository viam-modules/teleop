package pipeline

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	pb "go.viam.com/api/service/motion/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
	"google.golang.org/protobuf/encoding/protojson"
)

// recorder is a Motion that records every DoCommand map.
type recorder struct {
	mu    sync.Mutex
	cmds  []map[string]interface{}
	err   error
	block chan struct{}
}

func (r *recorder) GetPose(_ context.Context, _, dst string, _ []*referenceframe.LinkInFrame, _ map[string]interface{}) (*referenceframe.PoseInFrame, error) {
	return referenceframe.NewPoseInFrame(dst, spatialmath.NewPoseFromPoint(r3.Vector{X: 7})), nil
}

func (r *recorder) DoCommand(_ context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	r.mu.Lock()
	r.cmds = append(r.cmds, cmd)
	block, err := r.block, r.err
	r.mu.Unlock()
	if block != nil {
		<-block
	}
	if err != nil {
		return nil, err
	}
	return map[string]interface{}{"teleop_status": map[string]interface{}{"running": true, "error": "boom"}}, nil
}

func (r *recorder) count() int {
	r.mu.Lock()
	defer r.mu.Unlock()
	return len(r.cmds)
}

func (r *recorder) last() map[string]interface{} {
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.cmds[len(r.cmds)-1]
}

func TestPayloadsAreProtojsonOfTheRealMessages(t *testing.T) {
	ctx := context.Background()
	rec := &recorder{}
	p := New(rec, "builtin", "xarm-gripper")

	pose, err := p.CurrentPose(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pose.Point().X, test.ShouldEqual, 7)

	test.That(t, p.Start(ctx, spatialmath.NewPoseFromPoint(r3.Vector{X: 1, Y: 2, Z: 3})), test.ShouldBeNil)
	var req pb.MoveRequest
	test.That(t, protojson.Unmarshal([]byte(rec.last()["teleop_start"].(string)), &req), test.ShouldBeNil)
	test.That(t, req.ComponentName, test.ShouldEqual, "xarm-gripper")
	test.That(t, req.Name, test.ShouldEqual, "builtin")
	test.That(t, req.Destination.ReferenceFrame, test.ShouldEqual, WorldFrame)
	test.That(t, req.Destination.Pose.Z, test.ShouldEqual, 3)

	test.That(t, p.Move(ctx, spatialmath.NewPoseFromPoint(r3.Vector{X: 4})), test.ShouldBeNil)
	test.That(t, p.Move(ctx, spatialmath.NewPoseFromPoint(r3.Vector{X: 5})), test.ShouldBeNil)
	m := rec.last()
	var pif commonpb.PoseInFrame
	test.That(t, protojson.Unmarshal([]byte(m["teleop_move"].(string)), &pif), test.ShouldBeNil)
	test.That(t, pif.Pose.X, test.ShouldEqual, 5)
	test.That(t, m["component_name"], test.ShouldEqual, "xarm-gripper")
	test.That(t, m["seq"], test.ShouldEqual, 2.0) // increasing, as a float64 for structpb

	test.That(t, p.Stop(ctx), test.ShouldBeNil)
	test.That(t, rec.last()["teleop_stop"], test.ShouldEqual, true)
	test.That(t, rec.last()["component_name"], test.ShouldEqual, "xarm-gripper")

	st, err := p.Status(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, st["error"], test.ShouldEqual, "boom")
}

func TestIsSessionLostMatchesBothServerPaths(t *testing.T) {
	test.That(t, IsSessionLost(errors.New("teleop_move: teleop pipeline is not running; call teleop_start first")), test.ShouldBeTrue)
	test.That(t, IsSessionLost(errors.New(`component "g" not registered in teleop pipeline; registered: [h]`)), test.ShouldBeTrue)
	test.That(t, IsSessionLost(errors.New("context deadline exceeded")), test.ShouldBeFalse)
	test.That(t, IsSessionLost(nil), test.ShouldBeFalse)
}

func TestSenderKeepsOneMoveInFlightAndDropsTheRest(t *testing.T) {
	rec := &recorder{block: make(chan struct{})}
	s := NewSender(New(rec, "builtin", "g"), time.Second)
	test.That(t, s.TrySend(spatialmath.NewPoseFromPoint(r3.Vector{X: 1})), test.ShouldBeTrue)
	test.That(t, s.TrySend(spatialmath.NewPoseFromPoint(r3.Vector{X: 2})), test.ShouldBeFalse)
	test.That(t, s.TrySend(spatialmath.NewPoseFromPoint(r3.Vector{X: 3})), test.ShouldBeFalse)
	c := s.Snapshot()
	test.That(t, c.Sends, test.ShouldEqual, 1)
	test.That(t, c.Dropped, test.ShouldEqual, 2)
	test.That(t, s.LastSent().Point().X, test.ShouldEqual, 1)
	close(rec.block)
	s.Wait()
	test.That(t, rec.count(), test.ShouldEqual, 1)
	test.That(t, s.TrySend(spatialmath.NewPoseFromPoint(r3.Vector{X: 4})), test.ShouldBeTrue)
	s.Wait()
	test.That(t, s.Snapshot().Sends, test.ShouldEqual, 2)
	test.That(t, s.Snapshot().LastRPCMs, test.ShouldBeGreaterThanOrEqualTo, 0)
}

func TestSenderReportsErrorsOnceAndFlagsALostSession(t *testing.T) {
	rec := &recorder{err: errors.New("teleop pipeline is not running")}
	s := NewSender(New(rec, "builtin", "g"), time.Second)
	test.That(t, s.TrySend(spatialmath.NewZeroPose()), test.ShouldBeTrue)
	s.Wait()
	test.That(t, s.Lost(), test.ShouldBeTrue)
	err := s.TakeErr()
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, s.TakeErr(), test.ShouldBeNil)
	// Lost sessions refuse further sends until Reset.
	test.That(t, s.TrySend(spatialmath.NewZeroPose()), test.ShouldBeFalse)
	s.Reset()
	test.That(t, s.Lost(), test.ShouldBeFalse)
	test.That(t, s.Snapshot().Sends, test.ShouldEqual, 0)
	test.That(t, s.LastSent(), test.ShouldBeNil)
}
