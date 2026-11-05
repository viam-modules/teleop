// Package clutch drives a follower arm from a hand-held or hand-guided input only while a
// clutch control is held: a controller's grip, a foot pedal. Every press captures a fresh
// reference on both sides (where the hand is, where the follower is) and the hand's motion
// from there is mirrored as scaled deltas onto the follower through viam-server's builtin
// motion service teleop pipeline. Releasing stops the follower where it is; the operator
// repositions freely and presses again.
//
// Where the hand's pose comes from is a Source: a body in a pose tracker (a Vive wand), or
// a leader arm's end effector (an SO-101 held by the hand with the pedal under the foot).
// Everything after the read is the same.
package clutch

import (
	"context"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// Sample is one read of the hand.
type Sample struct {
	// Pose of the hand in Frame. Nil when not Tracked.
	Pose spatialmath.Pose
	// Frame is the frame-system name Pose is expressed in. The engine asks the frame system
	// once per engage for this frame's rotation into world.
	Frame string
	// Tracked is false when the device has no fix right now (a body missing from a pose
	// tracker's map). It is not an error: the engine releases if engaged and waits.
	Tracked bool
}

// Source is where the hand's pose comes from.
type Source interface {
	Read(ctx context.Context) (Sample, error)
	// String names the source for logs.
	String() string
}

// PoseTracker is the slice of posetracker.PoseTracker a TrackerSource needs.
type PoseTracker interface {
	Poses(ctx context.Context, bodyNames []string, extra map[string]interface{}) (referenceframe.FrameSystemPoses, error)
}

// TrackerSource reads one body of a pose tracker. A body the tracker leaves out of its map
// is "not tracked". The pose's frame is the PoseInFrame's parent, or the tracker's own
// resource name when the tracker does not say.
type TrackerSource struct {
	Tracker PoseTracker
	Name    string // the tracker's resource name
	Body    string
}

// Read implements Source.
func (s *TrackerSource) Read(ctx context.Context) (Sample, error) {
	poses, err := s.Tracker.Poses(ctx, []string{s.Body}, nil)
	if err != nil {
		return Sample{}, fmt.Errorf("read pose tracker %s: %w", s.Name, err)
	}
	pif, ok := poses[s.Body]
	if !ok || pif == nil || pif.Pose() == nil {
		return Sample{Frame: s.Name}, nil
	}
	frame := pif.Parent()
	if frame == "" {
		frame = s.Name
	}
	return Sample{Pose: pif.Pose(), Frame: frame, Tracked: true}, nil
}

func (s *TrackerSource) String() string {
	return fmt.Sprintf("pose tracker %s body %s", s.Name, s.Body)
}

// ArmReader is the slice of arm.Arm an ArmSource needs.
type ArmReader interface {
	EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error)
}

// ArmSource reads a leader arm's end-effector pose. EndPosition is in the arm's base frame,
// which the frame system names "<arm>_origin" when the arm has a frame block.
type ArmSource struct {
	Arm  ArmReader
	Name string // the arm's resource name
	// ToolOffset moves the mirrored point from the reported end-effector frame out to the
	// physical tip, in the tool frame. Without it, twisting a leader whose EndPosition is a
	// wrist frame behind the handle translates the follower by scale × the lever arm.
	ToolOffset r3.Vector
}

// Read implements Source. A read error is an error (the arm is wired, not tracked).
func (s *ArmSource) Read(ctx context.Context) (Sample, error) {
	p, err := s.Arm.EndPosition(ctx, nil)
	if err != nil {
		return Sample{}, fmt.Errorf("read leader arm %s: %w", s.Name, err)
	}
	if s.ToolOffset.Norm() != 0 {
		p = spatialmath.Compose(p, spatialmath.NewPoseFromPoint(s.ToolOffset))
	}
	return Sample{Pose: p, Frame: s.Name + "_origin", Tracked: true}, nil
}

func (s *ArmSource) String() string { return "arm " + s.Name }
