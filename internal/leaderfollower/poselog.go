package leaderfollower

import (
	"math"
	"time"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"teleop/internal/posemath"
)

// Debug pose logging. Every pose is reported as a flat map of rounded float64s
// ({x, y, z} in mm, {ox, oy, oz, theta} as an orientation vector in degrees) so the same
// value can go into a structured log line and, unchanged, into a structpb DoCommand result.

// poseLogInterval rate-limits the run loop's pose line to one per pair per interval.
const poseLogInterval = time.Second

// inputsDeg renders revolute joint inputs (radians) as rounded degrees.
func inputsDeg(in []referenceframe.Input) []interface{} {
	out := make([]interface{}, len(in))
	for i, v := range in {
		out[i] = posemath.Round1(float64(v) * 180 / math.Pi)
	}
	return out
}

// residualMap renders the residual's translation (mm) and rotation (orientation vector, deg,
// plus its total angle).
func residualMap(r Residual) map[string]interface{} {
	m := map[string]interface{}{
		"x": posemath.Round1(r.Translation.X), "y": posemath.Round1(r.Translation.Y), "z": posemath.Round1(r.Translation.Z),
		"translation_mm": posemath.Round1(r.Translation.Norm()),
		"rotation_deg":   0.0,
	}
	if r.Rotation != nil {
		ov := r.Rotation.OrientationVectorDegrees()
		m["ox"], m["oy"], m["oz"], m["theta"] = posemath.Round1(ov.OX), posemath.Round1(ov.OY), posemath.Round1(ov.OZ), posemath.Round1(ov.Theta)
		m["rotation_deg"] = posemath.Round1(posemath.OrientationError(spatialmath.NewZeroOrientation(), r.Rotation).Norm() * 180 / math.Pi)
	}
	return m
}

// PosesMap is the alignment's poses for the status DoCommand: structpb-compatible (only
// float64, string, bool, []interface{} and map[string]interface{}).
func (al *Alignment) PosesMap() map[string]interface{} {
	m := map[string]interface{}{
		"aligned":               al.Aligned,
		"position_error_mm":     posemath.Round1(al.PositionErrorMM),
		"orientation_error_deg": posemath.Round1(al.OrientationErrorDeg),
		"leader_inputs_deg":     inputsDeg(al.LeaderInputs),
		"residual":              residualMap(al.Residual),
	}
	for k, p := range map[string]spatialmath.Pose{
		"follower_pose": al.FollowerPose,
		"leader_target": al.LeaderTarget,
		"leader_actual": al.LeaderActual,
		"mapped":        al.Mapped,
	} {
		if pm := posemath.PoseMap(p); pm != nil {
			m[k] = pm
		}
	}
	return m
}

// logAlign writes the one structured align line.
func (e *Engine) logAlign(al *Alignment) {
	e.deps.Logger.Infow("leader-follower align poses",
		"session", e.sessionID,
		"aligned", al.Aligned,
		"follower_pose", posemath.PoseMap(al.FollowerPose),
		"want", posemath.PoseMap(al.LeaderTarget),
		"leader_inputs_deg", inputsDeg(al.LeaderInputs),
		"leader_actual", posemath.PoseMap(al.LeaderActual),
		"mapped", posemath.PoseMap(al.Mapped),
		"residual", residualMap(al.Residual),
		"position_error_mm", posemath.Round1(al.PositionErrorMM),
		"orientation_error_deg", posemath.Round1(al.OrientationErrorDeg),
	)
}

// maybeLogPose writes the run loop's pose line, at most once per poseLogInterval. leader is
// the leader's end pose, target the mapped follower target (residual applied), and
// follower_cmd the last target handed to the pipeline (the follower's commanded pose).
func (e *Engine) maybeLogPose(leader, target spatialmath.Pose) {
	now := e.deps.Now()
	last := e.poseLogAt.Load()
	if last != 0 && now.Sub(time.Unix(0, last)) < poseLogInterval {
		return
	}
	if !e.poseLogAt.CompareAndSwap(last, now.UnixNano()) {
		return
	}
	e.deps.Logger.Infow("leader-follower pose",
		"session", e.sessionID,
		"leader", posemath.PoseMap(leader),
		"target", posemath.PoseMap(target),
		"follower_cmd", posemath.PoseMap(e.sender.LastSent()),
		"leader_read_ms", posemath.Round1(float64(e.leaderReadNanos.Load())/1e6),
	)
}
