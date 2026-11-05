// Package clutch implements viam:teleop:clutch, a generic service that drives a follower arm
// from a hand only between a start and a stop.
//
// The service does not read the clutch. Whatever owns it (a footswitch actions service
// mapping press to start and release to stop, a wand's grip) sends the DoCommands; start
// captures where the hand and the follower are and mirrors the hand from there, stop lets
// the follower go. A pair's hand is a body in a pose tracker or a leader arm's end effector.
// One service runs every pair on a machine; start and stop take a "pair". The loop lives in
// internal/clutch.
package clutch

import (
	"context"
	"errors"
	"fmt"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"

	engine "teleop/internal/clutch"
	"teleop/internal/pairs"
	"teleop/internal/pipeline"
)

// Model is viam:teleop:clutch.
var Model = resource.NewModel("viam", "teleop", "clutch")

func init() {
	resource.RegisterService(
		generic.API,
		Model,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newService,
		})
}

// Config is the service's attributes: the embedded Settings are the defaults every pair
// inherits (with no "pairs" they are the one pair, named "main"); each entry of Pairs is a
// "name" plus any Settings keys. The ",squash" is load-bearing: rdk decodes attributes with
// mapstructure on the json tag, and an untagged embedded struct would be read as a nested
// key.
type Config struct {
	engine.Settings `json:",squash"`
	Pairs           []map[string]interface{} `json:"pairs,omitempty"`
}

func (cfg *Config) resolvePairs() ([]pairs.Pair[engine.Settings], error) {
	return pairs.Resolve(cfg.Settings, cfg.Pairs, engine.Settings.Validate)
}

// Validate checks the attributes and declares the dependencies of every pair. The frame
// system is not declared: viam-server hands every module resource a client for it.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	resolved, err := cfg.resolvePairs()
	if err != nil {
		return nil, nil, fmt.Errorf("%s: %w", path, err)
	}
	var deps []string
	seen := map[string]bool{}
	add := func(names ...string) {
		for _, n := range names {
			if n != "" && !seen[n] {
				seen[n] = true
				deps = append(deps, n)
			}
		}
	}
	for _, p := range resolved {
		s := p.Settings
		add(s.PoseTracker, s.LeaderArm, s.FollowerComponent, s.MotionName(),
			s.FollowerGripper, s.LeaderGripper, s.CaptureSensor)
	}
	return deps, nil, nil
}

type namedEngine struct {
	name   string
	engine *engine.Engine
}

type service struct {
	resource.Named
	resource.AlwaysRebuild
	logger  logging.Logger
	engines []namedEngine
}

func newService(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (resource.Resource, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}
	resolved, err := cfg.resolvePairs()
	if err != nil {
		return nil, err
	}
	// Without a frame system (tests, or a server that does not hand one out) the source frame
	// is taken to be world; the engine warns once per frame when it matters.
	var frames engine.Frames
	if fs, err := framesystem.FromProvider(deps); err == nil {
		frames = fs
	} else {
		logger.Debugf("no frame system in dependencies: %v", err)
	}

	s := &service{Named: conf.ResourceName().AsNamed(), logger: logger}
	for _, p := range resolved {
		e, err := buildEngine(deps, frames, p, logger)
		if err != nil {
			return nil, err
		}
		s.engines = append(s.engines, namedEngine{name: p.Name, engine: e})
	}
	// Leaders go limp now and stay limp until the service closes: start and stop only
	// engage and release the follower.
	if _, err := s.forEach(ctx, "", "hold leader", (*engine.Engine).HoldLeader); err != nil {
		s.forEachRelease(ctx)
		return nil, err
	}
	return s, nil
}

func buildEngine(deps resource.Dependencies, frames engine.Frames, p pairs.Pair[engine.Settings], logger logging.Logger) (*engine.Engine, error) {
	s := p.Settings
	engineCfg, err := s.Resolve()
	if err != nil {
		return nil, fmt.Errorf("pair %q: %w", p.Name, err)
	}
	get := func(what, name string) string { return fmt.Sprintf("pair %q: failed to get %s %q", p.Name, what, name) }

	eDeps := engine.Deps{Frames: frames, Logger: logger.Sublogger(p.Name)}
	if s.HasArmSource() {
		leader, err := arm.FromProvider(deps, s.LeaderArm)
		if err != nil {
			return nil, fmt.Errorf("%s: %w", get("leader_arm", s.LeaderArm), err)
		}
		eDeps.Source = &engine.ArmSource{Arm: leader, Name: s.LeaderArm, ToolOffset: s.ToolOffset()}
		eDeps.Leader = leader
	} else {
		tracker, err := posetracker.FromProvider(deps, s.PoseTracker)
		if err != nil {
			return nil, fmt.Errorf("%s: %w", get("pose_tracker", s.PoseTracker), err)
		}
		eDeps.Source = &engine.TrackerSource{Tracker: tracker, Name: s.PoseTracker, Body: s.Body}
	}
	motionSvc, err := motion.FromProvider(deps, s.MotionName())
	if err != nil {
		return nil, fmt.Errorf("%s: %w", get("motion_service", s.MotionName()), err)
	}
	eDeps.Pipeline = pipeline.New(motionSvc, s.MotionName(), s.FollowerComponent)
	// A nil interface must stay nil: assign only on success.
	if s.LeaderGripper != "" {
		g, err := gripper.FromProvider(deps, s.LeaderGripper)
		if err != nil {
			return nil, fmt.Errorf("%s: %w", get("leader_gripper", s.LeaderGripper), err)
		}
		eDeps.LeaderGripper = g
	}
	if s.FollowerGripper != "" {
		g, err := gripper.FromProvider(deps, s.FollowerGripper)
		if err != nil {
			return nil, fmt.Errorf("%s: %w", get("follower_gripper", s.FollowerGripper), err)
		}
		eDeps.FollowerGripper = g
	}
	if s.CaptureSensor != "" {
		c, err := sensor.FromProvider(deps, s.CaptureSensor)
		if err != nil {
			return nil, fmt.Errorf("%s: %w", get("capture_control_sensor", s.CaptureSensor), err)
		}
		eDeps.Capture = c
	}
	e, err := engine.New(eDeps, engineCfg)
	if err != nil {
		return nil, fmt.Errorf("pair %q: %w", p.Name, err)
	}
	return e, nil
}

// forEach runs fn on the named pair, or on every pair when pair is empty. Every pair is
// attempted even if an earlier one fails. Each failure is logged under its own name as well
// as joined into the returned error.
func (s *service) forEach(ctx context.Context, pair, what string, fn func(*engine.Engine, context.Context) error) ([]string, error) {
	var names []string
	var errs []error
	for _, ne := range s.engines {
		if pair != "" && ne.name != pair {
			continue
		}
		names = append(names, ne.name)
		if err := fn(ne.engine, ctx); err != nil {
			s.logger.Errorf("pair %q: %s failed: %v", ne.name, what, err)
			errs = append(errs, fmt.Errorf("%s: %w", ne.name, err))
		}
	}
	if pair != "" && len(names) == 0 {
		return nil, fmt.Errorf("unknown pair %q; configured: %v", pair, s.pairNames())
	}
	return names, errors.Join(errs...)
}

func (s *service) pairNames() []string {
	out := make([]string, len(s.engines))
	for i, ne := range s.engines {
		out[i] = ne.name
	}
	return out
}

// DoCommand accepts start, stop, status and set_rotation_mode, each optionally scoped with
// "pair": "<name>" ("side" is an alias); without it, it acts on every pair. start engages
// (captures both poses, mirrors); stop releases the follower. Both are idempotent, so a
// press-to-start / release-to-stop mapping can send them freely. Leader holds are applied
// on configure and released on close, never by start or stop.
func (s *service) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	pair, err := pairs.Scope(cmd, "mode")
	if err != nil {
		return nil, err
	}
	switch cmd["command"] {
	case "start":
		if _, err := s.forEach(ctx, pair, "start", (*engine.Engine).Start); err != nil {
			return nil, err
		}
		return s.statusMap(pair)
	case "stop":
		if _, err := s.forEach(ctx, pair, "stop", (*engine.Engine).Stop); err != nil {
			return nil, err
		}
		return s.statusMap(pair)
	case "status":
		return s.statusMap(pair)
	case "set_rotation_mode":
		mode, _ := cmd["mode"].(string)
		if mode == "" {
			return nil, fmt.Errorf("set_rotation_mode needs \"mode\": absolute, relative or off")
		}
		if _, err := s.forEach(ctx, pair, "set_rotation_mode", func(e *engine.Engine, _ context.Context) error {
			return e.SetRotationMode(engine.RotationMode(mode))
		}); err != nil {
			return nil, err
		}
		return s.statusMap(pair)
	default:
		return nil, fmt.Errorf("unknown command %v; expected start, stop, status or set_rotation_mode", cmd["command"])
	}
}

// statusMap reports {"running": any pair running, "pairs": {name: status}}. Values are
// structpb-native types, because DoCommand results cross gRPC as a Struct.
func (s *service) statusMap(pair string) (map[string]interface{}, error) {
	out := map[string]interface{}{}
	anyRunning := false
	for _, ne := range s.engines {
		if pair != "" && ne.name != pair {
			continue
		}
		st := ne.engine.Status()
		anyRunning = anyRunning || st.Running
		out[ne.name] = map[string]interface{}{
			"running":            st.Running,
			"session_id":         st.SessionID,
			"sessions":           st.Sessions,
			"stopped_reason":     string(st.StoppedReason),
			"tracked":            st.Tracked,
			"leader_held":        st.LeaderHeld,
			"last_hold_error":    st.LastHoldError,
			"cycles":             st.Cycles,
			"sends":              st.Sends,
			"dropped":            st.Dropped,
			"speed_limited":      st.SpeedLimited,
			"deadzoned":          st.Deadzoned,
			"rejected":           st.Rejected,
			"tick_overruns":      st.TickOverruns,
			"consecutive_errors": st.ConsecutiveErrors,
			"errors_total":       st.ErrorsTotal,
			"last_error":         st.LastError,
			"last_error_at":      st.LastErrorAt,
			"last_rpc_ms":        st.LastRPCMs,
			"source_read_ms":     st.SourceReadMs,
			"gripper_read_ms":    st.GripperReadMs,
			"rate_hz":            st.RateHz,
			"rotation_mode":      string(st.RotationMode),
			"gripper_pct":        st.GripperPct,
			"gripper_writes":     st.GripperWrites,
			"pipeline_error":     st.PipelineError,
			"pipeline_plan_ms":   st.PipelinePlanMs,
		}
	}
	if pair != "" && len(out) == 0 {
		return nil, fmt.Errorf("unknown pair %q; configured: %v", pair, s.pairNames())
	}
	return map[string]interface{}{"running": anyRunning, "pairs": out}, nil
}

func (s *service) forEachRelease(ctx context.Context) {
	for _, ne := range s.engines {
		ne.engine.ReleaseLeader(ctx)
	}
}

func (s *service) Close(ctx context.Context) error {
	_, err := s.forEach(ctx, "", "stop", (*engine.Engine).Stop)
	s.forEachRelease(ctx)
	return err
}
