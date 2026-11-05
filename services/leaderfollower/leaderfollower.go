// Package leaderfollower implements viam:teleop:leader-follower, a generic service that
// mirrors one or more leader arms' end-effector poses onto follower arms of different
// kinematics through the builtin motion service's teleop pipeline.
//
// One service drives every leader/follower pair on a machine: top-level attributes are
// defaults shared by every pair, and each entry of "pairs" overrides what it needs (normally
// just the arm names and the origin). start/stop/status act on every pair unless given a
// "pair". The service is a thin resource wrapper; the loop, alignment and mapping live in
// internal/leaderfollower so the standalone client (cmd/teleop-client) runs the same code.
package leaderfollower

import (
	"context"
	"errors"
	"fmt"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"

	engine "teleop/internal/leaderfollower"
	"teleop/internal/pairs"
	"teleop/internal/pipeline"
)

// Model is viam:teleop:leader-follower.
var Model = resource.NewModel("viam", "teleop", "leader-follower")

func init() {
	resource.RegisterService(
		generic.API,
		Model,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newService,
		})
}

// Config is the service's attributes.
//
// The embedded Settings are the defaults every pair inherits. With no "pairs" they ARE the
// one pair (named "main"), so a single-arm config is just the flat attribute block the
// standalone client takes. Each entry of Pairs is a "name" plus any Settings keys, overlaid on
// the defaults key by key.
//
// The ",squash" is load-bearing. rdk decodes attributes with mapstructure keyed on the json
// tag and WITHOUT its global Squash option, so an untagged embedded struct is read as a
// nested "Settings" key and every attribute is silently dropped. encoding/json ignores the
// unknown option and inlines the embedded struct as before. TestAttributesDecodeThroughRDK
// pins this.
type Config struct {
	engine.Settings `json:",squash"`
	Pairs           []map[string]interface{} `json:"pairs,omitempty"`
	AutoStart       bool                     `json:"auto_start,omitempty"`
}

func (cfg *Config) resolvePairs() ([]pairs.Pair[engine.Settings], error) {
	return pairs.Resolve(cfg.Settings, cfg.Pairs, engine.Settings.Validate)
}

// Validate checks the attributes and declares the dependencies of every pair. Each follower
// component is declared so the frame system contains it before the service starts, even
// though the service never calls it directly (the motion service does).
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
		add(p.Settings.LeaderArm, p.Settings.FollowerComponent, p.Settings.MotionName(),
			p.Settings.LeaderGripper, p.Settings.FollowerGripper)
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

	s := &service{Named: conf.ResourceName().AsNamed(), logger: logger}
	for _, p := range resolved {
		e, err := buildEngine(deps, p, logger)
		if err != nil {
			return nil, err
		}
		s.engines = append(s.engines, namedEngine{name: p.Name, engine: e})
	}
	if cfg.AutoStart {
		if _, err := s.forEach(ctx, "", "start", (*engine.Engine).Start); err != nil {
			// viam-server retries a failed constructor; release whatever did start so the
			// retry never finds a leader already held.
			_, _ = s.forEach(ctx, "", "stop", (*engine.Engine).Stop)
			return nil, fmt.Errorf("auto_start failed: %w", err)
		}
	}
	return s, nil
}

func buildEngine(deps resource.Dependencies, p pairs.Pair[engine.Settings], logger logging.Logger) (*engine.Engine, error) {
	s := p.Settings
	engineCfg, err := s.Resolve()
	if err != nil {
		return nil, fmt.Errorf("pair %q: %w", p.Name, err)
	}
	leader, err := arm.FromProvider(deps, s.LeaderArm)
	if err != nil {
		return nil, fmt.Errorf("pair %q: failed to get leader_arm %q: %w", p.Name, s.LeaderArm, err)
	}
	motionSvc, err := motion.FromProvider(deps, s.MotionName())
	if err != nil {
		return nil, fmt.Errorf("pair %q: failed to get motion_service %q: %w", p.Name, s.MotionName(), err)
	}
	eDeps := engine.Deps{
		Leader:   leader,
		Pipeline: pipeline.New(motionSvc, s.MotionName(), s.FollowerComponent),
		Logger:   logger.Sublogger(p.Name),
	}
	// A nil interface must stay nil: a typed nil gripper would be "present" to the engine,
	// so assign only on success rather than through a typed variable.
	if s.LeaderGripper != "" {
		g, err := gripper.FromProvider(deps, s.LeaderGripper)
		if err != nil {
			return nil, fmt.Errorf("pair %q: failed to get leader_gripper %q: %w", p.Name, s.LeaderGripper, err)
		}
		eDeps.LeaderGripper = g
	}
	if s.FollowerGripper != "" {
		g, err := gripper.FromProvider(deps, s.FollowerGripper)
		if err != nil {
			return nil, fmt.Errorf("pair %q: failed to get follower_gripper %q: %w", p.Name, s.FollowerGripper, err)
		}
		eDeps.FollowerGripper = g
	}
	e, err := engine.New(eDeps, engineCfg)
	if err != nil {
		return nil, fmt.Errorf("pair %q: %w", p.Name, err)
	}
	return e, nil
}

// forEach runs fn on the named pair, or on every pair when pair is empty. Every pair is
// attempted even if an earlier one fails: one side's align failing must not leave the other
// side unstarted (or, for stop, still holding its leader). It returns the names it acted on.
// Each pair's failure is also logged under its own name: the joined error only reaches the
// DoCommand caller, and a start that fails after the align move leaves the leader stiff at
// the aligned pose with nothing in the machine logs to say why.
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

// DoCommand accepts start, stop and status, each optionally scoped with "pair": "<name>".
// start reports a failing pair as an error but still starts the others; status then shows
// which ones are running.
func (s *service) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	pair, err := pairs.Scope(cmd)
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
	default:
		return nil, fmt.Errorf("unknown command %v; expected start, stop or status", cmd["command"])
	}
}

// statusMap reports {"running": any pair running, "pairs": {name: status}}. Values are kept to
// structpb-native types (no []float64), because DoCommand results cross gRPC as a Struct.
func (s *service) statusMap(pair string) (map[string]interface{}, error) {
	out := map[string]interface{}{}
	anyRunning := false
	for _, ne := range s.engines {
		if pair != "" && ne.name != pair {
			continue
		}
		st := ne.engine.Status()
		anyRunning = anyRunning || st.Running
		ps := map[string]interface{}{
			"running":                     st.Running,
			"session_id":                  st.SessionID,
			"stopped_reason":              string(st.StoppedReason),
			"leader_held":                 st.LeaderHeld,
			"last_hold_error":             st.LastHoldError,
			"cycles":                      st.Cycles,
			"sends":                       st.Sends,
			"dropped":                     st.Dropped,
			"speed_limited":               st.SpeedLimited,
			"deadzoned":                   st.Deadzoned,
			"rejected":                    st.Rejected,
			"tick_overruns":               st.TickOverruns,
			"consecutive_errors":          st.ConsecutiveErrors,
			"errors_total":                st.ErrorsTotal,
			"last_error":                  st.LastError,
			"last_error_at":               st.LastErrorAt,
			"last_rpc_ms":                 st.LastRPCMs,
			"leader_read_ms":              st.LeaderReadMs,
			"gripper_read_ms":             st.GripperReadMs,
			"rate_hz":                     st.RateHz,
			"align_position_error_mm":     st.AlignPosErrMM,
			"align_orientation_error_deg": st.AlignOriErrDeg,
			"gripper_pct":                 st.GripperPct,
			"gripper_writes":              st.GripperWrites,
			"pipeline_error":              st.PipelineError,
			"pipeline_plan_ms":            st.PipelinePlanMs,
		}
		if al := ne.engine.Align(); al != nil {
			ps["align_poses"] = al.PosesMap()
		}
		out[ne.name] = ps
	}
	if pair != "" && len(out) == 0 {
		return nil, fmt.Errorf("unknown pair %q; configured: %v", pair, s.pairNames())
	}
	return map[string]interface{}{"running": anyRunning, "pairs": out}, nil
}

func (s *service) Close(ctx context.Context) error {
	_, err := s.forEach(ctx, "", "stop", (*engine.Engine).Stop)
	return err
}
