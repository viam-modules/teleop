// Command teleop-client runs the leader→follower mirror from a laptop, against live machines
// over gRPC, using exactly the code the viam:teleop:leader-follower service runs on a
// machine. It exists to try a mapping, scale or hold mode without reconfiguring a robot.
//
//	go run ./cmd/teleop-client -config teleop.json -leader-addr <machine>.viam.cloud
//
// The file takes the same attributes as the service, including "pairs"; every pair runs in
// this one process over one connection. The follower may live on a second machine
// (-follower-addr); credentials come from flags or the VIAM_API_KEY_ID / VIAM_API_KEY
// environment variables, and apply to both machines unless -follower-key-id / -follower-key
// are given. A leader served by a local viam-server with no cloud config (so no TLS and no
// auth) is dialled with -leader-insecure.
package main

import (
	"bytes"
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/utils/rpc"

	engine "teleop/internal/leaderfollower"
	"teleop/internal/pairs"
	"teleop/internal/pipeline"
)

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, "error:", err)
		os.Exit(1)
	}
}

// fileConfig is the service's attribute block as a file: the defaults plus an optional
// pairs list. Unknown keys are rejected, so a typo cannot silently fall back to a default.
type fileConfig struct {
	engine.Settings
	Pairs []map[string]interface{} `json:"pairs,omitempty"`
}

func run() error {
	var (
		configPath       = flag.String("config", "", "JSON file with the service attributes, including an optional \"pairs\" list (required)")
		leaderAddr       = flag.String("leader-addr", "", "address of the machine with the leader arm (required)")
		followerAddr     = flag.String("follower-addr", "", "address of the machine with the follower and its motion service (default: same as leader)")
		keyID            = flag.String("key-id", os.Getenv("VIAM_API_KEY_ID"), "API key id (or VIAM_API_KEY_ID)")
		key              = flag.String("key", os.Getenv("VIAM_API_KEY"), "API key (or VIAM_API_KEY)")
		followerKeyID    = flag.String("follower-key-id", "", "API key id for the follower machine (default: -key-id)")
		followerKey      = flag.String("follower-key", "", "API key for the follower machine (default: -key)")
		leaderInsecure   = flag.Bool("leader-insecure", false, "dial the leader machine without TLS or credentials (a local viam-server with no cloud config, e.g. localhost:8080)")
		followerInsecure = flag.Bool("follower-insecure", false, "dial the follower machine without TLS or credentials")
		onlyPair         = flag.String("pair", "", "run only this pair of the config's \"pairs\" list")
		statusEvery      = flag.Duration("status-every", 2*time.Second, "how often to print status; 0 disables")
		alignOnly        = flag.Bool("align-only", false, "align the leader to the follower, print the residual, and exit without mirroring")
		debug            = flag.Bool("debug", false, "debug logging")
	)
	flag.Parse()
	if *configPath == "" || *leaderAddr == "" {
		flag.Usage()
		return fmt.Errorf("-config and -leader-addr are required")
	}

	raw, err := os.ReadFile(*configPath)
	if err != nil {
		return err
	}
	var fc fileConfig
	dec := json.NewDecoder(bytes.NewReader(raw))
	dec.DisallowUnknownFields()
	if err := dec.Decode(&fc); err != nil {
		return fmt.Errorf("parse %s: %w", *configPath, err)
	}
	resolved, err := pairs.Resolve(fc.Settings, fc.Pairs, engine.Settings.Validate)
	if err != nil {
		return fmt.Errorf("%s: %w", *configPath, err)
	}
	if *onlyPair != "" {
		var keep []pairs.Pair[engine.Settings]
		for _, p := range resolved {
			if p.Name == *onlyPair {
				keep = append(keep, p)
			}
		}
		if len(keep) == 0 {
			return fmt.Errorf("-pair %q not in config", *onlyPair)
		}
		resolved = keep
	}

	logger := logging.NewLogger("teleop-client")
	if *debug {
		logger.SetLevel(logging.DEBUG)
	}
	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer cancel()

	leaderRobot, err := dial(ctx, *leaderAddr, *keyID, *key, *leaderInsecure, logger)
	if err != nil {
		return fmt.Errorf("connect to leader machine: %w", err)
	}
	defer leaderRobot.Close(context.Background())
	followerRobot := leaderRobot
	if *followerAddr != "" && *followerAddr != *leaderAddr {
		fkid, fk := *followerKeyID, *followerKey
		if fkid == "" {
			fkid, fk = *keyID, *key
		}
		if followerRobot, err = dial(ctx, *followerAddr, fkid, fk, *followerInsecure, logger); err != nil {
			return fmt.Errorf("connect to follower machine: %w", err)
		}
		defer followerRobot.Close(context.Background())
	}

	type running struct {
		name   string
		engine *engine.Engine
	}
	var engines []running
	stopAll := func() error {
		var firstErr error
		for _, r := range engines {
			stopCtx, stopCancel := context.WithTimeout(context.Background(), 10*time.Second)
			if err := r.engine.Stop(stopCtx); err != nil && firstErr == nil {
				firstErr = fmt.Errorf("%s: %w", r.name, err)
			}
			stopCancel()
		}
		return firstErr
	}

	for _, p := range resolved {
		s := p.Settings
		cfg, err := s.Resolve()
		if err != nil {
			return fmt.Errorf("%s: %w", p.Name, err)
		}
		leader, err := arm.FromProvider(leaderRobot, s.LeaderArm)
		if err != nil {
			return err
		}
		motionSvc, err := motion.FromProvider(followerRobot, s.MotionName())
		if err != nil {
			return err
		}
		deps := engine.Deps{
			Leader:   leader,
			Pipeline: pipeline.New(motionSvc, s.MotionName(), s.FollowerComponent),
			Logger:   logger.Sublogger(p.Name),
		}
		if s.LeaderGripper != "" {
			g, err := gripper.FromProvider(leaderRobot, s.LeaderGripper)
			if err != nil {
				return err
			}
			deps.LeaderGripper = g
		}
		if s.FollowerGripper != "" {
			g, err := gripper.FromProvider(followerRobot, s.FollowerGripper)
			if err != nil {
				return err
			}
			deps.FollowerGripper = g
		}
		e, err := engine.New(deps, cfg)
		if err != nil {
			return fmt.Errorf("%s: %w", p.Name, err)
		}
		if err := e.Start(ctx); err != nil {
			// One pair failing (an align the leader cannot reach, say) should not take the
			// others down with it; report it and carry on with whatever started.
			fmt.Printf("[%s] NOT STARTED: %v\n", p.Name, err)
			continue
		}
		engines = append(engines, running{name: p.Name, engine: e})
		al := e.Align()
		if al.Aligned {
			fmt.Printf("[%s] aligned: leader within %.1f mm / %.1f deg of the follower; leader joints %v\n",
				p.Name, al.PositionErrorMM, al.OrientationErrorDeg, al.LeaderInputs)
		} else {
			fmt.Printf("[%s] captured: mirroring deltas from the leader's pose (%.0f mm / %.0f deg offset absorbed)\n",
				p.Name, al.PositionErrorMM, al.OrientationErrorDeg)
		}
	}
	if len(engines) == 0 {
		return fmt.Errorf("no pair started")
	}
	if *alignOnly {
		return stopAll()
	}
	fmt.Println("mirroring; Ctrl-C to stop")

	var tick <-chan time.Time
	if *statusEvery > 0 {
		ticker := time.NewTicker(*statusEvery)
		defer ticker.Stop()
		tick = ticker.C
	}
	for {
		select {
		case <-ctx.Done():
			return stopAll()
		case <-tick:
			allStopped := true
			for _, r := range engines {
				st := r.engine.Status()
				fmt.Printf("[%s] running=%v cycles=%d sends=%d dropped=%d deadzoned=%d rejected=%d overruns=%d rpc=%.1fms read=%.1fms plan=%.0fms gripper=%.0f%% err=%q pipeline=%q\n",
					r.name, st.Running, st.Cycles, st.Sends, st.Dropped, st.Deadzoned, st.Rejected, st.TickOverruns,
					st.LastRPCMs, st.LeaderReadMs, st.PipelinePlanMs, st.GripperPct, st.LastError, st.PipelineError)
				if st.Running {
					allStopped = false
				} else {
					fmt.Printf("[%s] mirror loop stopped (%s): %s (leader left held; Ctrl-C releases it)\n", r.name, st.StoppedReason, st.LastError)
				}
			}
			if allStopped {
				return fmt.Errorf("every mirror loop stopped (stop: %v)", stopAll())
			}
		}
	}
}

func dial(ctx context.Context, addr, keyID, key string, insecure bool, logger logging.Logger) (*client.RobotClient, error) {
	opts := []client.RobotClientOption{client.WithRefreshEvery(0)}
	switch {
	case insecure:
		opts = append(opts, client.WithDialOptions(rpc.WithInsecure()))
	case keyID != "":
		opts = append(opts, client.WithDialOptions(rpc.WithEntityCredentials(keyID, rpc.Credentials{
			Type:    rpc.CredentialsTypeAPIKey,
			Payload: key,
		})))
	}
	return client.New(ctx, addr, logger, opts...)
}

var _ = strings.TrimSpace // keep strings for the flag help formatting below if extended
