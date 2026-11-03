// Package gripper mirrors a 0..100 % opening onto a follower gripper, and reads one off a
// leader gripper, through configuration rather than any one module's DoCommand vocabulary.
//
// Every gripper module exposes its position differently today (the SO-101 answers
// {"command": "get_position"} with position_percentage; the uFactory xArm takes {"set":
// steps}; others differ again), so until the gripper API is position-agnostic the leader read
// command, its response key and the follower write template are user configuration. No
// vendor's names are defaulted here.
package gripper

import (
	"context"
	"fmt"
	"math"
	"strings"
	"sync"
	"sync/atomic"
	"time"
)

// Gripper is the slice of gripper.Gripper this package uses on either side.
type Gripper interface {
	Open(ctx context.Context, extra map[string]interface{}) error
	Grab(ctx context.Context, extra map[string]interface{}) (bool, error)
	DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error)
}

// Mode is how the follower gripper is driven from a percentage.
type Mode string

const (
	// ModeCommand sends the configured DoCommand template with $steps / $pct substituted.
	ModeCommand Mode = "command"
	// ModeOpenClose uses the gripper API: Open above 60 %, Grab below 40 %.
	ModeOpenClose Mode = "open_close"
	// ModeOff mirrors nothing.
	ModeOff Mode = "off"
	// modeXArmAlias is accepted for one release as an alias of ModeCommand, for configs
	// migrated from the so-101 service.
	modeXArmAlias Mode = "xarm"
)

// Tokens a command template may carry; they are replaced by numbers at any depth.
const (
	TokenSteps = "$steps" // round(pct / 100 * Range)
	TokenPct   = "$pct"   // the percentage itself
)

// Hysteresis thresholds for ModeOpenClose.
const (
	openAbovePct  = 60
	closeBelowPct = 40
)

// writeTimeout bounds one follower write. The xArm gripper's "set" blocks until the jaw
// arrives, which can take seconds.
const writeTimeout = 10 * time.Second

// OutputConfig configures an Output.
type OutputConfig struct {
	Mode Mode
	// Template is the DoCommand map sent in ModeCommand, carrying TokenSteps or TokenPct.
	Template map[string]interface{}
	// Range is the value of TokenSteps at 100 %. Default 850 (the xArm's scale) only because
	// that is the follower this module is gated on; it is a number, not a vendor name.
	Range float64
	// Invert flips the percentage before anything else: a leader whose trigger reads near
	// zero at rest needs it for squeeze-to-close.
	Invert bool
	// DeadbandPct is how much the percentage must change before a new write. Default 3.
	DeadbandPct float64
}

// NormalizeMode maps the migration alias onto ModeCommand and reports whether it did.
func NormalizeMode(m Mode) (Mode, bool) {
	if m == modeXArmAlias {
		return ModeCommand, true
	}
	return m, false
}

// ValidateTemplate checks that a ModeCommand template carries at least one token.
func ValidateTemplate(t map[string]interface{}) error {
	if len(t) == 0 {
		return fmt.Errorf("follower_gripper_command is required in %q mode", ModeCommand)
	}
	if !hasToken(t) {
		return fmt.Errorf("follower_gripper_command carries neither %q nor %q", TokenSteps, TokenPct)
	}
	return nil
}

func hasToken(v interface{}) bool {
	switch x := v.(type) {
	case string:
		return x == TokenSteps || x == TokenPct
	case map[string]interface{}:
		for _, e := range x {
			if hasToken(e) {
				return true
			}
		}
	case []interface{}:
		for _, e := range x {
			if hasToken(e) {
				return true
			}
		}
	}
	return false
}

// Substitute returns a copy of template with the tokens replaced by numbers for pct.
func Substitute(template map[string]interface{}, pct, rng float64) map[string]interface{} {
	out, _ := substitute(template, pct, rng).(map[string]interface{})
	return out
}

func substitute(v interface{}, pct, rng float64) interface{} {
	switch x := v.(type) {
	case string:
		switch x {
		case TokenSteps:
			return math.Round(pct / 100 * rng)
		case TokenPct:
			return pct
		}
		return x
	case map[string]interface{}:
		out := make(map[string]interface{}, len(x))
		for k, e := range x {
			out[k] = substitute(e, pct, rng)
		}
		return out
	case []interface{}:
		out := make([]interface{}, len(x))
		for i, e := range x {
			out[i] = substitute(e, pct, rng)
		}
		return out
	}
	return v
}

// Output drives a follower gripper from percentages, asynchronously and latest-wins: a
// percentage that arrives while a write is blocked is sent when the write returns, and any
// earlier pending value is discarded.
type Output struct {
	g   Gripper
	cfg OutputConfig

	inFlight atomic.Bool
	pending  atomic.Value // float64; NaN when empty
	wg       sync.WaitGroup

	mu      sync.Mutex
	sent    bool
	lastPct float64
	open    *bool // ModeOpenClose: last state sent
	err     error
	writes  uint64
}

// NewOutput returns an Output for g. g may be nil, in which case Set is a no-op.
func NewOutput(g Gripper, cfg OutputConfig) *Output {
	cfg.Mode, _ = NormalizeMode(cfg.Mode)
	if cfg.Range <= 0 {
		cfg.Range = 850
	}
	if cfg.DeadbandPct <= 0 {
		cfg.DeadbandPct = 3
	}
	o := &Output{g: g, cfg: cfg}
	o.pending.Store(math.NaN())
	return o
}

// Enabled reports whether Set does anything.
func (o *Output) Enabled() bool { return o.g != nil && o.cfg.Mode != ModeOff && o.cfg.Mode != "" }

// Reset forgets the session state so the next Set always writes.
func (o *Output) Reset() {
	o.Wait()
	o.mu.Lock()
	o.sent, o.open, o.err, o.writes = false, nil, nil, 0
	o.mu.Unlock()
}

// Wait blocks until no write is in flight.
func (o *Output) Wait() { o.wg.Wait() }

// Set offers a raw 0..100 percentage. It returns whether a write was scheduled (the value
// changed by more than the deadband).
func (o *Output) Set(rawPct float64) bool {
	if !o.Enabled() {
		return false
	}
	pct := math.Max(0, math.Min(100, rawPct))
	if o.cfg.Invert {
		pct = 100 - pct
	}
	o.mu.Lock()
	changed := !o.sent || math.Abs(pct-o.lastPct) >= o.cfg.DeadbandPct
	if changed {
		o.lastPct, o.sent = pct, true
	}
	o.mu.Unlock()
	if !changed {
		return false
	}
	if o.inFlight.CompareAndSwap(false, true) {
		o.wg.Add(1)
		go o.write(pct)
	} else {
		o.pending.Store(pct) // latest wins; sent when the current write returns
	}
	return true
}

// Pct is the last percentage accepted by Set, after inversion.
func (o *Output) Pct() float64 {
	o.mu.Lock()
	defer o.mu.Unlock()
	return o.lastPct
}

// Writes is how many follower writes were issued this session.
func (o *Output) Writes() uint64 {
	o.mu.Lock()
	defer o.mu.Unlock()
	return o.writes
}

// TakeErr returns and clears the most recent write error.
func (o *Output) TakeErr() error {
	o.mu.Lock()
	defer o.mu.Unlock()
	err := o.err
	o.err = nil
	return err
}

func (o *Output) write(pct float64) {
	defer o.wg.Done()
	defer o.inFlight.Store(false)
	for {
		ctx, cancel := context.WithTimeout(context.Background(), writeTimeout)
		err := o.writeOnce(ctx, pct)
		cancel()
		if err != nil {
			o.mu.Lock()
			o.err = err
			o.mu.Unlock()
		}
		next, _ := o.pending.Swap(math.NaN()).(float64)
		if math.IsNaN(next) {
			return
		}
		pct = next
	}
}

func (o *Output) writeOnce(ctx context.Context, pct float64) error {
	switch o.cfg.Mode {
	case ModeCommand:
		o.mu.Lock()
		o.writes++
		o.mu.Unlock()
		if _, err := o.g.DoCommand(ctx, Substitute(o.cfg.Template, pct, o.cfg.Range)); err != nil {
			return fmt.Errorf("write follower gripper: %w", err)
		}
	case ModeOpenClose:
		o.mu.Lock()
		prev := o.open
		o.mu.Unlock()
		var want *bool
		switch {
		case pct > openAbovePct && (prev == nil || !*prev):
			t := true
			want = &t
		case pct < closeBelowPct && (prev == nil || *prev):
			f := false
			want = &f
		}
		if want == nil {
			return nil
		}
		o.mu.Lock()
		o.writes++
		o.mu.Unlock()
		if *want {
			if err := o.g.Open(ctx, nil); err != nil {
				return fmt.Errorf("open follower gripper: %w", err)
			}
		} else if _, err := o.g.Grab(ctx, nil); err != nil {
			return fmt.Errorf("close follower gripper: %w", err)
		}
		o.mu.Lock()
		o.open = want
		o.mu.Unlock()
	}
	return nil
}

// Reader reads a 0..100 percentage off a leader gripper with a configured DoCommand and a
// dotted key into its response.
type Reader struct {
	g       Gripper
	command map[string]interface{}
	key     string
}

// NewReader returns a Reader. command and key are both required.
func NewReader(g Gripper, command map[string]interface{}, key string) (*Reader, error) {
	if len(command) == 0 {
		return nil, fmt.Errorf("leader_gripper_read_command is required when leader_gripper is set")
	}
	if key == "" {
		return nil, fmt.Errorf("leader_gripper_read_key is required when leader_gripper is set")
	}
	return &Reader{g: g, command: command, key: key}, nil
}

// Read sends the command and extracts the key. A missing key names the keys that were
// returned, so a misconfigured path is diagnosed from the error alone.
func (r *Reader) Read(ctx context.Context) (float64, error) {
	resp, err := r.g.DoCommand(ctx, r.command)
	if err != nil {
		return 0, fmt.Errorf("read leader gripper: %w", err)
	}
	v, ok := lookup(resp, r.key)
	if !ok {
		return 0, fmt.Errorf("leader_gripper_read_key %q not in response (keys: %s)", r.key, keys(resp))
	}
	switch x := v.(type) {
	case float64:
		return x, nil
	case int:
		return float64(x), nil
	case int64:
		return float64(x), nil
	}
	return 0, fmt.Errorf("leader_gripper_read_key %q is %T, want a number", r.key, v)
}

func lookup(m map[string]interface{}, dotted string) (interface{}, bool) {
	parts := strings.Split(dotted, ".")
	var cur interface{} = m
	for _, p := range parts {
		mm, ok := cur.(map[string]interface{})
		if !ok {
			return nil, false
		}
		if cur, ok = mm[p]; !ok {
			return nil, false
		}
	}
	return cur, true
}

func keys(m map[string]interface{}) string {
	if len(m) == 0 {
		return "none"
	}
	out := make([]string, 0, len(m))
	for k := range m {
		out = append(out, k)
	}
	return strings.Join(out, ", ")
}
