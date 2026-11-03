package gripper

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	"go.viam.com/test"
)

type fake struct {
	mu     sync.Mutex
	cmds   []map[string]interface{}
	opens  []bool
	resp   map[string]interface{}
	err    error
	block  chan struct{}
	blocks int
}

func (f *fake) Open(context.Context, map[string]interface{}) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.opens = append(f.opens, true)
	return nil
}

func (f *fake) Grab(context.Context, map[string]interface{}) (bool, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.opens = append(f.opens, false)
	return true, nil
}

func (f *fake) DoCommand(_ context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	f.mu.Lock()
	f.cmds = append(f.cmds, cmd)
	block, err, resp := f.block, f.err, f.resp
	f.mu.Unlock()
	if block != nil {
		<-block
	}
	return resp, err
}

func (f *fake) count() int {
	f.mu.Lock()
	defer f.mu.Unlock()
	return len(f.cmds)
}

func (f *fake) last() map[string]interface{} {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.cmds[len(f.cmds)-1]
}

func TestTemplateTokensAreSubstitutedAtAnyDepth(t *testing.T) {
	tpl := map[string]interface{}{
		"command": "move",
		"args":    map[string]interface{}{"position": TokenSteps, "speed": 50.0, "pct": []interface{}{TokenPct}},
	}
	out := Substitute(tpl, 50, 850)
	args := out["args"].(map[string]interface{})
	test.That(t, args["position"], test.ShouldEqual, 425.0)
	test.That(t, args["speed"], test.ShouldEqual, 50.0)
	test.That(t, args["pct"].([]interface{})[0], test.ShouldEqual, 50.0)
	test.That(t, out["command"], test.ShouldEqual, "move")
	// The template itself is untouched.
	test.That(t, tpl["args"].(map[string]interface{})["position"], test.ShouldEqual, TokenSteps)

	test.That(t, ValidateTemplate(tpl), test.ShouldBeNil)
	test.That(t, ValidateTemplate(map[string]interface{}{"set": 1.0}), test.ShouldNotBeNil)
	test.That(t, ValidateTemplate(nil), test.ShouldNotBeNil)

	m, aliased := NormalizeMode("xarm")
	test.That(t, m, test.ShouldEqual, ModeCommand)
	test.That(t, aliased, test.ShouldBeTrue)
}

func TestOutputWritesOnChangeOnlyWithADeadband(t *testing.T) {
	g := &fake{}
	o := NewOutput(g, OutputConfig{Mode: ModeCommand, Template: map[string]interface{}{"set": TokenSteps}, Range: 850, DeadbandPct: 3})
	test.That(t, o.Set(50), test.ShouldBeTrue)
	o.Wait()
	test.That(t, g.last()["set"], test.ShouldEqual, 425.0)
	test.That(t, o.Set(50), test.ShouldBeFalse)
	test.That(t, o.Set(52), test.ShouldBeFalse) // inside the deadband
	test.That(t, o.Set(60), test.ShouldBeTrue)
	o.Wait()
	test.That(t, g.count(), test.ShouldEqual, 2)
	test.That(t, g.last()["set"], test.ShouldEqual, 510.0)
	test.That(t, o.Pct(), test.ShouldEqual, 60)
	test.That(t, o.Writes(), test.ShouldEqual, 2)

	// Invert flips before the deadband; values are clamped to 0..100.
	inv := NewOutput(g, OutputConfig{Mode: ModeCommand, Template: map[string]interface{}{"set": TokenSteps}, Invert: true})
	inv.Set(130)
	inv.Wait()
	test.That(t, g.last()["set"], test.ShouldEqual, 0.0)
	test.That(t, inv.Pct(), test.ShouldEqual, 0)
}

func TestABlockingWriteDoesNotBlockSetAndTheLatestValueWins(t *testing.T) {
	g := &fake{block: make(chan struct{})}
	o := NewOutput(g, OutputConfig{Mode: ModeCommand, Template: map[string]interface{}{"set": TokenPct}})
	done := make(chan struct{})
	go func() {
		o.Set(10)
		o.Set(20)
		o.Set(30)
		o.Set(40)
		close(done)
	}()
	select {
	case <-done:
	case <-time.After(time.Second):
		t.Fatal("Set blocked behind a slow write")
	}
	deadline := time.Now().Add(time.Second)
	for g.count() < 1 && time.Now().Before(deadline) {
		time.Sleep(time.Millisecond)
	}
	test.That(t, g.count(), test.ShouldEqual, 1) // only the first is on the wire
	close(g.block)
	o.Wait()
	test.That(t, g.count(), test.ShouldEqual, 2) // then only the latest pending one
	test.That(t, g.last()["set"], test.ShouldEqual, 40.0)
}

func TestOpenCloseModeHasHysteresis(t *testing.T) {
	g := &fake{}
	o := NewOutput(g, OutputConfig{Mode: ModeOpenClose})
	for _, pct := range []float64{80, 55, 45, 30, 50, 61} {
		o.Set(pct)
		o.Wait()
	}
	test.That(t, g.opens, test.ShouldResemble, []bool{true, false, true})
	test.That(t, g.count(), test.ShouldEqual, 0) // no DoCommand in this mode

	off := NewOutput(g, OutputConfig{Mode: ModeOff})
	test.That(t, off.Enabled(), test.ShouldBeFalse)
	test.That(t, off.Set(50), test.ShouldBeFalse)
	test.That(t, NewOutput(nil, OutputConfig{Mode: ModeOpenClose}).Enabled(), test.ShouldBeFalse)
}

func TestOutputErrorsAreTakenOnce(t *testing.T) {
	g := &fake{err: errors.New("jaw stuck")}
	o := NewOutput(g, OutputConfig{Mode: ModeCommand, Template: map[string]interface{}{"set": TokenPct}})
	o.Set(10)
	o.Wait()
	test.That(t, o.TakeErr(), test.ShouldNotBeNil)
	test.That(t, o.TakeErr(), test.ShouldBeNil)
}

func TestReaderFollowsADottedKeyAndNamesTheKeysOnAMiss(t *testing.T) {
	ctx := context.Background()
	g := &fake{resp: map[string]interface{}{"state": map[string]interface{}{"opening_pct": 42.0}, "ok": true}}
	r, err := NewReader(g, map[string]interface{}{"command": "get_position"}, "state.opening_pct")
	test.That(t, err, test.ShouldBeNil)
	v, err := r.Read(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, v, test.ShouldEqual, 42)
	test.That(t, g.last()["command"], test.ShouldEqual, "get_position")

	wrong, _ := NewReader(g, map[string]interface{}{"command": "get_position"}, "position_percentage")
	_, err = wrong.Read(ctx)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, `leader_gripper_read_key "position_percentage" not in response`)
	test.That(t, err.Error(), test.ShouldContainSubstring, "state")

	notNum, _ := NewReader(g, map[string]interface{}{"command": "get_position"}, "ok")
	_, err = notNum.Read(ctx)
	test.That(t, err.Error(), test.ShouldContainSubstring, "want a number")

	_, err = NewReader(g, nil, "x")
	test.That(t, err, test.ShouldNotBeNil)
	_, err = NewReader(g, map[string]interface{}{"command": "x"}, "")
	test.That(t, err, test.ShouldNotBeNil)
}
