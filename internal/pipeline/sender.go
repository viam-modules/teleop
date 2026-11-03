package pipeline

import (
	"context"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/spatialmath"
)

// Sender sends target poses to a Pipeline with at most one Move in flight. A pose offered
// while one is in flight is dropped, not queued: the pipeline keeps only the latest target,
// and the next tick has a fresher one anyway. This is the sender the vive module, the mobile
// app and the so-101 engine each carried a copy of.
type Sender struct {
	p       *Pipeline
	timeout time.Duration

	inFlight    atomic.Bool
	sessionLost atomic.Bool
	lastRPC     atomic.Int64 // nanoseconds
	lastSent    atomic.Pointer[spatialmath.Pose]

	mu      sync.Mutex
	sends   uint64
	dropped uint64
	err     error // the most recent failed Move, taken by the next TakeErr
	wg      sync.WaitGroup
}

// NewSender returns a Sender for p. timeout bounds each Move RPC.
func NewSender(p *Pipeline, timeout time.Duration) *Sender {
	if timeout <= 0 {
		timeout = 500 * time.Millisecond
	}
	return &Sender{p: p, timeout: timeout}
}

// Reset clears the session state and counters for a new session.
func (s *Sender) Reset() {
	s.Wait()
	s.mu.Lock()
	s.sends, s.dropped, s.err = 0, 0, nil
	s.mu.Unlock()
	s.sessionLost.Store(false)
	s.inFlight.Store(false)
	s.lastRPC.Store(0)
	s.lastSent.Store(nil)
}

// TrySend sends target unless a Move is already in flight. It returns whether the send was
// started; false counts as a drop.
func (s *Sender) TrySend(target spatialmath.Pose) bool {
	if s.sessionLost.Load() {
		return false
	}
	if !s.inFlight.CompareAndSwap(false, true) {
		s.mu.Lock()
		s.dropped++
		s.mu.Unlock()
		return false
	}
	s.mu.Lock()
	s.sends++
	s.mu.Unlock()
	s.lastSent.Store(&target)
	s.wg.Add(1)
	go s.send(target)
	return true
}

func (s *Sender) send(target spatialmath.Pose) {
	defer s.wg.Done()
	defer s.inFlight.Store(false)
	ctx, cancel := context.WithTimeout(context.Background(), s.timeout)
	defer cancel()
	start := time.Now()
	err := s.p.Move(ctx, target)
	s.lastRPC.Store(time.Since(start).Nanoseconds())
	if err != nil {
		if IsSessionLost(err) {
			s.sessionLost.Store(true)
		}
		s.mu.Lock()
		s.err = err
		s.mu.Unlock()
	}
}

// Wait blocks until no Move is in flight.
func (s *Sender) Wait() { s.wg.Wait() }

// TakeErr returns and clears the most recent send error.
func (s *Sender) TakeErr() error {
	s.mu.Lock()
	defer s.mu.Unlock()
	err := s.err
	s.err = nil
	return err
}

// Lost reports whether a send found the server-side session gone.
func (s *Sender) Lost() bool { return s.sessionLost.Load() }

// LastSent is the last pose handed to the pipeline, or nil.
func (s *Sender) LastSent() spatialmath.Pose {
	if p := s.lastSent.Load(); p != nil {
		return *p
	}
	return nil
}

// Counters is a snapshot of the sender's statistics.
type Counters struct {
	Sends     uint64
	Dropped   uint64 // a send was due but one was already in flight
	LastRPCMs float64
}

// Snapshot returns the counters so far.
func (s *Sender) Snapshot() Counters {
	s.mu.Lock()
	defer s.mu.Unlock()
	return Counters{Sends: s.sends, Dropped: s.dropped, LastRPCMs: float64(s.lastRPC.Load()) / 1e6}
}
