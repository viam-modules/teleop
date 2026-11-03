// Package filter conditions a stream of sampled target poses before they are sent to a
// follower: outlier rejection, time-normalised smoothing, and a deadzone against the last
// pose that passed. Every rate is per unit time rather than per frame, so the feel does not
// change with the sampling rate.
package filter

import (
	"math"
	"time"

	"go.viam.com/rdk/spatialmath"

	"teleop/internal/posemath"
)

// Config tunes a Filter. Zero values disable the corresponding stage.
type Config struct {
	// SmoothTau is the EMA time constant for position and the SLERP time constant for
	// orientation. Zero disables smoothing.
	SmoothTau time.Duration
	// OutlierPosMMPerSec / OutlierRotDegPerSec reject a sample that would imply a faster
	// motion than this. Zero disables that check.
	OutlierPosMMPerSec  float64
	OutlierRotDegPerSec float64
	// OutlierAcceptAfter is how many consecutive rejections before the jump is accepted as
	// real and the filter re-seeded. Default 5.
	OutlierAcceptAfter int
	// DeadzonePosMM / DeadzoneRotDeg skip a pose that differs from the last passed pose by
	// less than this. Zero disables the deadzone on that axis.
	DeadzonePosMM  float64
	DeadzoneRotDeg float64
}

// Filter is the per-session state behind Config. The first Step seeds it and always passes.
//
// Deadzone semantics: lastPassed advances when the deadzone passes, before the caller decides
// whether it can actually send (a latest-wins sender may drop the pose). vive advances its
// reference only on an actual send. Phase 0 keeps the so-101 behaviour; the clutch service
// should pick deliberately.
type Filter struct {
	cfg        Config
	prev       spatialmath.Pose
	prevTime   time.Time
	lastPassed spatialmath.Pose
	streak     int
	rejected   uint64
	deadzoned  uint64
}

// New returns a Filter for cfg.
func New(cfg Config) *Filter {
	if cfg.OutlierAcceptAfter <= 0 {
		cfg.OutlierAcceptAfter = 5
	}
	return &Filter{cfg: cfg}
}

// Reset forgets the session state so the next Step seeds again. Counters are kept.
func (f *Filter) Reset() {
	f.prev, f.lastPassed, f.streak = nil, nil, 0
	f.prevTime = time.Time{}
}

// Counters are the filter's per-session statistics.
type Counters struct {
	Rejected  uint64 // outlier rejections
	Deadzoned uint64 // deadzone skips
}

// Counters returns the statistics so far.
func (f *Filter) Counters() Counters {
	return Counters{Rejected: f.rejected, Deadzoned: f.deadzoned}
}

// Step conditions raw, sampled at now. It returns the pose to send and whether to send it.
func (f *Filter) Step(now time.Time, raw spatialmath.Pose) (spatialmath.Pose, bool) {
	if f.prev == nil {
		f.prev, f.prevTime, f.lastPassed = raw, now, raw
		return raw, true
	}
	dt := now.Sub(f.prevTime).Seconds()
	// Clamp like the mobile app does: a stall must not become an infinite speed, a burst must
	// not become an instant one.
	dt = math.Min(math.Max(dt, 1.0/120), 0.25)

	if f.cfg.OutlierPosMMPerSec > 0 || f.cfg.OutlierRotDegPerSec > 0 {
		posRate := raw.Point().Sub(f.prev.Point()).Norm() / dt
		rotRate := posemath.AngleBetweenDeg(f.prev, raw) / dt
		if (f.cfg.OutlierPosMMPerSec > 0 && posRate > f.cfg.OutlierPosMMPerSec) ||
			(f.cfg.OutlierRotDegPerSec > 0 && rotRate > f.cfg.OutlierRotDegPerSec) {
			f.streak++
			if f.streak < f.cfg.OutlierAcceptAfter {
				f.rejected++
				return nil, false
			}
			// Accepted as real: re-seed so the EMA does not drag it out.
			f.prev, f.prevTime, f.lastPassed, f.streak = raw, now, raw, 0
			return raw, true
		}
	}
	f.streak = 0

	target := raw
	if f.cfg.SmoothTau > 0 {
		alpha := 1 - math.Exp(-dt/f.cfg.SmoothTau.Seconds())
		target = spatialmath.Interpolate(f.prev, raw, alpha)
	}
	f.prev, f.prevTime = target, now

	if f.cfg.DeadzonePosMM > 0 || f.cfg.DeadzoneRotDeg > 0 {
		dPos := target.Point().Sub(f.lastPassed.Point()).Norm()
		dRot := posemath.AngleBetweenDeg(f.lastPassed, target)
		// A zero threshold means "ignore that axis", not "never inside".
		inPos := f.cfg.DeadzonePosMM == 0 || dPos < f.cfg.DeadzonePosMM
		inRot := f.cfg.DeadzoneRotDeg == 0 || dRot < f.cfg.DeadzoneRotDeg
		if inPos && inRot {
			f.deadzoned++
			return nil, false
		}
	}
	f.lastPassed = target
	return target, true
}
