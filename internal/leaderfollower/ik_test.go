package leaderfollower

import (
	"context"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"teleop/internal/testfake"
)

var zero5 = []referenceframe.Input{0, 0, 0, 0, 0}

func TestSolveBestEffortReachesAReachablePose(t *testing.T) {
	model := testfake.Model(t)
	// Forward kinematics of a real configuration is reachable by construction.
	truth := []referenceframe.Input{0.3, -0.6, 0.9, 0.4, -0.2}
	goal, err := model.Transform(truth)
	test.That(t, err, test.ShouldBeNil)

	res, err := SolveBestEffort(context.Background(), model, zero5, goal, IKOptions{})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res.PositionErrorMM, test.ShouldBeLessThan, 0.5)
	test.That(t, res.OrientationErrorDeg, test.ShouldBeLessThan, 0.5)
	test.That(t, len(res.Inputs), test.ShouldEqual, 5)
	for i, l := range model.DoF() {
		test.That(t, res.Inputs[i], test.ShouldBeBetweenOrEqual, l.Min, l.Max)
	}
}

func TestSolveBestEffortReportsTheResidualOfAnUnreachableOrientation(t *testing.T) {
	model := testfake.Model(t)
	truth := []referenceframe.Input{0.2, -0.5, 0.8, 0.3, 0}
	reachable, err := model.Transform(truth)
	test.That(t, err, test.ShouldBeNil)

	// Same position, but yaw the tool about world Z with the tip held fixed. On an arm whose
	// base rotates about Z and whose remaining joints pitch in the arm's vertical plane, the
	// approach direction must lie in that plane, so this orientation is unreachable at this
	// position. The solver must keep the position and report the orientation residual
	// instead of failing.
	yawed := spatialmath.Compose(
		spatialmath.NewPoseFromOrientation(&spatialmath.EulerAngles{Yaw: 0.6}),
		spatialmath.NewPoseFromOrientation(reachable.Orientation()),
	).Orientation()
	twisted := spatialmath.NewPose(reachable.Point(), yawed)
	res, err := SolveBestEffort(context.Background(), model, zero5, twisted, IKOptions{})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, res.PositionErrorMM, test.ShouldBeLessThan, 25)
	test.That(t, res.OrientationErrorDeg, test.ShouldBeGreaterThan, 1) // genuinely unreachable
	test.That(t, res.OrientationErrorDeg, test.ShouldBeLessThan, 90)
}

func TestSolveBestEffortNeverFailsForAnOutOfReachPosition(t *testing.T) {
	model := testfake.Model(t)
	farAway := spatialmath.NewPose(r3.Vector{X: 2000, Y: 0, Z: 300}, spatialmath.NewZeroOrientation())
	res, err := SolveBestEffort(context.Background(), model, zero5, farAway, IKOptions{})
	test.That(t, err, test.ShouldBeNil)
	// It stretched toward the goal: the residual is far smaller than the 2 m request but
	// the arm is only ~0.25 m long, so it cannot be zero.
	test.That(t, res.PositionErrorMM, test.ShouldBeGreaterThan, 1500)
	test.That(t, res.PositionErrorMM, test.ShouldBeLessThan, 1950)
	test.That(t, math.IsNaN(res.Cost), test.ShouldBeFalse)
}

func TestSolveBestEffortIsReproducibleAndRejectsAWrongSeed(t *testing.T) {
	model := testfake.Model(t)
	goal := spatialmath.NewPose(r3.Vector{X: 120, Y: 60, Z: 150}, &spatialmath.OrientationVectorDegrees{OX: 0, OY: 0, OZ: -1, Theta: 0})
	a, err := SolveBestEffort(context.Background(), model, zero5, goal, IKOptions{})
	test.That(t, err, test.ShouldBeNil)
	b, err := SolveBestEffort(context.Background(), model, zero5, goal, IKOptions{})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, a.Inputs, test.ShouldResemble, b.Inputs)

	_, err = SolveBestEffort(context.Background(), model, []referenceframe.Input{0}, spatialmath.NewZeroPose(), IKOptions{})
	test.That(t, err, test.ShouldNotBeNil)
}
