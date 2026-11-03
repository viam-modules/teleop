package pairs

import (
	"errors"
	"testing"

	"go.viam.com/test"
)

type settings struct {
	Arm   string  `json:"arm"`
	Scale float64 `json:"scale,omitempty"`
	Flag  *bool   `json:"flag,omitempty"`
}

func validate(s settings) error {
	if s.Arm == "" {
		return errors.New("arm is required")
	}
	return nil
}

func TestNoPairsListIsOnePairNamedMain(t *testing.T) {
	out, err := Resolve(settings{Arm: "a", Scale: 2}, nil, validate)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out, test.ShouldHaveLength, 1)
	test.That(t, out[0].Name, test.ShouldEqual, DefaultName)
	test.That(t, out[0].Settings.Scale, test.ShouldEqual, 2)

	_, err = Resolve(settings{}, nil, validate)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestPairsInheritAbsentKeysAndOverridePresentOnes(t *testing.T) {
	f := false
	out, err := Resolve(settings{Arm: "default", Scale: 2, Flag: &f}, []map[string]interface{}{
		{"name": "left", "arm": "left-arm"},
		{"name": "right", "arm": "right-arm", "scale": 3, "flag": true},
	}, validate)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out, test.ShouldHaveLength, 2)
	test.That(t, out[0].Name, test.ShouldEqual, "left") // order preserved
	test.That(t, out[0].Settings.Arm, test.ShouldEqual, "left-arm")
	test.That(t, out[0].Settings.Scale, test.ShouldEqual, 2) // inherited
	test.That(t, *out[0].Settings.Flag, test.ShouldBeFalse)
	test.That(t, out[1].Settings.Scale, test.ShouldEqual, 3) // overridden
	test.That(t, *out[1].Settings.Flag, test.ShouldBeTrue)
}

func TestPairErrorsNameThePair(t *testing.T) {
	_, err := Resolve(settings{Arm: "a"}, []map[string]interface{}{{"arm": "x"}}, validate)
	test.That(t, err.Error(), test.ShouldContainSubstring, "pairs[0]: name is required")

	_, err = Resolve(settings{Arm: "a"}, []map[string]interface{}{{"name": "l"}, {"name": "l"}}, validate)
	test.That(t, err.Error(), test.ShouldContainSubstring, `pairs[1]: duplicate name "l"`)

	_, err = Resolve(settings{}, []map[string]interface{}{{"name": "l"}}, validate)
	test.That(t, err.Error(), test.ShouldContainSubstring, "pairs[0] (l): arm is required")

	_, err = Resolve(settings{Arm: "a"}, []map[string]interface{}{{"name": "l", "scale": "wide"}}, validate)
	test.That(t, err.Error(), test.ShouldContainSubstring, "pairs[0] (l)")
}
