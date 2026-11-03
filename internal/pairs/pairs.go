// Package pairs resolves the "top-level defaults plus a pairs list" configuration shape both
// teleop services use: the top-level attributes are the defaults every pair inherits, and
// each entry of "pairs" is a name plus any keys it overrides. With no pairs list, the
// defaults are the one pair, named "main".
//
// The overlay works at the JSON level so that "set in the pair" and "set in the defaults"
// mean exactly what they mean in a config file: a key present in the pair wins, an absent
// key inherits. Pair order is preserved.
package pairs

import (
	"encoding/json"
	"fmt"
)

// DefaultName names the single pair of a config that has no "pairs" list.
const DefaultName = "main"

// Pair is one resolved entry.
type Pair[T any] struct {
	Name     string
	Settings T
}

// Resolve overlays each entry of overrides on defaults. validate, if non-nil, is run on each
// resolved pair and its error is prefixed with the pair's position and name.
func Resolve[T any](defaults T, overrides []map[string]interface{}, validate func(T) error) ([]Pair[T], error) {
	if len(overrides) == 0 {
		if validate != nil {
			if err := validate(defaults); err != nil {
				return nil, err
			}
		}
		return []Pair[T]{{Name: DefaultName, Settings: defaults}}, nil
	}
	base, err := json.Marshal(defaults)
	if err != nil {
		return nil, err
	}
	var baseMap map[string]interface{}
	if err := json.Unmarshal(base, &baseMap); err != nil {
		return nil, err
	}

	out := make([]Pair[T], 0, len(overrides))
	seen := map[string]bool{}
	for i, o := range overrides {
		name, _ := o["name"].(string)
		if name == "" {
			return nil, fmt.Errorf("pairs[%d]: name is required", i)
		}
		if seen[name] {
			return nil, fmt.Errorf("pairs[%d]: duplicate name %q", i, name)
		}
		seen[name] = true

		merged := make(map[string]interface{}, len(baseMap)+len(o))
		for k, v := range baseMap {
			merged[k] = v
		}
		for k, v := range o {
			if k != "name" {
				merged[k] = v
			}
		}
		raw, err := json.Marshal(merged)
		if err != nil {
			return nil, fmt.Errorf("pairs[%d] (%s): %w", i, name, err)
		}
		var s T
		if err := json.Unmarshal(raw, &s); err != nil {
			return nil, fmt.Errorf("pairs[%d] (%s): %w", i, name, err)
		}
		if validate != nil {
			if err := validate(s); err != nil {
				return nil, fmt.Errorf("pairs[%d] (%s): %w", i, name, err)
			}
		}
		out = append(out, Pair[T]{Name: name, Settings: s})
	}
	return out, nil
}
