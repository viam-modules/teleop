# teleop Module

The `viam:teleop` module drives a follower arm from a human input through the builtin motion service's teleop pipeline (the `teleop_start` / `teleop_move` / `teleop_stop` DoCommands), which plans IK and streams joints to the arm under the motion service's velocity and acceleration limits. It is pure Go (no cgo) and builds for linux/amd64, linux/arm64, darwin/arm64 and windows/amd64.

1. **`viam:teleop:clutch`** — Drags a follower arm with a hand between a `start` and a `stop`, like lifting a mouse to reposition it. The hand is a leader arm's end effector or a pose tracker body.
2. **`viam:teleop:leader-follower`** — Mirrors a hand-guided leader arm's end-effector pose onto a follower arm of different kinematics, continuously, from `start` to `stop`.

Both models:

- Run any number of **pairs** (one hand, one follower) in one service. Top-level attributes are defaults every pair inherits; each entry of `pairs` is a `name` plus the keys it overrides. Without `pairs`, the top level is one pair named `main`.
- Cap how fast the follower's commanded pose may move (`max_speed_mm_per_s`, `max_rotation_deg_per_s`), whatever the hand does and whatever `scale` is. The caps cannot be turned off.
- End the session when the teleop pipeline reports **any** error (an arm fault, an emergency stop, a planner failure) and refuse `start` while it does. A failing pipeline never fails the sends, because `teleop_move` only stores a pose, so the 200 ms status poll is the only place it shows.
- Need the follower, its gripper and the motion service in one frame system, with a `frame` block on the follower so `follower_component` has a pose in `world`.

---

## Model: `viam:teleop:clutch`

**API:** `rdk:service:generic`

`start` captures where the hand is and where the follower is; from then on the hand's motion is mirrored as scaled deltas onto the follower. `stop` leaves the follower where it is. Reposition the hand freely and `start` again.

The service does not read the clutch. Whatever owns it sends `start` and `stop`: a footswitch `actions` service mapping a pedal's press to `start` and its release to `stop`, a Vive grip, a button in an app.

The hand is one of:
- `leader_arm` → the arm's `EndPosition` (composed with `leader_tool_offset_mm`), in its base frame `<arm>_origin`.
- `pose_tracker` + `body` → the body's pose from `Poses`. A body missing from the map is "not tracked".

`start`, in order: refuses if the hand is not tracked or the pipeline reports an error; reads the follower's pose in `world`; asks the frame system once for the rotation of the hand's frame into `world` (a frame it does not know is taken to be `world`, with one warning); registers the follower with the pipeline **at its own pose**, so nothing moves; sends `start-capture` to the capture sensor if configured. Then, `rate_hz` times a second: read the hand, map it, filter it (outlier rejection, smoothing, deadzone), walk the commanded pose toward it no faster than the speed caps, send (at most one `teleop_move` in flight, latest wins), and mirror the leader gripper.

Mapping:

```
position    = follower_ref + scale × R_source × (hand − hand_ref)
orientation = relative:  R_source ∘ hand ∘ (hand_ref_world⁻¹ ∘ follower_ref)
              absolute:  R_source ∘ hand ∘ tool_orientation
              off:       follower_ref
```

`relative` turns the follower by however much the hand has turned since `start`, so nothing jumps; use it for a leader arm. `absolute` points the tool where the hand points; use it for a wand, with `tool_orientation` for the angle it is held at.

Leader arms are held (`leader_hold`) from configure until the service closes, never by `start` or `stop`, so the hand can reposition a limp leader between drags.

A session also ends on its own when the hand loses tracking, the pipeline reports an error, the pipeline drops the follower, or 10 consecutive reads or sends fail. The follower is released and `stopped_reason` says why; nothing moves until the next `start`.

### Configuration

```json
{
  "rotation_mode": "relative",
  "scale": 2.5,
  "leader_tool_offset_mm": { "x": 0, "y": 0, "z": 100 },
  "leader_hold": "manual",
  "leader_gripper_commands": { "on_start": [{ "command": "set_torque", "enable": false }] },
  "leader_gripper_read_command": { "command": "get_position" },
  "leader_gripper_read_key": "position_percentage",
  "follower_gripper_mode": "command",
  "follower_gripper_command": { "set": "$steps" },
  "pairs": [
    { "name": "right", "leader_arm": "right-leader-arm", "leader_gripper": "right-leader-gripper",
      "follower_component": "right-gripper", "follower_gripper": "right-gripper", "gripper_range": 850 },
    { "name": "left", "leader_arm": "left-leader-arm", "leader_gripper": "left-leader-gripper",
      "follower_component": "left-arm", "follower_gripper": "left-gripper", "gripper_range": 255, "gripper_invert": true }
  ]
}
```

A footswitch `clint:footswitch:actions` service as the clutch for the right pair:

```json
{
  "input_controller": "footswitch",
  "controls": {
    "pedal": {
      "on_press":   { "component": "teleop-clutch", "method": "do_command", "args": { "command": "start", "pair": "right" } },
      "on_release": { "component": "teleop-clutch", "method": "do_command", "args": { "command": "stop" } }
    }
  }
}
```

More in [`examples/clutch-pedal-so101.json`](examples/clutch-pedal-so101.json) and [`examples/clutch-vive.json`](examples/clutch-vive.json).

| Name                          | Type        | Required          | Description                                                                                                                                       |
| ----------------------------- | ----------- | ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `leader_arm`                  | string      | One source        | Leader arm whose `EndPosition` is the hand. Exclusive with `pose_tracker`.                                                                        |
| `pose_tracker`                | string      | One source        | Pose tracker that reports the hand. Needs `body`.                                                                                                 |
| `body`                        | string      | With tracker      | Body name in the tracker's `Poses` map.                                                                                                           |
| `follower_component`          | string      | Yes               | Frame-system name the pipeline positions, normally the follower gripper so its TCP tracks the hand.                                               |
| `motion_service`              | string      | No                | Motion service name. Defaults to `builtin`.                                                                                                       |
| `scale`                       | float       | No                | Translation scale, hand → follower. Defaults to `1`.                                                                                              |
| `rotation_mode`               | string      | No                | `absolute`, `relative` or `off`. Defaults to `absolute`.                                                                                          |
| `tool_orientation`            | orientation | No                | `absolute` mode: fixed rotation applied after the hand's, frame-config syntax. Defaults to identity.                                              |
| `leader_tool_offset_mm`       | vector      | No                | Arm source: the leader's physical tip in its `EndPosition` frame, so the tip (not the wrist) is mirrored. Defaults to zero.                       |
| `rate_hz`                     | float       | No                | Hand sampling rate. Defaults to `60` for an arm, `90` for a pose tracker.                                                                        |
| `smooth_tau_ms`               | float       | No                | EMA/SLERP time constant. `0` disables. Defaults to `200`.                                                                                         |
| `pos_deadzone_mm`             | float       | No                | Skip targets that move less than this. Defaults to `3`.                                                                                           |
| `rot_deadzone_deg`            | float       | No                | Skip targets that turn less than this. Defaults to `3`.                                                                                           |
| `max_speed_mm_per_s`          | float       | No                | Follower linear speed cap. Cannot be turned off. Defaults to `250`.                                                                               |
| `max_rotation_deg_per_s`      | float       | No                | Follower angular speed cap. Cannot be turned off. Defaults to `180`.                                                                              |
| `leader_hold`                 | string      | No                | Arm source: `none`, `manual` or `custom`, applied on configure and released on close. See [Leader hold](#leader-hold). Defaults to `none`.        |
| `manual_mode`                 | object      | No                | With `manual`: the `extra` map passed to `SetManualMode`.                                                                                         |
| `leader_commands`             | object      | No                | With `custom`: `before_start`, `after_start`, `on_stop` lists of leader DoCommand payloads.                                                       |
| `leader_gripper_commands`     | object      | No                | `on_start` and `on_stop` lists of leader gripper DoCommand payloads, sent with the hold.                                                           |
| `leader_gripper`              | string      | No                | Arm source: leader gripper to mirror. Needs `follower_gripper`, `leader_gripper_read_command` and `leader_gripper_read_key`.                       |
| `follower_gripper`            | string      | No                | Follower gripper. Needs `leader_gripper`.                                                                                                         |
| `leader_gripper_read_command` | object      | With gripper      | DoCommand sent to the leader gripper every tick.                                                                                                  |
| `leader_gripper_read_key`     | string      | With gripper      | Dotted path to the 0..100 opening in the response. Checked on `start`.                                                                            |
| `follower_gripper_mode`       | string      | No                | `open_close` (`Open` above 60 %, `Grab` below 40 %), `command` (send `follower_gripper_command`) or `off`. Defaults to `open_close`.             |
| `follower_gripper_command`    | object      | With `command`    | DoCommand template. `"$steps"` → `round(pct / 100 × gripper_range)`, `"$pct"` → the percentage.                                                  |
| `gripper_range`               | float       | No                | Value of `$steps` at 100 %. Defaults to `850`.                                                                                                    |
| `gripper_invert`              | bool        | No                | Flip the leader percentage. Defaults to `false`.                                                                                                  |
| `capture_control_sensor`      | string      | No                | Sensor sent `{"start-capture": true}` on `start` and `{"stop-capture": true}` on `stop`.                                                          |
| `pairs`                       | object[]    | No                | Pairs, each a `name` plus any keys above.                                                                                                         |

Fixed: outlier rejection at 4500 mm/s and 2700 deg/s (accepted after 5 in a row), a 500 ms send timeout, a 3 % gripper deadband.

### DoCommand

Every command takes an optional `"pair": "<name>"` (`"side"` is an alias); without it, it acts on every pair. Any other key is an error. Every command returns the status.

**`start`** — Engage: capture both poses and start mirroring. A `start` on a running pair does nothing, so it never re-captures mid-drag.

```json
{ "command": "start", "pair": "right" }
```

**`stop`** — Release the follower where it is. The leader stays held.

```json
{ "command": "stop" }
```

**`set_rotation_mode`** — Change the rotation mode, mid-session included.

```json
{ "command": "set_rotation_mode", "mode": "relative" }
```

**`status`** — Report every pair.

```json
{ "command": "status" }
```

Returns:

```json
{
  "running": true,
  "pairs": {
    "right": {
      "running": true,
      "session_id": "1793831554-6",
      "sessions": 6,
      "stopped_reason": "",
      "tracked": true,
      "leader_held": true,
      "sends": 160,
      "speed_limited": 41,
      "pipeline_error": "",
      "...": "..."
    }
  }
}
```

| Key                                                                    | Meaning                                                                                   |
| ---------------------------------------------------------------------- | ----------------------------------------------------------------------------------------- |
| `running`                                                              | The follower is being driven.                                                             |
| `session_id` / `sessions`                                              | The current or last session, and how many `start`s have engaged.                          |
| `stopped_reason`                                                       | `stopped`, `untracked`, `session_lost`, `pipeline_error` or `max_errors`.                 |
| `tracked`                                                              | The last hand read had a pose.                                                            |
| `leader_held` / `last_hold_error`                                      | The hold is applied; the last failure releasing it.                                       |
| `cycles`                                                               | Ticks completed without error.                                                            |
| `sends` / `dropped`                                                    | `teleop_move`s sent; targets dropped because one was in flight.                           |
| `speed_limited`                                                        | Ticks where a speed cap held the follower back.                                           |
| `deadzoned` / `rejected`                                               | Deadzone skips and outlier rejections.                                                    |
| `tick_overruns`                                                        | Ticks longer than the period.                                                             |
| `consecutive_errors` / `errors_total` / `last_error` / `last_error_at` | Cycle errors.                                                                             |
| `last_rpc_ms` / `source_read_ms` / `gripper_read_ms`                   | The last `teleop_move`, hand read and gripper read durations.                             |
| `rate_hz` / `rotation_mode`                                            | The configured rate and the current rotation mode.                                        |
| `gripper_pct` / `gripper_writes`                                       | The last percentage accepted and how many follower writes were issued.                    |
| `pipeline_error` / `pipeline_plan_ms`                                  | Copied from the motion service's `teleop_status` every 200 ms.                            |

---

## Model: `viam:teleop:leader-follower`

**API:** `rdk:service:generic`

Mirrors a hand-guided leader arm onto a follower arm of different kinematics, such as an SO-101 leader driving a uFactory xArm, from `start` until `stop`. There is no clutch: the mapping is absolute, which is what lets `start` drive the leader to where the follower is.

Mapping:

```
follower = origin ∘ scale(leader_end_position ∘ leader_tool_offset) ∘ tool_orientation
```

`start` has two modes:
- Default → **capture**: nothing moves. Both arms' current poses become the reference, the gap is absorbed as a constant offset, and the leader's motion is mirrored as scaled deltas from the follower's pose.
- `align_on_start: true` → **align**: the follower's pose is mapped back into the leader's frame, solved with a best-effort IK (a 5-DoF leader cannot reach every 6-DoF pose), and the leader is driven there with a blocking move. The leftover gap is absorbed, so the first target sent is exactly where the follower already is. A target outside the leader's reach is the usual cause of a large gap: pick `origin_translation_mm` so the follower's start pose maps into the leader's comfortable workspace (`origin = follower_start − scale × leader_tip`).

Either way the follower is registered with the pipeline at its own pose, and the leader is held (`leader_hold`) until `stop`. A loop that stops itself on errors leaves the leader held (a hand may be on it); `stop` releases it. On a pipeline error the follower is also removed from the pipeline at once, so an arm whose driver clears its own faults cannot resume toward a stale target.

Arm methods used on the leader: `EndPosition` and `JointPositions` always; `Kinematics` and `MoveToJointPositions` with `align_on_start`; `Properties` and `SetManualMode` with `leader_hold: manual`; `DoCommand` with `leader_hold: custom`.

### Configuration

```json
{
  "scale": 2.5,
  "leader_tool_offset_mm": { "x": 0, "y": 0, "z": 100 },
  "tool_orientation": { "type": "ov_degrees", "value": { "x": 0, "y": 0, "z": 1, "th": -90 } },
  "align_on_start": false,
  "leader_hold": "manual",
  "leader_gripper_commands": { "on_start": [{ "command": "set_torque", "enable": false }] },
  "leader_gripper_read_command": { "command": "get_position" },
  "leader_gripper_read_key": "position_percentage",
  "follower_gripper_mode": "command",
  "follower_gripper_command": { "set": "$steps" },
  "pairs": [
    { "name": "right", "leader_arm": "right-leader-arm", "leader_gripper": "right-leader-gripper",
      "follower_component": "right-gripper", "follower_gripper": "right-gripper" },
    { "name": "left", "leader_arm": "left-leader-arm", "leader_gripper": "left-leader-gripper",
      "follower_component": "left-arm", "follower_gripper": "left-gripper",
      "origin_translation_mm": { "x": 0, "y": 1000, "z": 105 }, "gripper_range": 255, "gripper_invert": true }
  ]
}
```

More in [`examples/so101-to-xarm.json`](examples/so101-to-xarm.json).

| Name                          | Type        | Required       | Description                                                                                                                              |
| ----------------------------- | ----------- | -------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `leader_arm`                  | string      | Yes            | Leader arm to read.                                                                                                                      |
| `follower_component`          | string      | Yes            | Frame-system name the pipeline positions, normally the follower gripper.                                                                |
| `motion_service`              | string      | No             | Motion service name. Defaults to `builtin`.                                                                                              |
| `scale`                       | float       | No             | Translation scale, leader → follower. Defaults to `1`.                                                                                   |
| `origin_translation_mm`       | vector      | No             | Leader base position in the follower's world. Defaults to zero.                                                                          |
| `origin_orientation`          | orientation | No             | Leader base orientation in the follower's world, frame-config syntax. Defaults to identity.                                             |
| `leader_tool_offset_mm`       | vector      | No             | Leader tip in its `EndPosition` frame, applied before `scale`. Defaults to zero.                                                        |
| `tool_orientation`            | orientation | No             | Fixed tool-frame rotation reconciling the two arms' tool conventions. Defaults to identity.                                             |
| `rotation_enabled`            | bool        | No             | Mirror orientation as well as position. Defaults to `true`.                                                                              |
| `rate_hz`                     | float       | No             | Leader sampling rate. Defaults to `60`.                                                                                                  |
| `smooth_tau_ms`               | float       | No             | EMA/SLERP time constant. `0` disables. Defaults to `100`.                                                                                |
| `pos_deadzone_mm`             | float       | No             | Skip targets that move less than this. Defaults to `1`.                                                                                  |
| `rot_deadzone_deg`            | float       | No             | Skip targets that turn less than this. Defaults to `1`.                                                                                  |
| `max_speed_mm_per_s`          | float       | No             | Follower linear speed cap. Cannot be turned off. Defaults to `250`.                                                                      |
| `max_rotation_deg_per_s`      | float       | No             | Follower angular speed cap. Cannot be turned off. Defaults to `180`.                                                                     |
| `align_on_start`              | bool        | No             | Drive the leader to the follower's pose on `start`. Defaults to `false`.                                                                 |
| `align_max_position_error_mm` | float       | No             | With `align_on_start`, refuse to start if the leader lands further than this. `0` is best effort. Defaults to `0`.                       |
| `leader_hold`                 | string      | No             | `none`, `manual` or `custom`. See [Leader hold](#leader-hold). Defaults to `none`.                                                       |
| `manual_mode`                 | object      | No             | With `manual`: the `extra` map passed to `SetManualMode`.                                                                                |
| `leader_commands`             | object      | No             | With `custom`: `before_start`, `after_start`, `on_stop` lists of leader DoCommand payloads.                                              |
| `leader_gripper_commands`     | object      | No             | `on_start` and `on_stop` lists of leader gripper DoCommand payloads. An SO-101 needs `set_torque` off on start to free its trigger.      |
| `leader_gripper`              | string      | No             | Leader gripper to mirror. Set with `follower_gripper`.                                                                                   |
| `follower_gripper`            | string      | No             | Follower gripper. Set with `leader_gripper`.                                                                                             |
| `leader_gripper_read_command` | object      | With gripper   | DoCommand sent to the leader gripper every tick.                                                                                         |
| `leader_gripper_read_key`     | string      | With gripper   | Dotted path to the 0..100 opening in the response. Checked on `start`.                                                                   |
| `follower_gripper_mode`       | string      | No             | `open_close`, `command` or `off`. `xarm` is accepted as an alias of `command`. Defaults to `open_close`.                                  |
| `follower_gripper_command`    | object      | With `command` | DoCommand template with `"$steps"` and `"$pct"` tokens.                                                                                  |
| `gripper_range`               | float       | No             | Value of `$steps` at 100 %. Defaults to `850`.                                                                                           |
| `gripper_invert`              | bool        | No             | Flip the leader percentage. Defaults to `false`.                                                                                         |
| `auto_start`                  | bool        | No             | Start every pair on configure. A pair that fails fails the configure. Defaults to `false`.                                               |
| `pairs`                       | object[]    | No             | Pairs, each a `name` plus any keys above.                                                                                                |

Fixed: outlier rejection at 4500 mm/s and 1800 deg/s, a 500 ms send timeout, 10 consecutive cycle failures before the loop stops, a 3 % gripper deadband, align residual compensation. Keys removed since `devrel:so101:teleop-cartesian` (`blocked_feedback`, `gripper_lock`, `gripper_in_*_pct`, `outlier_*`, …) are rejected by name with the replacement in the error.

### Leader hold

Shared by both models.

| Value    | What is sent                                                                                                                                                       | When                                                                   |
| -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------- |
| `none`   | Nothing.                                                                                                                                                           | A leader that is already compliant, or one you hold yourself.          |
| `manual` | `SetManualMode(false)` first, `SetManualMode(true, manual_mode)` to hold, `SetManualMode(false)` to release. Refused if `Properties` lacks `SupportManualMode`.     | Any arm module with manual mode (SO-101 since v0.13).                  |
| `custom` | `leader_commands` lists at the same points. `before_start` failures are ignored; an `after_start` failure aborts and sends `on_stop`; `on_stop` failures are warned. | A leader whose module has no manual-mode API.                          |

### DoCommand

Every command takes an optional `"pair": "<name>"` (`"side"` is an alias). Any other key is an error. `start` and `stop` attempt every pair even if one fails. Every command returns the status.

**`start`** — Capture or align, hold the leader, register the follower, mirror.

```json
{ "command": "start", "pair": "left" }
```

**`stop`** — Remove the follower from the pipeline and release the leader.

```json
{ "command": "stop" }
```

**`status`** — Report every pair.

```json
{ "command": "status" }
```

Returns the same shape as the clutch, with `leader_read_ms` in place of `source_read_ms`, no `tracked` or `rotation_mode`, and the alignment:

| Key                                                         | Meaning                                                              |
| ----------------------------------------------------------- | -------------------------------------------------------------------- |
| `stopped_reason`                                            | `stopped`, `session_lost`, `pipeline_error` or `max_errors`.         |
| `align_position_error_mm` / `align_orientation_error_deg`   | The gap between the mapped leader and the follower after `start`.    |
| `align_poses`                                               | Follower pose, leader target, leader actual, joints and residual.    |

### Client

`cmd/teleop-client` runs the leader-follower engine against live machines over gRPC with the same attribute block, for trying a mapping from a laptop. `-align-only` aligns, prints the residual and exits.

```sh
VIAM_API_KEY_ID=... VIAM_API_KEY=... \
go run ./cmd/teleop-client -config examples/so101-to-xarm.json -leader-addr <machine>.viam.cloud -align-only
```

---

## Development

```
cmd/module/                module entrypoint
cmd/teleop-client/         the leader-follower engine from a laptop
services/clutch/           viam:teleop:clutch: attributes, pairs, DoCommand
services/leaderfollower/   viam:teleop:leader-follower: attributes, pairs, DoCommand
internal/clutch/           clutch engine: sources (pose tracker, arm), start/stop, delta mapping
internal/leaderfollower/   leader-follower engine, absolute mapping, best-effort IK, pose log
internal/hold/             leader hold (none / arm manual mode / custom DoCommands)
internal/pipeline/         motion service teleop pipeline client + latest-wins sender
internal/filter/           smoothing, outlier rejection, deadzone, speed caps
internal/gripper/          follower gripper output (async, latest-wins) and leader gripper reader
internal/pairs/            "top-level defaults + pairs list" config resolution
internal/posemath/         orientation error, pose maps
internal/testfake/         test fakes: arm with real kinematics, motion service, gripper, pose tracker, frame system
```

`internal/*` imports no `services/*`. Production code imports nothing that needs cgo; `GOOS=windows GOARCH=amd64 go build -tags no_cgo ./...` is the check. The Go module path is `teleop` (no host), so it is a binary, not a library.

```sh
go test ./...
make module.tar.gz
```
