[← Part of the teleop module](../README.md)

# Model viam:teleop:leader-follower

A generic service that mirrors a hand-guided **leader** arm's end-effector pose onto a
**follower** arm of different kinematics, such as an SO-101 leader driving a uFactory xArm.
It works in Cartesian space: the leader's tool pose is mapped into the follower's world and
handed to viam-server's builtin motion service **teleop pipeline** (the `teleop_start` /
`teleop_move` / `teleop_stop` DoCommands), which plans IK and streams joints to the follower
under its own velocity and acceleration limits.

The leader is read through the standard arm API. Nothing here is specific to what either arm
is: the two places where arm modules still differ (how a leader is made compliant, how a
gripper's position is read and written) are configuration, not code. See
[Leader hold](#leader-hold) and [Gripper mirroring](#gripper-mirroring).

The same code runs as a standalone client, `cmd/teleop-client`, for trying a mapping from a
laptop before putting it in a machine config.

## What the leader must implement

| Mode | Arm methods used |
| --- | --- |
| always | `EndPosition`, `JointPositions` |
| `align_on_start` | `Kinematics`, `MoveToJointPositions` (blocking) |
| `leader_hold: manual` | `Properties` (must report `SupportManualMode`), `SetManualMode` |
| `leader_hold: custom` | `DoCommand` with the payloads you configure |
| gripper mirroring | `DoCommand` on the leader gripper; `DoCommand` or `Open`/`Grab` on the follower gripper |

The follower arm, its gripper and the motion service must all be in the same viam-server's
frame system: either on the same machine, or on a machine configured as a remote. The
follower needs a `frame` block so `follower_component` has a pose in `world`.

## What happens on start

Nothing on the follower side moves. By default the service **captures** both arms where they
are: the follower's current pose and the leader's current pose become the session's
reference, the gap between them is absorbed as a constant offset, and from then on the
leader's motion is mirrored as scaled deltas from the follower's pose. The leader is handed
to the operator according to `leader_hold`. There is no clutch and no IK on this side.

### Optional: `align_on_start`

With `"align_on_start": true` the service instead **drives the leader to the follower's pose**
before mirroring, so the two arms correspond absolutely:

1. The follower's current pose is read from the motion service (`GetPose` of
   `follower_component` in `world`).
2. That pose is mapped back into the leader's base frame (the inverse of the mapping below)
   and solved for leader joints by a best-effort IK: a 5-DoF arm usually cannot reach a
   6-DoF pose exactly, so the solver returns the closest configuration and reports the
   residual (`align_position_error_mm`, `align_orientation_error_deg` in `status`).
3. The leader is driven there with a normal profiled move, blocking.
4. The leader is handed to the operator according to `leader_hold`.
5. The leftover gap from step 2 is captured as a constant correction, so the first target
   sent to the follower is **exactly where the follower already is**.
6. The follower is registered with the pipeline at its current pose, and mirroring begins.

Align is best effort: a leader that cannot reach the mapped follower pose goes as close as it
can and the gap is absorbed into the offset (and logged when it is large). Set
`align_max_position_error_mm` to refuse instead. A joint that stops short of the IK target
(an arm clamping to its *calibrated* range, which the model does not know) is accepted and
absorbed; an arm that accepts the move but does not move at all is refused when a limit is
set and warned about otherwise.

On **stop**, the follower is removed from the pipeline and holds where it is, and the leader
is released from its hold.

## The mapping

```
follower_pose = origin ∘ scale(leader_pose ∘ leader_tool_offset) ∘ tool_orientation
```

- `leader_pose` is the leader arm's `EndPosition`, in its own base frame.
- `leader_tool_offset_mm` moves the mirrored point from the leader's `EndPosition` frame out
  to its physical tip, in the tool frame, **before** scaling. If `EndPosition` is a wrist
  frame some distance behind the handle tip, a leader rotation pivots the follower about a
  point `scale × that distance` off its tip unless this is set. Measure it from the leader's
  tool frame to the point you hold. (For the stock SO-101 leader handle it is `{"z": 96}`.)
- `scale` multiplies the leader's (offset) translation about its base, so a desktop arm can
  span a large follower workspace.
- `origin_translation_mm` / `origin_orientation` place the leader's base in the follower's
  world frame.
- `tool_orientation` is a fixed rotation in the tool frame that reconciles the two arms'
  tool-frame conventions.

To tune `scale` and `origin_*`: run the client with `-align-only` and read
`align_position_error_mm` / `align_orientation_error_deg`; adjust until the leader lands
where you expect it next to the follower.

Orientation mirroring is relative to this mapping and limited by the leader: a 5-DoF wrist
cannot express every rotation, and those it cannot are never requested. Set
`rotation_enabled: false` to hold the follower's starting orientation and mirror position
only, which many operators prefer.

## Configuration

```json
{
  "leader_arm": "leader-arm",
  "leader_gripper": "leader-gripper",
  "follower_component": "xarm-gripper",
  "follower_gripper": "xarm-gripper",
  "scale": 2.0,
  "leader_tool_offset_mm": { "x": 0, "y": 0, "z": 96 },
  "origin_translation_mm": { "x": 0, "y": 0, "z": 0 },
  "origin_orientation": { "type": "ov_degrees", "value": { "x": 0, "y": 0, "z": 1, "th": 0 } },
  "rotation_enabled": true,
  "align_on_start": true,
  "leader_hold": "manual",
  "leader_gripper_commands": { "on_start": [ { "command": "set_torque", "enable": false } ] },
  "leader_gripper_read_command": { "command": "get_position" },
  "leader_gripper_read_key": "position_percentage",
  "follower_gripper_mode": "command",
  "follower_gripper_command": { "set": "$steps" },
  "gripper_range": 850
}
```

The gripper commands above are the SO-101 leader's and the uFactory xArm follower's. They
are examples: this service defaults none of them, because every gripper module speaks a
different DoCommand dialect today.

### Several leader/follower pairs in one service

One service drives every pair on a machine. Top-level attributes become defaults shared by
every pair; each entry of `pairs` has a `name` and overrides whatever it needs, normally just
the arm names and the origin. A key set in a pair wins; a key it omits is inherited. Without
`pairs`, the top-level attributes are the one pair, named `main`.

```json
{
  "scale": 2.5,
  "leader_tool_offset_mm": { "x": 0, "y": 0, "z": 96 },
  "align_on_start": true,
  "leader_hold": "manual",
  "pairs": [
    {
      "name": "left",
      "leader_arm": "left-leader-arm", "follower_component": "left-gripper",
      "origin_translation_mm": { "x": 0, "y": 1003, "z": 0 }
    },
    {
      "name": "right",
      "leader_arm": "right-leader-arm", "follower_component": "right-gripper"
    }
  ]
}
```

Every pair registers with the same motion service teleop pipeline, which plans all registered
followers in **one joint plan**. So a target the planner rejects on one side (a collision with
the table, a wall, or the other arm) stalls the other side too. That is a property of the
pipeline, and running the pairs as separate services does not change it.

### Attributes

| Name | Type | Inclusion | Description |
| --- | --- | --- | --- |
| `leader_arm` | string | **Required** | Leader arm to read. |
| `follower_component` | string | **Required** | Frame-system name the pipeline positions, normally the follower gripper so its TCP tracks the leader. |
| `motion_service` | string | Optional | Motion service name. Default `builtin`. |
| `leader_gripper` / `follower_gripper` | string | Optional | Set both to mirror the gripper; then `leader_gripper_read_command` and `leader_gripper_read_key` are required. |
| `scale` | number | Optional | Translation scale, leader → follower. Default `1`. |
| `origin_translation_mm` | `{x,y,z}` | Optional | Leader base position in follower world. Default origin. |
| `origin_orientation` | orientation | Optional | Leader base orientation in follower world (frame-config syntax, e.g. `ov_degrees`). Default identity. |
| `leader_tool_offset_mm` | `{x,y,z}` | Optional | Leader tip in its `EndPosition` (tool) frame, applied before `scale`. Default zero. |
| `tool_orientation` | orientation | Optional | Fixed tool-frame rotation. Default identity. |
| `rotation_enabled` | bool | Optional | Mirror orientation as well as position. Default `true`. |
| `rate_hz` | number | Optional | Leader sampling rate. Default `60`. Sends are at most one in flight ("latest wins"), so the send rate is bounded by the RPC round trip. |
| `smooth_tau_ms` | number | Optional | EMA/SLERP time constant. Default `100`; `0` disables. |
| `pos_deadzone_mm` / `rot_deadzone_deg` | number | Optional | Skip sends that move less than this. Default `1` / `1`. |
| `align_on_start` | bool | Optional | Drive the leader to the follower's pose before mirroring. Default `false`: capture both arms where they are. |
| `align_max_position_error_mm` | number | Optional | With `align_on_start`, refuse to start if the leader ends up further than this from the mapped follower pose. Default `0`: best effort, the gap is absorbed and logged. |
| `leader_hold` | string | Optional | `none` (default), `manual`, or `custom`. See [Leader hold](#leader-hold). |
| `manual_mode` | object | Optional | With `manual`: the `extra` map passed to the arm's `SetManualMode` (module-specific gains, for example). |
| `leader_commands` | object | Optional | With `custom`: `before_start`, `after_start`, `on_stop`, each a list of DoCommand payloads for the leader arm. |
| `leader_gripper_commands` | object | Optional | `on_start` and `on_stop`, each a list of DoCommand payloads for the leader gripper (any hold). Typical use: free the trigger servo. |
| `follower_gripper_mode` | string | Optional | `open_close` (default; gripper API `Open` above 60 %, `Grab` below 40 %), `command` (send `follower_gripper_command`), or `off`. `xarm` is accepted as an alias of `command` for one release. |
| `follower_gripper_command` | object | With `command` mode | DoCommand template for the follower gripper. `"$steps"` is replaced by `round(pct / 100 × gripper_range)` and `"$pct"` by the percentage, at any nesting depth. |
| `gripper_range` | number | Optional | Value of `$steps` at 100 %. Default `850`. |
| `gripper_invert` | bool | Optional | Flip the leader percentage. A leader trigger that reads near `0` at rest needs this for squeeze-to-close. |
| `leader_gripper_read_command` | object | With `leader_gripper` | DoCommand sent to the leader gripper every cycle. |
| `leader_gripper_read_key` | string | With `leader_gripper` | Dotted path into the response holding the 0..100 opening. Checked once at start; a wrong key fails `start` naming the keys that came back. |
| `pairs` | list | Optional | Leader/follower pairs, each a `name` plus any of the attributes above, overlaid on the top-level defaults. |
| `auto_start` | bool | Optional | Start every pair on configure. Default `false`. A pair that fails to start fails the configure and releases the pairs that did start. |

Fixed, no longer attributes: outlier rejection at 4500 mm/s and 1800 deg/s (accepted after 5
in a row), a 500 ms send timeout, 10 consecutive cycle failures before the loop stops, a 3 %
gripper deadband, and align residual compensation (always on).

### Leader hold

How the leader is presented to the operator's hand once a session starts.

| Value | What is sent | When |
| --- | --- | --- |
| `none` | Nothing. The leader stays however its module left it. | Default. A leader that is already compliant, or one you hold yourself. |
| `manual` | `SetManualMode(false)` before capture/align, `SetManualMode(true, manual_mode)` after, `SetManualMode(false)` on stop, through the arm API. Refused at `start` if the arm's `Properties` do not report `SupportManualMode`. | Any arm module that implements manual mode (the SO-101 module does since its v0.13). |
| `custom` | Your `leader_commands` lists at the same three points. `before_start` failures are logged and ignored; an `after_start` failure aborts `start` and sends `on_stop`; `on_stop` failures are warned. | A leader whose module has no manual-mode API. |

A loop that stops itself (the error threshold, or the pipeline no longer knowing the
follower) deliberately **does not** send the stop commands, because a hand may be on the
leader; `stop` sends them, and a `start` on a session that ended that way sends them first.

## Gripper mirroring

Every cycle the leader gripper is read with `leader_gripper_read_command` and the 0..100
value at `leader_gripper_read_key` (inverted if `gripper_invert`) goes to the follower on
change (3 % deadband). Follower writes are asynchronous and latest-wins, because some
gripper modules block until the jaw arrives. A failing follower write is logged and does not
stop the mirror; a failing leader read counts as a cycle error.

The leader gripper is also sent `leader_gripper_commands.on_start` once the leader is at its
session pose, and `on_stop` on stop. An SO-101 leader needs
`{"command": "set_torque", "enable": false}` on start to free its trigger servo, because
the arm's bring-up torques the whole servo bus.

When the gripper API gains a position-agnostic get/set, a mode will be added beside
`command`; nothing here is deprecated by that.

## DoCommand

```json
{ "command": "start" }
{ "command": "stop" }
{ "command": "status" }
{ "command": "start", "pair": "left" }
```

Each command acts on every pair, or on one pair when given `"pair": "<name>"` (`"side"` is
accepted as an alias). Any other key is an error, so a typo cannot silently act on every
pair. `start` attempts every pair even if one fails (an align the leader cannot reach, say),
and returns the failure as an error; `status` then shows which pairs are running. `stop`
likewise releases every leader it can.

Every command returns `{"running": <any pair running>, "pairs": {"<name>": {...}}}`. Each
pair reports:

| Key | Meaning |
| --- | --- |
| `running` | the mirror loop is alive |
| `session_id` | set at each `start`; also on every structured log line of the session |
| `stopped_reason` | `""`, `stopped`, `session_lost` or `max_errors` |
| `leader_held` | the hold is applied and has not been released |
| `last_hold_error` | the last failure applying or releasing the hold |
| `cycles` | ticks completed without error |
| `sends` / `dropped` | `teleop_move`s sent, and ticks whose target was dropped because one was in flight |
| `deadzoned` / `rejected` | filter skips and outlier rejections |
| `tick_overruns` | ticks that took longer than the period (`rate_hz` not being met) |
| `consecutive_errors` / `errors_total` / `last_error` / `last_error_at` | cycle errors |
| `last_rpc_ms` / `leader_read_ms` / `gripper_read_ms` | the last `teleop_move`, leader `EndPosition` and gripper read durations |
| `rate_hz` | the configured rate |
| `align_position_error_mm` / `align_orientation_error_deg` / `align_poses` | what `start` established |
| `gripper_pct` / `gripper_writes` | the last percentage accepted and how many follower writes were issued |
| `pipeline_error` / `pipeline_plan_ms` | copied from the motion service's `teleop_status` every 200 ms: a goal the planner rejects shows up **only** here, the sends keep succeeding |

## Running as a client

```sh
VIAM_API_KEY_ID=... VIAM_API_KEY=... \
go run ./cmd/teleop-client -config teleop.json -leader-addr so101-station.viam.cloud \
  [-follower-addr xarm-cell.viam.cloud] [-pair left] [-align-only]
```

`teleop.json` holds the attribute block above, `pairs` included; unknown keys are rejected.
`-align-only` aligns the leader, prints the residual, and exits, which is the quickest way to
tune `scale` and `origin_*`. The leader can be served by a cloud-less local viam-server on
the laptop (`-leader-addr localhost:8080 -leader-insecure`). Expect tens of sends per second
from a laptop (two network RPCs per cycle); on the machine itself the pipeline's round trip
is the limit.

## Migrating from `devrel:so101:teleop-cartesian`

The config block is the same shape with these changes. Removed keys are rejected by name,
not ignored, so a stale config fails `configure` rather than running without the behaviour
it asked for.

| Old | New |
| --- | --- |
| `leader_hold: manual` | `leader_hold: manual` (through the arm API; needs an SO-101 module that implements manual mode) |
| `leader_hold: stiff` | `leader_hold: none` |
| `leader_hold: torque_off` | `leader_hold: custom` with `leader_commands.after_start: [{"command": "set_torque", "enable": false}]` and `on_stop: [{"command": "set_torque", "enable": true}]` |
| `manual_mode` | unchanged (passed as the `extra` map) |
| the leader gripper's trigger release (was implicit) | `leader_gripper_commands.on_start: [{"command": "set_torque", "enable": false}]` |
| the leader gripper read (was implicit) | `leader_gripper_read_command: {"command": "get_position"}`, `leader_gripper_read_key: "position_percentage"` |
| `follower_gripper_mode: xarm` | `follower_gripper_mode: command`, `follower_gripper_command: {"set": "$steps"}` (`xarm` still accepted for one release) |
| `gripper_in_min_pct` / `gripper_in_max_pct` | removed; rescale in the leader gripper module or use `gripper_invert` |
| `gripper_deadband_pct`, `outlier_*`, `move_timeout_ms`, `max_consecutive_errors`, `compensate_align_residual`, `align_orientation_weight_mm_per_rad` | removed; fixed constants |
| `blocked_feedback`, `feedback_*`, `follower_arm`, `gripper_lock`, `gripper_lock_ms` | removed with the SO-101 manual-hold dependency |
| DoCommand `"side"` | `"pair"` (`side` still accepted) |
| status `blocked`, `blocked_reason`, `gripper_locked`, `align_leader_joints` | removed; `align_poses.leader_inputs_deg` carries the joints |

## Safety

- The motion service's teleop joint velocity/acceleration limits are the hard bound on how
  fast the follower moves. Tune them on the follower before the first session, with nobody
  in reach.
- With `align_on_start` the **leader** moves on `start`, under its own module's motion limits.
- A pipeline error (a collision with a world obstacle, an unreachable goal) does not stop the
  mirror: the follower stands still while `sends` keep succeeding. Watch `pipeline_error`.

## Requirements

- The follower machine's viam-server needs the motion teleop DoCommands (rdk ≥ v0.119.0) and
  benefits from the streamed teleop executor (newer rdk plus a follower module that implements
  `MoveThroughJointPositionsStreamed`; the uFactory xArm module does).
