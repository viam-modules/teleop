# Module viam:teleop

Arm teleoperation services for viam-server. Each model takes a human input device and drives
a follower arm through the builtin motion service's teleop pipeline (the `teleop_start` /
`teleop_move` / `teleop_stop` DoCommands), which plans IK and streams joints to the arm
under the motion service's velocity and acceleration limits. The module is pure Go (no cgo)
and builds for linux/amd64, linux/arm64, darwin/arm64 and windows/amd64.

## Models

- [`viam:teleop:leader-follower`](docs/leader-follower.md): a hand-guided leader arm's
  end-effector pose, read through the arm API, mirrored onto a follower arm of different
  kinematics. Optional align-on-start (the leader is driven to where the follower is), a
  configurable leader hold (the arm API's manual mode, or your own DoCommands), and gripper
  mirroring through configurable commands.

Planned: `viam:teleop:clutch` (a pose tracker body and an input controller button, as the
vive module's teleop service works today).

## Trying it from a laptop

`cmd/teleop-client` runs the leader-follower engine against live machines over gRPC with the
same attribute block the service takes:

```sh
VIAM_API_KEY_ID=... VIAM_API_KEY=... \
go run ./cmd/teleop-client -config examples/so101-to-xarm.json -leader-addr <machine>.viam.cloud -align-only
```

## Development

```
cmd/module/            module entrypoint
cmd/teleop-client/     the leader-follower engine from a laptop
services/leaderfollower/   the viam:teleop:leader-follower resource: attributes, pairs, DoCommand
internal/leaderfollower/   engine, mapping, best-effort IK, settings, hold, pose log
internal/pipeline/     client for the motion service teleop pipeline + the latest-wins sender
internal/filter/       time-normalised EMA/SLERP, outlier rejection, deadzone
internal/gripper/      follower gripper output (async, latest-wins) and leader gripper reader
internal/pairs/        "top-level defaults + pairs list" config resolution
internal/posemath/     orientation error, pose maps
internal/testfake/     fakes for tests: arm with real kinematics, motion service, gripper
```

`internal/*` imports no `services/*`, so the client binary runs the same engine as the
module. Production code imports nothing that needs cgo; `GOOS=windows GOARCH=amd64 go build
-tags no_cgo ./...` is the check. Tests use the module's own fakes in `internal/testfake`,
not rdk's `testutils/inject`.

```sh
go test ./...
make module.tar.gz
```

The Go module path is `teleop` (no host), so it cannot be imported by another Go module; it
is a binary, not a library.
