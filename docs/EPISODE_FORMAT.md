# Episode stream format (Pi -> Orin)

What `server/yor_bridge.py` (run on the robot as YOR's `robot/whole_body_teleop.py`)
publishes for data collection. YOR's older `robot/teleop/vr_teleop.py` publishes the same
topics and follows the same null/error rules.

ZMQ PUB, bound on the Pi at `*:5558` (`--episode-port`). Every message is two frames:
`[topic, JSON]`. Timestamps are Pi `time.time()` seconds.

## Topics

| Topic | When | JSON |
|---|---|---|
| `episode/signal` | episode start | `{timestamp, active: true, episode_id, task, operator?, notes?, ...}` |
| `episode/signal` | episode end | `{timestamp, active: false, success, episode_id, num_frames}` |
| `teleop/episode_result` | episode end | `{timestamp, result: "success" \| "fail", episode_id}` |
| `teleop/episode` | 30 Hz while an episode is open | one frame, below |

`episode_id` is `YYYY-MM-DD--HH-MM-SS` (Pi local time at start). Frames are only sent
between a start and its end. Right stick click starts / ends as success, left stick click
ends as fail, with teleop on or off (so policy rollouts can be marked too).

## Frame (`teleop/episode`)

All robot values are read from `yor.py` once per frame, so they record whatever is driving
the robot (teleop or a policy). `{side}` is `left` and `right`.

### The one rule: no stand-ins

A field is either its own signal or `null`. No field is ever filled in from another one: a
missing measurement is not replaced by the command, and a missing command is not replaced by
the measurement. Where a field has an `_error` partner, `null` comes with a string saying
why, and the error is `null` exactly when the value is present.

| Value | Error partner | `null` means |
|---|---|---|
| `{side}_gripper` | `{side}_gripper_error` | no fresh gripper feedback (CAN 0x2A8 older than 0.25 s, not being read, or no gripper) |
| `{side}_ee_target` | `{side}_ee_target_error` | no teleop target: `"no teleop commands yet"`, `"teleop off"`, `"{side} arm not clutched"` |
| `{side}_gripper_cmd` | (none) | gripper is limp: nothing is being commanded |

### Fields

| Field | Type | Units / meaning | Measured or commanded |
|---|---|---|---|
| `timestamp` | float | Pi time the state was sampled, s | |
| `episode_id`, `seq`, `active` | str, int, bool | `seq` counts from 0 within the episode | |
| `teleop_on` | bool | VR teleop enabled (false while e.g. a policy drives) | |
| `{side}_joints` | float[7] | arm joint angles, rad | measured |
| `{side}_joints_cmd` | float[7] \| null | last joint target sent to the arm, rad; `null` until the first one | commanded |
| `{side}_ee` | float[7] \| null | end-effector pose `[qw, qx, qy, qz, x, y, z]`, m, robot base frame | measured (FK) |
| `{side}_ee_target` | float[7] \| null | teleop's EE target, same layout | commanded (teleop) |
| `{side}_ee_target_error` | str \| null | see above | |
| `{side}_ik_ok` | bool \| null | last IK solve converged; `null` if never asked | |
| `{side}_gripper` | float \| null | opening, 0 = closed .. 1 = open | measured |
| `{side}_gripper_cmd` | float \| null | opening, 0 = closed .. 1 = open | commanded |
| `{side}_gripper_error` | str \| null | see above | |
| `base_vel`, `cmd_vel` | float[3] | `[vx, vy, wz]`, m/s and rad/s, base frame (same value twice, for old collectors) | commanded |
| `enc_vel` | float[3] \| null | `[vx, vy, wz]` from swerve FK on the wheel encoders | measured |
| `lift_height` | float \| null | m | measured |
| `lift_cmd` | int | last manual lift command: +1 up, -1 down, 0 stop | commanded |
| `neck_deg` | float[2] \| null | `[pan, tilt]`, deg | measured |
| `neck_cmd_deg` | float[2] \| null | `[pan, tilt]` goal, deg | commanded |
| `base_pose`, `cam_pose`, `zed_timestamp`, `zed_age_s` | null | always `null` here: the Orin has the ZED and joins it by timestamp | |

`enc_vel`, `lift_height`, `neck_deg` and `{side}_ee` have no `_error` partner yet: `null`
there means the read failed.

## Pi-side copy (`.yep`)

The bridge also saves each episode as `<episode-dir>/<episode_id>.yep` (YOR:
`teleop_data/episodes/`). Read it with `episode_store.read_episode(path)` ->
`(meta, data)`, where `data[field]` is a `(T, width)` float array.

- Missing values are `NaN` (the file keeps a per-field mask).
- Error strings are not stored per frame. `meta["errors"]` is a list of
  `{seq, field, error}`, one entry each time an `_error` field changes; `error: null`
  means the signal came back at that `seq`.
- `meta` also holds the start-signal metadata, `success`, `started_at`, `ended_at`,
  `num_frames` and `frame_hz`.

`python server/episode_store.py FILE [--csv out.csv]` prints a summary or writes a CSV.
