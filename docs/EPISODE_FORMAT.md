# Episode stream format (Pi -> Orin)

What `server/yor_bridge.py` (run on the robot as YOR's `robot/whole_body_teleop.py`)
publishes for data collection. YOR's older `robot/teleop/vr_teleop.py` publishes the same
topics and follows the same null/error rules.

ZMQ PUB, bound on the Pi at `*:5558` (`--episode-port`). Every message is two frames:
`[topic, JSON]`. Timestamps are Pi `time.time()` seconds.

## Topics

| Topic | When | JSON |
|---|---|---|
| `episode/signal` | episode start | `{timestamp, active: true, episode_id, task, operator?, notes?, ...}` + [cycle fields](#cycles-orin---pi) |
| `episode/signal` | episode end | `{timestamp, active: false, success, episode_id, num_frames}` + `cycle`, `subtask_index` in a cycle |
| `teleop/episode_result` | episode end | `{timestamp, result: "success" \| "fail", episode_id}` + `cycle`, `subtask_index` in a cycle |
| `teleop/episode` | 30 Hz while an episode is open | one frame, below |

`episode_id` is `YYYY-MM-DD--HH-MM-SS` (Pi local time at start). Frames are only sent
between a start and its end. Right stick click starts / ends as success, left stick click
ends as fail, with teleop on or off (so policy rollouts can be marked too).

## Cycles (Orin -> Pi)

A cycle is a self-resetting list of subtasks (open fridge, grab soda, close fridge, open
fridge, place soda, close fridge), each attempt recorded as its own episode. The Orin
collector picks the cycle and announces it; the Pi, which gets the stick clicks, steps
through it (`server/cycles.py`). What the Orin collector has to change: [ORIN_CYCLES.md](ORIN_CYCLES.md).

The collector binds a ZMQ PUB (default `*:5559`) and sends, about once a second for as long
as it runs, `[b"collection/cycle", JSON]`:

```json
{"timestamp": 1793122155.2, "cycle": "fridge_soda", "run_id": "3f9c0a1b2d4e",
 "subtasks": ["open the fridge", "grab the soda", "close the fridge",
              "open the fridge", "place the soda in the fridge", "close the fridge"]}
```

`subtasks` entries may also be `{"task": "...", ...}`. `run_id` is new each time the
collector starts. The bridge connects to it with `--cycle-source host:port` (YOR's
`whole_body_teleop.py`: `CYCLE_SOURCE` in `robot/config.py`, default `ORIN_IP:5559`).
`tools/announce_cycle.py` is a stand-in collector for testing. Until a cycle is heard, every
episode gets `--task` as before.

Clicks in a cycle:

| Click | No episode open | Episode open |
|---|---|---|
| right | start the current subtask | end as success, move to the next subtask (wraps to the first: one more lap) |
| left | go back one subtask | end as fail; the next right click retries the same subtask |

So to pause, end a subtask as a success and leave the next one un-started (or start it and
fail it; those short fails are kept and can be filtered out later, as can a ~1 s success
from double-clicking forward after going back too far). A different cycle announced
mid-episode takes effect when that episode ends. The current subtask is saved per cycle name
in `teleop_data/cycle_state.json`, so a bridge or collector restart resumes where the scene
is (same name and same subtask list; otherwise it starts at the first subtask).

Extra `episode/signal` start fields in a cycle (also in the `.yep` metadata):

| Field | Meaning |
|---|---|
| `task` | the subtask's prompt (replaces `--task`) |
| `cycle`, `cycle_run_id` | the announced cycle name and `run_id` |
| `subtask_index`, `num_subtasks` | 0-based position in the cycle, and its length |
| `subtask_attempt` | 1 for the first try at this subtask since reaching it, +1 per retry |
| `cycle_lap` | completed passes through the whole cycle |

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
| `lift_height` | `lift_height_error` | no fresh Pico height (reply older than 0.5 s, no reply, serial not open, or `YOR_LIFT=0`) |
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
| `lift_height` | float \| null | m, carriage travel above the bottom stop when `lift_homed` (see [Lift height](#lift-height)); polled from the Pico at 50 Hz | measured |
| `lift_height_error` | str \| null | see above | |
| `lift_homed` | bool | `lift_height` is absolute: the lift was zeroed at the bottom stop since the Pico last powered on | |
| `lift_ref` | str \| null | what `lift_height` is measured from: `"bottom_hard_stop"` (homed) or `"pico_power_on"` (not homed); `null` with no lift or an older `yor.py` | |
| `lift_cmd` | int | last manual lift command: +1 up, -1 down, 0 stop | commanded |
| `neck_deg` | float[2] \| null | `[pan, tilt]`, deg | measured |
| `neck_cmd_deg` | float[2] \| null | `[pan, tilt]` goal, deg | commanded |
| `view_pitch_deg` | float \| null | headset view tilt the operator set (right stick left/right, teleop off), added to the head pitch before it becomes the neck tilt; − = robot looks further down than the operator, deg | setting |
| `base_pose`, `cam_pose`, `zed_timestamp`, `zed_age_s` | null | always `null` here: the Orin has the ZED and joins it by timestamp | |

`enc_vel`, `neck_deg` and `{side}_ee` have no `_error` partner yet: `null`
there means the read failed.

### Lift height

`lift_height` = carriage travel in metres above the fully-down reference: **0 = fully down
(the bottom hard stop), increasing upward**. Measured travel: **0 .. 0.6726 m** (bottom stop to
top stop, 2026-10-23). Floor (and robot base frame) to the carriage at travel 0: TBD (not measured yet), so absolute
height = offset + `lift_height`.

The lift has an incremental encoder and no limit switches. The Pico zeroes it wherever the
carriage is at power-on, so on its own the reading moves between sessions (e.g. 2026-10-26
data starting at 0.000 and going to -0.33 m). The only fixed reference is the bottom hard
stop. When the carriage stalls there while driving down, the Pico re-zeroes the height. That
happens on `python robot/lift_home.py` (`lift_home` RPC), and also on any manual drive down
until the lift stops. From then on the reading is absolute and `lift_homed` is `true`. It
stays `true` across `yor.py` and bridge restarts. It goes back to `false` after a Pico reset,
power cycle or USB re-plug, and after a down-stall more than 10 mm above the bottom stop (an
obstruction: the Pico then re-zeroed in the wrong place).

When `lift_homed` is `false`, `lift_height` is still the raw reading, relative to the
Pico's power-on position (`lift_ref: "pico_power_on"`). It can be negative. Don't combine
it with homed data. Nothing is ever fabricated or shifted on the Pi. The lift is never
homed automatically, because driving it down at startup could put the arms into whatever is
below them. Instead, an episode started while it isn't homed logs a warning on the Pi, and
the headset HUD shows LIFT NOT HOMED (red while recording).

Repeatability: the Pico counts encoder ticks only while it drives the motor, so slip or
coast after a stop is not counted. Each re-zero logs the drift it corrected ("drift since
the last home: +x mm") on the `yor.py` console.

## Pi-side copy (`.yep`)

The bridge also saves each episode as `<episode-dir>/<episode_id>.yep` (YOR:
`teleop_data/episodes/`). Read it with `episode_store.read_episode(path)` ->
`(meta, data)`, where `data[field]` is a `(T, width)` float array.

- Missing values are `NaN` (the file keeps a per-field mask).
- `lift_homed` is stored per frame (1/0). `lift_ref` is not; it is `"bottom_hard_stop"`
  exactly when `lift_homed`.
- Error strings are not stored per frame. `meta["errors"]` is a list of
  `{seq, field, error}`, one entry each time an `_error` field changes; `error: null`
  means the signal came back at that `seq`.
- `meta` also holds the start-signal metadata, `success`, `started_at`, `ended_at`,
  `num_frames` and `frame_hz`.

`python server/episode_store.py FILE [--csv out.csv]` prints a summary or writes a CSV.
