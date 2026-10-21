# AGENTS.md

Orientation for an AI coding agent (or a human) picking up robot-side
integration on this repo. Read `README.md` first for setup/running/the
full control-scheme spec — this file just orients you inside the code.

## What this repo is
VR headset teleop pipeline: a WebXR client (`static/`) talking to a Python/
aiohttp server (`server/`) over HTTPS + one WebSocket, on the LAN. The
server streams 3 camera feeds to the headset and receives head/controller
pose + button/axis data every XR frame; `server/teleop.py` turns that into
arm/base/neck commands.

## What's implemented right now
`server/teleop.py`'s `TeleopController` has the enable state machine
(IDLE/ARMS/BASE) and controller-pose -> EE-target mapping, plus a
precision/safety layer on top of it:

1. **Motion scaling** — `MOTION_SCALE_POS` / `MOTION_SCALE_ROT` constants
   (1.0 = pass-through). Scales operator motion before it's applied to the
   EE target.
2. **Pose smoothing** — the `PoseFilter` class, EMA smoothing on raw head/
   controller pose, weight set by `POSE_FILTER_ALPHA`. This is the one
   deliberate, bounded source of added latency in the whole pipeline.
3. **Rate limiting** — `TeleopController._rate_limit()`, caps how fast a
   commanded EE target can move/rotate (`MAX_EE_LINEAR_MPS`,
   `MAX_EE_ANGULAR_RPS`), independent of whatever the input pose is doing —
   this is what catches a tracking glitch before it becomes a sudden jump.
4. **Recenter / re-clutch** — thumbstick click (`RECENTER_BUTTON_IDX`)
   while an arm is enabled re-baselines the controller-to-EE mapping at
   that arm's *current* commanded pose, so the arm doesn't move when you
   recenter — only the mapping origin does.
5. **Dead-man's switch** — `TeleopController._handle_deadman()`; if an
   enabled arm's controller pose goes missing for longer than
   `DEADMAN_TIMEOUT_MS`, that arm auto-disables and goes home rather than
   holding or extrapolating stale data.
6. **Visual/haptic feedback** — the server pushes a
   `{"type": "teleop_state", "state", "arm_enabled", "warnings"}` JSON
   message back over the same WebSocket after every pose update.
   `static/main.js`'s `handleTeleopState()` uses it to drive a per-
   controller indicator sphere (color-coded), haptic pulses on state
   transitions, and a head-locked status HUD.

All six were verified in isolation with scripted pose/button sequences
(no robot attached) — if you change this logic, re-verify the same way:
construct a `TeleopController`, feed it synthetic payload dicts with
controlled timestamps, and check `.state` / `.arm_enabled` / the returned
command dict. There's no committed test suite yet, just ad-hoc scripts.

## Where to plug in the real robot
`TeleopController._dispatch(commands)`, at the bottom of `teleop.py`, is
the single seam — it currently only logs. `commands` contains:

- `state`: `"IDLE" | "ARMS" | "BASE"`
- `arm_enabled`: `{"left": bool, "right": bool}`
- `warnings`: list of strings (e.g. `"right_tracking_lost"`)
- `neck`: `{"yaw_deg", "pitch_deg"}`, when a head pose was available
- `left_ee_target` / `right_ee_target`: `{"pos": [x,y,z], "quat": [x,y,z,w]}`
  (robot base frame, meters), present when that arm is enabled
- `left_gripper` / `right_gripper`: trigger value 0..1, when that arm is
  enabled
- `base_velocity`: `{"vx", "vy", "yaw_rate"}`, present when `BASE` is active

Replace (or call out from) `_dispatch` to send these wherever they need to
go — ROS publish, a vendor SDK, a serial protocol, whatever this robot
turns out to use.

## Tunable constants
All at the top of `server/teleop.py`, currently placeholders, none
validated against real hardware: `HOME_POSE`, `MOTION_SCALE_POS`,
`MOTION_SCALE_ROT`, `POSE_FILTER_ALPHA`, `MAX_EE_LINEAR_MPS`,
`MAX_EE_ANGULAR_RPS`, `NECK_YAW_LIMIT_DEG`, `NECK_PITCH_LIMIT_DEG`,
`MAX_BASE_LINEAR_MPS`, `MAX_BASE_YAW_RPS`, `DEADMAN_TIMEOUT_MS`,
`COMBO_WINDOW_MS`. Retune all of these once real arms/base are connected.

## Camera streams
`server/cameras.py` — only the `"head"` stream is real hardware (this dev
machine's webcam, standing in for the robot's ZED camera).
`"gripper_left"` / `"gripper_right"` are placeholder "NO SIGNAL" frames.
Swap the capture threads for the real sources; the multiplexed WebSocket
wire format (`[1 stream-id byte] + [JPEG bytes]`) and the client-side demux
in `static/main.js` don't need to change.

## Whole-body clutch teleop (YOR)
`server/whole_body.py` (`WholeBodyTeleop`) is the clutched relative-delta
controller for YOR: hold grip to drive an arm, A+X toggles base mode, head
drives the neck, and `server/lift_retarget.py` sets the lift height from head
height + clutched arm targets. It is pure logic (`update(payload, robot_state,
now_s) -> commands`); the robot bridge and `../robot/whole_body_teleop.py`
launcher come next. Constants marked VERIFY (XR->robot axes, neck signs, lift
travel/band/speed) are unvalidated on hardware.
Tests: `python tests/test_whole_body.py` (also pytest-compatible).

`server/yor_bridge.py` is the robot bridge: it serves the page, runs
`WholeBodyTeleop` at 30 Hz and drives the robot through yor.py's RPC. The YOR
repo's `robot/whole_body_teleop.py` is a thin launcher for it (Orin video
address + neck calibration file). The exact RPC calls, frames and units are in
`docs/YOR_RPC_INTERFACE.md` -- a simulator that serves those calls can run the
unchanged bridge (`python server/yor_bridge.py --rpc-host localhost`).

## Two-machine setup (current)
The robot Pi serves the page and runs control (no video); the Orin runs
`server/server.py --video-only --port 8444` for camera feeds. The page opens
the video socket from `?video=<host:port>`. Orin setup and the camera
ownership constraint: `docs/ORIN_VIDEO_SERVER.md`. Check a video server with
`tools/video_probe.py <host:port>` or `GET /health`.

## Stereo ZED view
Stream 3 (`head_stereo`) carries rectified left|right side by side; the video
server also sends a `stereo_info` JSON message with per-eye intrinsics.
`static/main.js` shows each half to one eye only (three.js XR layers 1/2),
true scale from fx/fy/cx/cy, eye planes `ipd` apart. IPD: right stick up/down
(not in base mode), right stick click resets to the headset's measured IPD,
stored in localStorage, `?ipd=<mm>` overrides. Both eye planes are shifted
down to the vertical centre of the headset's eye frustums (Quest sees further
down than up); `?center=0` turns that off.

## HUD
`static/main.js` "Halo HUD": head-locked canvas panels (top: mode/links/REC/
neck tape; sides: arm clutch, IK, gripper gauge; bottom: lift, base, task;
alert banner). Each redraws only when its view changes. Data: the
`teleop_state` message plus optional `robot` (from
`robot/whole_body_teleop.py` `RobotLink.hud()`, polled at 5 Hz off the control
path) and optional `recording: {active, episode, elapsed_s, task}` (nothing
sends it yet). Open the page in a desktop browser to preview it without VR.
