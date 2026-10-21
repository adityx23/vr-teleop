# VR Teleop

VR headset teleop over the LAN: multi-camera streaming into the headset,
plus head/controller pose capture driving a (currently stubbed) arm/base/
head-camera control pipeline.

## Architecture
- `server/server.py` — HTTPS + WebSocket server on this PC.
- `server/cameras.py` — `MultiCameraStreamer`: broadcasts 3 camera feeds
  multiplexed over one WebSocket (`[1 stream-id byte] + [JPEG bytes]`).
  Only the **head** stream is real hardware right now (this PC's webcam,
  standing in for the robot's ZED camera); **gripper_left**/**gripper_right**
  are placeholder "NO SIGNAL" frames until those cameras are wired up.
- `static/` — WebXR (Three.js) page. Renders all 3 streams as head-locked
  screens (one large center view for the head/ZED cam, two small insets in
  the lower corners for the gripper cams) and both controllers. Every XR
  frame it sends head pose + left/right controller pose (position,
  quaternion, trigger/grip values, thumbstick axes, button states) back to
  the server.
- `server/teleop.py` — `TeleopController`: the enable state machine +
  pose/button -> command mapping described below. `_dispatch()` is the one
  place that currently just logs computed commands — replace it with real
  robot calls once that interface exists.
- `server/quat.py` — small quaternion helpers for frame-relative poses.

## Control scheme (robot side, as specified)
Button layout follows the WebXR "xr-standard" mapping Quest Touch
controllers use: `buttons[4]` is the primary face button — **A** on the
right controller, **X** on the left. Thumbsticks are `axes[2]`/`axes[3]`.

**Enable state machine** — `IDLE` / `ARMS` / `BASE`, mutually exclusive:
- From `IDLE`: **A** alone enables the right arm, **X** alone enables the
  left arm (state -> `ARMS`). **A+X together** enables the base
  (state -> `BASE`).
- From `ARMS`: pressing the *other* side's button alone brings that arm up
  too (so you can enable one arm, then the other). **A+X together**
  disables both arms, commands them to their home pose, and returns to
  `IDLE`.
- From `BASE`: **A+X together** disables the base and returns to `IDLE`.

"Together" is detected via a 150ms window (`COMBO_WINDOW_MS` in
`teleop.py`), not an exact-same-frame requirement, since two physical button
presses are never perfectly simultaneous. A single-button press is held
for that same window before being committed, in case the other button
joins it as a combo.

**Arm control (while enabled):**
- Controller pose -> end-effector target: each controller's pose is first
  expressed *relative to the head pose* (so the mapping holds regardless of
  which way you're facing/standing), then tracked as a **delta** from the
  pose captured at the moment that arm was enabled. That delta is applied
  on top of the arm's home pose. This is a "clutch" style mapping — you can
  disable/re-enable an arm to reset where your controller needs to be
  without the arm jumping.
- Trigger value (0–1) -> gripper close amount for that arm.

**Base control (while enabled):** left thumbstick -> linear x/y velocity,
right thumbstick's x-axis -> yaw rate. Scaled by `MAX_BASE_LINEAR_MPS` /
`MAX_BASE_YAW_RPS` in `teleop.py` — both placeholders, tune for the real
base.

**Head camera (2-DOF active perception / ZED neck):** pan/tilt mirrors the
headset's yaw/pitch 1:1, continuously, independent of arm/base enable
state — so you can always look around. Zeroed ("straight ahead") to
whatever orientation the headset reports on the first frame of the
session; clamped to `NECK_YAW_LIMIT_DEG` / `NECK_PITCH_LIMIT_DEG`.

All of the above is implemented as pure logic against a single shared
`TeleopController` instance and verified with scripted button/pose
sequences (no robot attached) — see the state transitions, EE delta
tracking, neck mirroring, and base mapping all producing correct output in
isolation. **No real camera (ZED/gripper) or robot arm/base interface is
wired up yet** — that's the explicit next step once that hardware/SDK
access exists.

## One-time setup

```powershell
cd C:\Users\Aditya\vr_teleop
python -m pip install -r requirements.txt
```

Find this PC's LAN IP (Wi-Fi adapter) — already known to be `192.168.123.156`
on this network. Generate a cert that's valid for that IP (WebXR requires
HTTPS when not on localhost):

```powershell
python server\gen_cert.py 192.168.123.156
```

Re-run `gen_cert.py` if this PC's IP changes (different network, DHCP
renewal, etc.) — pass whatever the new IP is.

## Running

```powershell
python server\server.py
```

You should see: `Serving https://0.0.0.0:8443 ...`

### Windows Firewall
First run will likely trigger a Windows Defender Firewall prompt for
`python.exe` — allow it. If you don't get a prompt and the headset can't
connect, add a rule manually:

```powershell
New-NetFirewallRule -DisplayName "VR Teleop" -Direction Inbound -Protocol TCP -LocalPort 8443 -Action Allow
```

### Network note
WebXR and the WebSocket both require the headset to actually reach this PC
on the LAN. If connections work from this machine but not from the
headset, check for client/AP isolation on the router/hotspot — some
networks (especially ones named like dedicated "teleop" networks) block
device-to-device traffic by default even on the same subnet.

## On the headset (192.168.123.227)
1. Make sure the headset is on the same Wi-Fi network as this PC.
2. Open the browser (e.g. Oculus/Meta Browser) and go to:
   `https://192.168.123.156:8443`
3. Accept the self-signed certificate warning (Advanced -> Proceed /
   "visit this website").
4. Click **Enter VR**. You'll see the head-camera feed as a big head-locked
   screen and two small gripper-camera insets (currently "NO SIGNAL" stubs)
   in the lower corners, plus your controllers as colored cones
   (green = left, red = right). All three screens follow your head like a
   HUD.
5. Watch this PC's console: it logs the current state
   (`IDLE`/`ARMS`/`BASE`), which arms are enabled, the computed neck
   pan/tilt, arm EE targets, and base velocity a couple of times a second —
   try the button combos above and confirm the state machine responds as
   expected.

## Simulator (YOR MuJoCo)
The YOR repo's `robot/yor_mujoco.py` serves the same RPC calls as `yor.py`
(`docs/YOR_RPC_INTERFACE.md`), so `server/yor_bridge.py` drives the sim
unchanged. The sim publishes its head ZED view over local ZMQ; the video
server streams it with the `"zmq"` camera source:

```bash
python server/server.py --video-only --port 8444 --camera-config camera_config.sim.json
python server/yor_bridge.py --rpc-host localhost --video <this-pc-ip>:8444
```

From the YOR repo, `python robot/sim_vr_teleop.py` starts the sim and both of
these in one command; setup and controls are in YOR's `docs/SIM_VR_TELEOP.md`.

## Known limitations / next steps
- Camera streams are JPEG-over-WebSocket (head cam ~20fps, gripper stubs
  ~2fps, 640x480) — fine for validating the pipeline, but swap for a proper
  codec (WebRTC/H.264) once latency matters for real teleop.
- Gripper cameras are placeholder frames; head cam is this PC's webcam
  standing in for the ZED. Swap `cameras.py`'s capture threads for the real
  ZED SDK + gripper camera sources when available.
- `teleop.py`'s `_dispatch()` only logs — no real arm IK solver, no base
  driver, no neck servo connection yet. The EE *targets* it computes are
  ready to feed into whatever IK solver the robot uses.
- No auth on the server — fine on a trusted LAN, not for anything else.
