# VR Teleop

WebXR teleop for the YOR robot from a Quest headset over the LAN: the robot
Pi runs control, the Jetson Orin streams the cameras (ZED stereo + wrist
cams) into the headset.

## System (two machines + headset)

```
Quest headset (WebXR page, served by the Pi)
 ├─ wss://192.168.123.55:8443/ws    poses up, HUD down  -> Pi: whole-body control -> robot
 └─ wss://192.168.123.117:8444/ws   video down          -> Orin: ZED + wrist cams
```

- **Pi** (`192.168.123.55`): runs `robot/whole_body_teleop.py` from the YOR
  repo, which calls `server/server.py`'s `run()` with `server/whole_body.py`
  as the pose handler. Serves the page, drives arms/lift/neck/base. No video.
  Tells the page where the video server is via `GET /config.json`.
- **Orin** (`192.168.123.117`): `./start_video.sh` runs
  `server/server.py --video-only --port 8444` with `camera_config.orin.json`.
  Ignores poses, sends no commands. Details, contract and camera-ownership
  rules: `docs/ORIN_VIDEO_SERVER.md`.
- **Headset** (`192.168.123.227`): opens `https://192.168.123.55:8443`.
  `?video=<host:port>` overrides the video server, `?video=none` turns video
  off, `?ipd=<mm>` sets the stereo IPD.

## Code map
- `server/server.py` — HTTPS + WebSocket server (`run()`), `/health`,
  `/config.json`, `--video-only` mode for the Orin.
- `server/cameras.py` — `MultiCameraStreamer`: camera sources from a JSON
  config (`stub` / `opencv` / `zed` / `oak`), multiplexed over one WebSocket as
  `[1 stream-id byte] + [JPEG bytes]`. Streams: `0 head` (mono), `1
  gripper_left`, `2 gripper_right`, `3 head_stereo` (rectified left|right
  side by side, plus a `stereo_info` JSON calibration message).
- `server/whole_body.py` — `WholeBodyTeleop`: clutched relative-delta
  whole-body teleop (arms, lift retargeting, neck, base). Pure logic; tests
  in `tests/test_whole_body.py`.
- `server/lift_retarget.py`, `server/quat.py` — helpers.
- `server/teleop.py` — older `TeleopController` (IDLE/ARMS/BASE scheme),
  only used by the standalone dev-PC mode below.
- `static/main.js` — WebXR (three.js) page: sends head + controller poses
  every XR frame, draws the HUD, the gripper insets, and the ZED view. With
  stereo frames it shows each half to one eye at true scale (from
  `stereo_info`); otherwise the mono head screen.
- `tools/video_probe.py` — connect like the headset and report fps per stream.

## Controls (Quest Touch, from `server/whole_body.py`)
| input | action |
|---|---|
| A + X, tap | toggle teleop on/off (starts **off**; while off nothing moves) |
| A + X, hold 1.5 s | home both arms + neck, teleop off |
| grip (hold) | clutch that arm: hand motion moves it; release and it holds |
| index trigger | gripper close amount while clutched |
| B + Y | toggle base mode |
| left stick / right stick x | base vx, vy / yaw rate (base mode) |
| left stick click | recenter the neck's "straight ahead" |
| head | neck pan/tilt, always on |
| head height + arms | lift (only while an arm is clutched) |
| right stick up/down | stereo IPD (not in base mode); right stick click resets to the headset's measured IPD |

Constants (scales, speed limits, neck signs/offsets, lift) are in
`WholeBodyConfig`; several are marked VERIFY on hardware.

## Running

**Orin** (cameras), two terminals:
```bash
cd ~/yor_ws && python3 collect.py --task "..."           # owns the ZED + OAKs, records; publishes zed/view on :5551
#   (not recording: cd ~/yor_ws && python3 zed_rgbd.py --vr-only)
cd ~/yor_ws/vr-teleop && ./start_video.sh                # video server, port 8444
```
The video server doesn't open the ZED: `head_stereo` forwards the stereo JPEG
that collect.py (or `zed_rgbd.py --vr-only`) publishes (3840x1080, both eyes
1920x1080) byte for byte, so the ZED records color + depth at the same time. The mono `head` stream
is a stub (the headset hides it while stereo arrives). The two OAK-D-SR
wrist cams are opened by the video server. Start order doesn't matter.
Uses the `vr_teleop` micromamba env. Check with
`curl -sk https://localhost:8444/health`: per stream `fps` (0 when stalled),
`frames`, `error`, `stalls`. A camera that stops delivering for 3 s is
reopened automatically; one that fails to open retries every few seconds.

**Pi**: run `robot/whole_body_teleop.py` (YOR repo).

**Headset**, first time per cert: visit `https://192.168.123.117:8444/` and
`https://192.168.123.55:8443/` and accept the self-signed cert warnings. Then
open `https://192.168.123.55:8443` and click **Enter VR**. The status line
shows `control: open | video: NN fps | IPD ...`; the page reconnects by
itself if either server restarts.

Certs: `python server/gen_cert.py <machine-ip>` on each machine; re-run if
its IP changes. Open the port in the firewall if needed (`sudo ufw allow
8444/tcp` on the Orin, 8443 on the Pi).

### Network note
If connections work locally but not from the headset, check for client/AP
isolation on the router — some networks block device-to-device traffic on
the same subnet.

## Dev PC (legacy, single machine, no robot)
`python server/server.py --cameras` serves the page, streams a local webcam
as `head` (default config), and runs the old `TeleopController`, which only
logs commands. On Windows:
```powershell
python -m pip install -r requirements.txt
python server\gen_cert.py <this-pc-ip>
python server\server.py --cameras
```
Allow `python.exe` through Windows Defender Firewall (or
`New-NetFirewallRule -DisplayName "VR Teleop" -Direction Inbound -Protocol TCP -LocalPort 8443 -Action Allow`).

## Known limitations / next steps
- Video is JPEG over WebSocket. The stereo stream is the big one (960x540
  per eye, ~82 KB/frame, ~20 Mbit/s at 30 fps); watch `/health`
  `frames_dropped` and drop to VGA (672x376 per eye) if Wi-Fi can't keep up.
  WebRTC/H.264 on the Orin's hardware encoder is the eventual fix.
- Wrist cameras: one OAK-D-SR per gripper (`oak` source, picked by device
  id: left `18443010D1B3121300`, right `194430103195095A00`; each streams
  its `CAM_C` color camera, 640x400, 30 fps, ~22-34 KB/frame). The fisheye
  isn't streamed (would need a 4th screen in `main.js`).
- Total video is now ~36 Mbit/s (stereo ~92 KB + two OAKs ~22-34 KB per
  frame at 30 fps); watch `frames_dropped`.
- The ZED's 120 mm baseline is ~2x human eye spacing, so depth looks
  exaggerated.
- No auth on either server — fine on the robot LAN, nothing else.
