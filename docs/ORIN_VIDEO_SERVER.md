# Orin video server: guide for the agent on the Orin

You are setting up the **camera side** of YOR's VR teleop on the Jetson Orin.
Read this whole file before changing anything. `README.md` and `AGENTS.md` in
this repo describe the older single-machine setup; where they disagree with
this file, this file wins for the Orin.

## The system you are part of

```
Quest headset (WebXR page)
 ├─ wss://192.168.123.55:8443/ws    poses up, HUD down   -> Pi: control -> robot (CAN, neck, lift, base)
 └─ wss://192.168.123.117:8444/ws   video down           -> Orin (you): ZED + wrist cams
```

- **Pi** (`192.168.123.55`, Wi-Fi): runs `robot/whole_body_teleop.py` from the
  YOR repo. It serves the WebXR page, receives headset poses, and drives the
  robot. **It publishes no video and never will.** It is already working.
- **Orin** (`192.168.123.117`): has the cameras, not the robot. Your job is to
  run `server/server.py --video-only` here so the headset shows the camera
  feeds. **Do not send robot commands or handle poses from the Orin.** Pose
  messages sent to the video server are ignored on purpose.
- **Headset**: opens `https://192.168.123.55:8443`. The Pi tells the page
  where your server is (default `192.168.123.117:8444`, via `/config.json`);
  `?video=<host:port>` in the URL overrides it, `?video=none` turns video off.

## Contract you must keep

These are shared with `static/main.js` (served by the Pi). If you change any
of them, the Pi side has to change too, so don't, unless the user asks:

- **Port** `8444`, HTTPS/WSS with a self-signed cert, WebSocket at `/ws`.
- **Wire format**: every binary WebSocket message is `[1 byte stream id] + [JPEG bytes]`.
- **Stream ids**: `0 = head` (ZED, left eye, mono fallback), `1 = gripper_left`,
  `2 = gripper_right` (wrist cameras), `3 = head_stereo` (see "Stereo head view"). They are defined in `server/cameras.py` `STREAM_IDS` and in
  `static/main.js` `STREAMS`.
- The client draws each JPEG into a 640x480 canvas (any size is scaled), so
  640x480 or smaller keeps bandwidth down.
- Text messages from clients are ignored. Your server sends none.
- `GET /health` returns JSON with the client count and per-stream `source`,
  `fps`, `frames`, `error`.

## Stereo head view (added 2026-10-20)

The headset now shows the ZED in stereo: left image to the left eye, right
image to the right eye, at true scale. This adds to the contract above:

- **Stream id 3, `head_stereo`**: one JPEG per frame with the **rectified**
  left and right images **side by side** (`[left | right]`, total width =
  2 x per-eye width). Both halves must come from the same `grab()`; never
  pair frames from separate captures. pyzed: `retrieve_image(mat, sl.VIEW.SIDE_BY_SIDE)`.
- **`stereo_info` text message**: sent to every client on connect, and again
  if it changes:
  `{"type": "stereo_info", "stream": 3, "width", "height", "fx", "fy", "cx", "cy", "baseline_m", "model"}`.
  `width`/`height` are **per eye** in pixels as streamed, and fx/fy/cx/cy are
  the rectified left-camera intrinsics **scaled to that size**, not the
  native resolution. The headset sizes the view from these, so wrong values
  make everything look too big or too small. Also exposed in `/health`.
- `server/cameras.py` on the `pi` branch implements this
  (`{"type": "zed", "view": "side_by_side"}` for the `head_stereo` stream;
  see `camera_config.example.json`). If you already changed `cameras.py`
  on the Orin, merge this in rather than replacing your version, and keep
  your approach to camera ownership.
- The ZED can only be opened once, so don't configure `head` and
  `head_stereo` both as `zed` sources. Set `head` to `stub` (the headset
  hides the mono screen while stereo frames arrive), or derive mono `head`
  from the left half of the same capture if you want a fallback.
- Suggested size: 960x540 per eye (1920x540 frame), 30 fps, JPEG quality ~75.
  That's the biggest stream by far: watch `/health` fps and `frames_dropped`,
  and drop to 672x376 per eye (ZED VGA) if the Wi-Fi can't keep up.
- The robot's ZED is a **ZED 2 / 2i (120 mm baseline)**, about twice human
  eye spacing, so operators will see depth exaggerated. That's expected; the
  headset's IPD adjustment only moves the comfortable fusion point.

Check: `python tools/video_probe.py localhost:8444` prints the `stereo_info`
message and a `head_stereo` fps line; `--save-dir` saves a frame whose two
halves should look nearly identical except for a small horizontal shift.

## Hardware state (as of 2026-10-20)

- Head camera: ZED 2i (S/N 30496273), ZED SDK 5.2.3 on the Orin, on the 2-DOF
  neck, which rides on the lift.
- Wrist cameras: one **OAK-D-SR** per gripper (Luxonis, two OV9782 color
  cameras `CAM_B`/`CAM_C`, no `CAM_A`; USB 2). Not UVC devices, so they use
  the `oak` source (depthai v3), selected by **device id** (USB ports move
  between boots): `gripper_left` = `18443010D1B3121300`, `gripper_right` =
  `194430103195095A00`, each `CAM_C` at 1280x800 (full sensor) @ 30, JPEG made on the
  camera (`"encode": "device"`, default): ~75 KB/frame at quality 50, ~4.5 MB/s for both,
  fits USB 2. The old raw-BGR path (`"encode": "host"`) caps USB 2 at 640x400 @ 30.
  Quality 70 (~142 KB/frame) overran the headset's Wi-Fi link (frames dropped) on 2026-10-21. Left/right was mapped from
  which camera was on the right gripper first; confirm by covering one lens.
  depthai 3.11.0 is the `~/.local` install, symlinked into the `vr_teleop`
  env (`depthai.cpython-310-*.so` + `depthai.libs`); `~/depthai-core` is the
  matching source/examples. First boot takes ~5 s each (the source raises
  `DEPTHAI_BOOTUP_TIMEOUT`); OAKs are opened one at a time because two
  booting at once fail with `X_LINK_DEVICE_ALREADY_IN_USE`.
  `pipeline.stop()` takes ~3 s on shutdown.
- There is also a fisheye camera. It is **not** part of the headset streams
  yet; adding a 4th stream means changing `main.js` too, so leave it out.

## Quick start (Orin, already set up)

```bash
cd ~/yor_ws/vr-teleop && ./start_video.sh   # Ctrl-C to stop
```
Needs `python3 ~/yor_ws/zed_rgbd.py --publish --stereo` running too (any
order): it owns the ZED and publishes `zed/view` on 127.0.0.1:5551 (left
color JPEG + stereo [left|right] JPEG + `stereo_info`, no depth). This is
approach 2 below. `camera_config.orin.json`: `head` stub (duplicate of the
stereo left eye, ~30 Mbit/s saved), `head_stereo` = `commlink` source
forwarding `stereo_jpeg` byte for byte (calibration relayed from the packets),
`gripper_left`/`gripper_right` = the two OAK-D-SRs (opened by the video
server). The `commlink`, `pyzmq` and `lz4` packages are symlinked into the
`vr_teleop` env from `~/.local` / system dist-packages, like depthai.

Watchdog (cameras.py): an open source with no frame for 3 s (`STALL_S`) is
closed and reopened (`stalls` in `/health`); a failed open streams stub
frames and retries after 2 s, then every 5 s (`RETRY_S`). `/health` fps
updates every second, so a stalled stream reads 0. Measured 2026-10-21: ZED
stereo ~15-17 fps (the rate `zed_rgbd.py` delivers, ~350-400 KB/frame), OAKs
30 fps each.

## Most important constraint: the ZED can only be opened by one process

The Orin already runs an **episode recorder** (in the nav-policy repo). It
records the 4 camera streams plus ZED depth, keyed on `episode_id` from the
Pi's `episode/signal` ZMQ topic on port 5558. The Orin also publishes ZED
pose on port 6000 for the Pi.

A ZED camera (and usually a V4L2 device) can be held by only one process. So
before anything else, find out what owns each camera right now:

```bash
ps aux | grep -i -E "zed|record|collect|pose" | grep -v grep
sudo fuser -v /dev/video* 2>/dev/null
ls -l /dev/v4l/by-id/
```

Then pick one approach and **confirm it with the user before you build it**:

1. **Video server opens the cameras itself** (`"type": "zed"` / `"opencv"` in
   the config). Simplest, but only works when nothing else holds them (the
   recorder and pose publisher are stopped, or don't use those cameras).
2. **The process that already owns the cameras shares frames.** It publishes
   small JPEGs (or raw frames) locally, e.g. ZMQ PUB on `ipc://` or
   `tcp://127.0.0.1`, and you add a source type to `cameras.py` that
   subscribes. This keeps recording and teleop video running at the same
   time, which is what data collection needs, so it is probably the right
   long-term answer. Don't make the recorder slower: encode on its side only
   if it has spare time, otherwise send raw frames and encode here.
3. **Run the video streamer inside the recorder process.** Fewest copies, but
   ties the two together. Only do this if the user prefers it.

If the ZED pose publisher on port 6000 uses the same ZED, stopping it breaks
the Pi's base navigation. Never stop it without asking.

## Setup steps

1. **Get the code.** The repo is `https://github.com/adityx23/vr-teleop`. The
   Orin-side changes (`--video-only`, camera config, `/health`,
   `tools/video_probe.py`) may only exist on the Pi until they're pushed. If
   your clone lacks `camera_config.example.json`, ask the user to push or
   copy the repo over.
2. **Python env**, separate from the recorder's if it uses one:
   ```bash
   python -m venv ~/vr-teleop-venv && source ~/vr-teleop-venv/bin/activate
   pip install aiohttp opencv-python cryptography
   ```
   For the ZED source, the `pyzed` wheel must match the installed ZED SDK
   (`python /usr/local/zed/get_python_api.py` installs it). Check with
   `python -c "import pyzed.sl as sl; print(sl.Camera.get_sdk_version())"`.
   On Jetson, pip's `opencv-python` is fine for JPEG encoding; switch to
   hardware JPEG (nvjpeg / GStreamer `nvjpegenc`) only if `/health` shows
   CPU-bound fps.
3. **Camera config.** Copy `camera_config.example.json` to `cams.json` and
   fill in real devices. Use stable `/dev/v4l/by-id/...` paths for USB
   cameras, never bare `/dev/videoN` (numbers change between boots). Find
   which by-id path is which wrist by unplugging one camera at a time.
   Never guess left/right from the device order: on this robot left/right
   has been mixed up before.
4. **Cert** for the Orin's IP:
   ```bash
   python server/gen_cert.py 192.168.123.117
   ```
5. **Run:**
   ```bash
   python server/server.py --video-only --port 8444 --camera-config cams.json
   ```
   Make sure the firewall allows TCP 8444 (`sudo ufw status`, if ufw is on).

## How to verify (in this order)

1. On the Orin: `curl -sk https://localhost:8444/health`. Every stream should
   show a real `source` (not `stub`) and `fps` near the target, with
   `error: null`.
2. On the Orin: `python tools/video_probe.py localhost:8444 --save-dir /tmp/frames`.
   Look at the saved JPEGs: right camera in the right slot, image not
   upside-down, sensible exposure.
3. From the Pi (or another LAN machine): `python tools/video_probe.py 192.168.123.117:8444`.
   This proves it's reachable over the network. Compare fps with step 2.
4. On the headset, **first visit `https://192.168.123.117:8444/` directly** and
   accept the cert warning. It should then say "vr-teleop video server OK".
   Use
   `https://`: plain `http://` gives `ERR_EMPTY_RESPONSE`. Without this step
   the video socket fails silently.
5. Then open `https://192.168.123.55:8443/?video=192.168.123.117:8444`. The
   status line on the page says `control: open | video: NN fps`. In VR, the
   HUD shows `NO VIDEO` while frames are missing.

## Targets and what to report back

- Head (ZED): 20–30 fps at 640x480, JPEG quality ~70, under ~60 KB/frame.
  Lower quality or resolution before lowering fps.
- Wrist cams: 15–30 fps.
- Total bandwidth stays well under the Wi-Fi link (the Pi is on the same
  network, and its control traffic must not suffer). With 3 streams at 30 fps
  and ~50 KB, that's ~36 Mbit/s; if the Pi's teleop shows lag, cut fps first.
- The server already drops frames for a slow client instead of queueing them
  (`frames_dropped` in `/health`). Rising drops mean the link is saturated.

When done, tell the user: which approach (1/2/3) you used, the final
`cams.json`, `/health` output, probe fps from the Pi, and anything you changed
in this repo.

## Don'ts

- Don't open or stop cameras that the recorder or the ZED pose publisher are
  using without asking.
- Don't change the wire format, stream ids or port (see the contract above).
- Don't add pose handling or robot control on the Orin.
- Don't use bare `/dev/videoN` paths in the config.
- The server has no auth. Fine on the robot LAN, never expose it beyond that.

## Later (not now)

- WebRTC / H.264 with the Orin's hardware encoder for lower latency and bandwidth.
- Fisheye as a 4th stream (needs a `main.js` layout change).
- Stereo ZED view (both eyes) for depth perception in the headset.
