# Orin video server: guide for the agent on the Orin

You are setting up the **camera side** of YOR's VR teleop on the Jetson Orin.
Read this whole file before changing anything. `README.md` and `AGENTS.md` in
this repo describe the older single-machine setup; where they disagree with
this file, this file wins for the Orin.

## The system you are part of

```
Quest headset (WebXR page)
 ├─ wss://192.168.123.55:8443/ws    poses up, HUD down   -> Pi: control -> robot (CAN, neck, lift, base)
 └─ wss://192.168.123.116:8444/ws   video down           -> Orin (you): ZED + wrist cams
```

- **Pi** (`192.168.123.55`, Wi-Fi): runs `robot/whole_body_teleop.py` from the
  YOR repo. It serves the WebXR page, receives headset poses, and drives the
  robot. **It publishes no video and never will.** It is already working.
- **Orin** (`192.168.123.116`): has the cameras, not the robot. Your job is to
  run `server/server.py --video-only` here so the headset shows the camera
  feeds. **Do not send robot commands or handle poses from the Orin.** Pose
  messages sent to the video server are ignored on purpose.
- **Headset**: opens `https://192.168.123.55:8443/?video=192.168.123.116:8444`.
  The page's `?video=` parameter tells it where your server is.

## Contract you must keep

These are shared with `static/main.js` (served by the Pi). If you change any
of them, the Pi side has to change too, so don't, unless the user asks:

- **Port** `8444`, HTTPS/WSS with a self-signed cert, WebSocket at `/ws`.
- **Wire format**: every binary WebSocket message is `[1 byte stream id] + [JPEG bytes]`.
- **Stream ids**: `0 = head` (ZED, left eye), `1 = gripper_left`, `2 = gripper_right`
  (wrist cameras). They are defined in `server/cameras.py` `STREAM_IDS` and in
  `static/main.js` `STREAMS`.
- The client draws each JPEG into a 640x480 canvas (any size is scaled), so
  640x480 or smaller keeps bandwidth down.
- Text messages from clients are ignored. Your server sends none.
- `GET /health` returns JSON with the client count and per-stream `source`,
  `fps`, `frames`, `error`.

## Hardware state (as of 2026-10-20)

- Head camera: ZED (SDK 4.2 era), on the 2-DOF neck, which rides on the lift.
- Wrist cameras: one per arm. **The left gripper is removed.** Ask the user
  whether a left wrist camera is still mounted; if not, leave `gripper_left`
  as a stub.
- There is also a fisheye camera. It is **not** part of the headset streams
  yet; adding a 4th stream means changing `main.js` too, so leave it out.

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
   python server/gen_cert.py 192.168.123.116
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
3. From the Pi (or another LAN machine): `python tools/video_probe.py 192.168.123.116:8444`.
   This proves it's reachable over the network. Compare fps with step 2.
4. On the headset, **first visit `https://192.168.123.116:8444/` directly** and
   accept the cert warning. It should then say "vr-teleop video server OK".
   Use
   `https://`: plain `http://` gives `ERR_EMPTY_RESPONSE`. Without this step
   the video socket fails silently.
5. Then open `https://192.168.123.55:8443/?video=192.168.123.116:8444`. The
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
