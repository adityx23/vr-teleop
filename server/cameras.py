"""
Multi-stream camera broadcast to the headset. Runs on the camera machine (the
Orin) via `server.py --video-only`; the robot Pi publishes no video.

Wire protocol over the WebSocket's binary messages: each message is
  [1 byte stream id] + [JPEG bytes]
so a single connection carries all streams; the client demuxes by that
leading byte (see static/main.js -- STREAMS there must match STREAM_IDS here).

Each stream is fed by a source picked in a JSON config (see
camera_config.example.json and docs/ORIN_VIDEO_SERVER.md):
  {"type": "stub"}                                      placeholder "NO SIGNAL" frames
  {"type": "opencv", "device": "/dev/video0" | 0}       V4L2 webcam / USB camera
  {"type": "zed", "serial": null, "view": "left"}       ZED via pyzed (opens the camera itself)
  {"type": "zed", "view": "side_by_side"}               ZED stereo: rectified left|right in one
                                                        frame, for the "head_stereo" stream
Common options: "width", "height", "fps", "jpeg_quality". For side_by_side,
width/height are per eye (the frame is 2*width wide).
A source that fails to open falls back to stub frames, labelled with the reason.

Stereo: a "head_stereo" source also produces calibration for the frames it
sends, which every client receives as a JSON text message
  {"type": "stereo_info", "stream": 3, "width", "height" (per eye, px),
   "fx", "fy", "cx", "cy" (px, at that size), "baseline_m", "model"}
on connect (and again if it changes). The headset uses it to show the view
at true scale, left image to the left eye and right image to the right eye.
"""
import asyncio
import json
import logging
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

log = logging.getLogger("vr_teleop.cameras")

STREAM_IDS = {"head": 0, "gripper_left": 1, "gripper_right": 2, "head_stereo": 3}

DEFAULTS = {"width": 640, "height": 480, "fps": 20, "jpeg_quality": 70}
STUB_FPS = 2  # placeholder frames are static, no need to push them fast

# Used when no config file is given: head = first V4L2/DirectShow camera. The headset
# page no longer shows gripper cameras, so they are off unless a config turns them on.
DEFAULT_CONFIG = {
    "head": {"type": "opencv", "device": 0},
    "gripper_left": {"type": "none"},
    "gripper_right": {"type": "none"},
    "head_stereo": {"type": "none"},
}


def load_config(path=None):
    if path is None:
        return dict(DEFAULT_CONFIG)
    cfg = json.loads(Path(path).read_text())
    unknown = set(cfg) - set(STREAM_IDS)
    if unknown:
        raise ValueError(f"unknown stream(s) {sorted(unknown)}; valid: {sorted(STREAM_IDS)}")
    # Only "head" falls back to a stub; absent head_stereo / gripper streams are off.
    return {name: cfg.get(name, {"type": "stub" if name == "head" else "none"}) for name in STREAM_IDS}


def _encode(frame, quality) -> bytes:
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    return buf.tobytes() if ok else b""


def _make_stub_frame(label: str, width: int, height: int):
    frame = np.full((height, width, 3), (30, 30, 30), dtype=np.uint8)
    cv2.putText(frame, "NO SIGNAL", (width // 2 - 140, height // 2 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (60, 60, 220), 2, cv2.LINE_AA)
    cv2.putText(frame, label[:40], (20, height // 2 + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1, cv2.LINE_AA)
    return frame


# ------------------------------------------------------------------ sources
class OpenCVSource:
    def __init__(self, opts):
        device = opts.get("device", 0)
        backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2
        self.cap = cv2.VideoCapture(device, backend)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, opts["width"])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, opts["height"])
        if "fourcc" in opts:  # e.g. "MJPG" for USB cams that only do high fps compressed
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*opts["fourcc"]))
        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError(f"could not open {device}")
        self.desc = f"opencv:{device}"

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self):
        self.cap.release()


class ZedSource:
    """Opens the ZED itself. Only one process can open a ZED: if another
    process (e.g. the episode recorder) already owns it, this fails and the
    stream falls back to stub -- see docs/ORIN_VIDEO_SERVER.md."""

    def __init__(self, opts):
        import pyzed.sl as sl

        self.sl = sl
        init = sl.InitParameters()
        init.camera_resolution = getattr(sl.RESOLUTION, opts.get("resolution", "HD720"))
        init.camera_fps = int(opts["fps"])
        init.depth_mode = sl.DEPTH_MODE.NONE
        if opts.get("serial"):
            init.set_from_serial_number(int(opts["serial"]))
        self.cam = sl.Camera()
        err = self.cam.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED open failed: {err}")
        view = opts.get("view", "left")
        self.stereo = view == "side_by_side"
        self.view = {"left": sl.VIEW.LEFT, "right": sl.VIEW.RIGHT, "side_by_side": sl.VIEW.SIDE_BY_SIDE}[view]
        self.mat = sl.Mat()
        w, h = int(opts["width"]), int(opts["height"])
        self.size = (2 * w, h) if self.stereo else (w, h)
        self.desc = f"zed:{opts.get('serial') or 'default'}:{view}"
        self.stereo_info = self._stereo_info(w, h) if self.stereo else None

    def _stereo_info(self, w, h):
        """Rectified left-camera intrinsics scaled to the streamed per-eye size."""
        info = self.cam.get_camera_information()
        cfg = getattr(info, "camera_configuration", info)  # SDK 4 nests it; SDK 3 doesn't
        calib = cfg.calibration_parameters
        left = calib.left_cam
        native_w = getattr(cfg, "resolution", None)
        native_w, native_h = (native_w.width, native_w.height) if native_w is not None else (left.image_size.width,
                                                                                               left.image_size.height)
        sx, sy = w / native_w, h / native_h
        try:
            baseline = calib.get_camera_baseline()  # in InitParameters.coordinate_units (default mm)
        except AttributeError:
            baseline = abs(calib.stereo_transform.get_translation().get()[0])
        baseline_m = baseline / 1000.0 if baseline > 1.0 else baseline
        return {
            "type": "stereo_info", "stream": STREAM_IDS["head_stereo"], "width": w, "height": h,
            "fx": left.fx * sx, "fy": left.fy * sy, "cx": left.cx * sx, "cy": left.cy * sy,
            "baseline_m": baseline_m, "model": str(getattr(info, "camera_model", "ZED")),
        }

    def read(self):
        if self.cam.grab() != self.sl.ERROR_CODE.SUCCESS:
            return None
        self.cam.retrieve_image(self.mat, self.view)
        frame = cv2.cvtColor(self.mat.get_data(), cv2.COLOR_BGRA2BGR)
        return cv2.resize(frame, self.size)

    def close(self):
        self.cam.close()


SOURCE_TYPES = {"opencv": OpenCVSource, "zed": ZedSource}


# ----------------------------------------------------------------- streamer
class MultiCameraStreamer:
    """One background thread per stream; each broadcasts
    [stream_id_byte + jpeg] to every connected client."""

    def __init__(self, loop: asyncio.AbstractEventLoop, config=None):
        self._loop = loop
        self._clients = set()
        self._clients_lock = threading.Lock()
        self._stop = threading.Event()
        self.config = config or dict(DEFAULT_CONFIG)
        self.stats = {name: {"source": None, "frames": 0, "fps": 0.0, "error": None} for name in STREAM_IDS}
        self.stats_dropped = 0
        self._pending = {}  # (id(ws), stream_id) -> future of the in-flight send
        self.stereo_info = None  # set once a head_stereo source opens
        self._threads = [
            threading.Thread(target=self._run, args=(name, {**DEFAULTS, **self.config.get(name, {})}), daemon=True)
            for name in STREAM_IDS
        ]

    def add_client(self, ws):
        with self._clients_lock:
            self._clients.add(ws)
            info = self.stereo_info
        if info is not None:
            asyncio.run_coroutine_threadsafe(self._safe_send_text(ws, json.dumps(info)), self._loop)

    def remove_client(self, ws):
        with self._clients_lock:
            self._clients.discard(ws)
            for key in [k for k in self._pending if k[0] == id(ws)]:
                del self._pending[key]

    def client_count(self):
        with self._clients_lock:
            return len(self._clients)

    def start(self):
        for t in self._threads:
            t.start()

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)

    # ------------------------------------------------------------- workers
    def _run(self, name, opts):
        kind = opts.get("type", "stub")
        if kind == "none":
            self.stats[name]["source"] = "none"
            return
        if kind == "stub":
            self._run_stub(name, opts, f"{name} (stub)")
            return
        try:
            source = SOURCE_TYPES[kind](opts)
        except Exception as e:  # noqa: BLE001 -- any failure -> visible stub, keep other streams alive
            log.error("%s: %s source failed (%s) -- streaming stub frames", name, kind, e)
            self.stats[name]["error"] = str(e)
            self._run_stub(name, opts, f"{name}: {kind} failed")
            return

        self.stats[name]["source"] = source.desc
        log.info("%s: %s opened, target %s fps", name, source.desc, opts["fps"])
        info = getattr(source, "stereo_info", None)
        if info is not None:
            log.info("%s: stereo calibration %s", name, info)
            self.stereo_info = info
            self._broadcast_text(json.dumps(info))
        period = 1.0 / opts["fps"]
        window_start, window_frames = time.time(), 0
        try:
            while not self._stop.is_set():
                t0 = time.time()
                frame = source.read()
                if frame is None:
                    time.sleep(0.01)
                    continue
                data = _encode(frame, opts["jpeg_quality"])
                if data:
                    self._broadcast(STREAM_IDS[name], data)
                    self.stats[name]["frames"] += 1
                    window_frames += 1
                if t0 - window_start >= 1.0:
                    self.stats[name]["fps"] = round(window_frames / (t0 - window_start), 1)
                    window_start, window_frames = t0, 0
                dt = time.time() - t0
                if dt < period:
                    time.sleep(period - dt)
        finally:
            source.close()

    def _run_stub(self, name, opts, label):
        self.stats[name]["source"] = self.stats[name]["source"] or "stub"
        data = _encode(_make_stub_frame(label, opts["width"], opts["height"]), opts["jpeg_quality"])
        while not self._stop.is_set():
            if data:
                self._broadcast(STREAM_IDS[name], data)
                self.stats[name]["frames"] += 1
            self.stats[name]["fps"] = float(STUB_FPS)
            time.sleep(1.0 / STUB_FPS)

    def _broadcast(self, stream_id: int, jpeg_bytes: bytes):
        payload = bytes([stream_id]) + jpeg_bytes
        with self._clients_lock:
            for ws in self._clients:
                # Drop this frame for a client whose previous frame on this stream is
                # still sending, so a slow link shows fewer frames instead of growing lag.
                key = (id(ws), stream_id)
                pending = self._pending.get(key)
                if pending is not None and not pending.done():
                    self.stats_dropped += 1
                    continue
                self._pending[key] = asyncio.run_coroutine_threadsafe(self._safe_send(ws, payload), self._loop)

    def _broadcast_text(self, text: str):
        with self._clients_lock:
            clients = list(self._clients)
        for ws in clients:
            asyncio.run_coroutine_threadsafe(self._safe_send_text(ws, text), self._loop)

    @staticmethod
    async def _safe_send_text(ws, text: str):
        try:
            if not ws.closed:
                await ws.send_str(text)
        except Exception:  # noqa: BLE001
            pass

    @staticmethod
    async def _safe_send(ws, data: bytes):
        try:
            if not ws.closed:
                await ws.send_bytes(data)
        except Exception:  # noqa: BLE001
            pass
