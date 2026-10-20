"""
Multi-stream camera broadcast: head camera (stand-in for the robot's ZED
camera) + left/right gripper cameras. Only the head stream is backed by
real hardware right now (this PC's webcam, index 0) -- the gripper streams
are placeholder frames until those cameras are wired up, so the rest of the
pipeline (multiplexed transport, client-side demux/render) can be built and
tested end to end today.

Wire protocol over the WebSocket's binary messages: each message is
  [1 byte stream id] + [JPEG bytes]
so a single connection carries all streams; the client demuxes by that
leading byte (see static/main.js).
"""
import asyncio
import logging
import threading
import time

import cv2
import numpy as np
from aiohttp import web

log = logging.getLogger("vr_teleop.cameras")

STREAM_IDS = {"head": 0, "gripper_left": 1, "gripper_right": 2}

HEAD_CAMERA_INDEX = 0
CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 480
HEAD_TARGET_FPS = 20
GRIPPER_STUB_FPS = 2  # placeholder frames are static, no need to push them fast
JPEG_QUALITY = 70


def _encode(frame) -> bytes:
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    return buf.tobytes() if ok else b""


def _make_stub_frame(label: str):
    frame = np.full((CAPTURE_HEIGHT, CAPTURE_WIDTH, 3), (30, 30, 30), dtype=np.uint8)
    cv2.putText(
        frame, "NO SIGNAL", (CAPTURE_WIDTH // 2 - 140, CAPTURE_HEIGHT // 2 - 10),
        cv2.FONT_HERSHEY_SIMPLEX, 1.1, (60, 60, 220), 2, cv2.LINE_AA,
    )
    cv2.putText(
        frame, label, (CAPTURE_WIDTH // 2 - 110, CAPTURE_HEIGHT // 2 + 30),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2, cv2.LINE_AA,
    )
    return frame


class MultiCameraStreamer:
    """One background thread per stream; each broadcasts
    [stream_id_byte + jpeg] to every connected client."""

    def __init__(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop
        self._clients: set[web.WebSocketResponse] = set()
        self._clients_lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = [
            threading.Thread(target=self._run_head, daemon=True),
            threading.Thread(target=self._run_stub, args=("gripper_left", "left gripper (stub)"), daemon=True),
            threading.Thread(target=self._run_stub, args=("gripper_right", "right gripper (stub)"), daemon=True),
        ]

    def add_client(self, ws: web.WebSocketResponse):
        with self._clients_lock:
            self._clients.add(ws)

    def remove_client(self, ws: web.WebSocketResponse):
        with self._clients_lock:
            self._clients.discard(ws)

    def start(self):
        for t in self._threads:
            t.start()

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)

    # ------------------------------------------------------------- workers
    def _run_head(self):
        stream_id = STREAM_IDS["head"]
        cap = cv2.VideoCapture(HEAD_CAMERA_INDEX, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
        if not cap.isOpened():
            log.error("Could not open head camera index %s -- falling back to stub frames", HEAD_CAMERA_INDEX)
            cap.release()
            self._run_stub("head", "head/ZED camera (stub, open failed)")
            return

        period = 1.0 / HEAD_TARGET_FPS
        log.info("Head camera %s opened, streaming at target %s fps", HEAD_CAMERA_INDEX, HEAD_TARGET_FPS)
        try:
            while not self._stop.is_set():
                t0 = time.time()
                ok, frame = cap.read()
                if not ok:
                    continue
                data = _encode(frame)
                if data:
                    self._broadcast(stream_id, data)
                dt = time.time() - t0
                if dt < period:
                    time.sleep(period - dt)
        finally:
            cap.release()

    def _run_stub(self, stream_name: str, label: str):
        stream_id = STREAM_IDS[stream_name]
        frame = _make_stub_frame(label)
        data = _encode(frame)
        period = 1.0 / GRIPPER_STUB_FPS
        log.info("Streaming stub frames for %s", stream_name)
        while not self._stop.is_set():
            if data:
                self._broadcast(stream_id, data)
            time.sleep(period)

    def _broadcast(self, stream_id: int, jpeg_bytes: bytes):
        payload = bytes([stream_id]) + jpeg_bytes
        with self._clients_lock:
            clients = list(self._clients)
        for ws in clients:
            asyncio.run_coroutine_threadsafe(self._safe_send(ws, payload), self._loop)

    @staticmethod
    async def _safe_send(ws: web.WebSocketResponse, data: bytes):
        try:
            if not ws.closed:
                await ws.send_bytes(data)
        except Exception:
            pass
