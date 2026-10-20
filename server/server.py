"""
VR teleop server:
  - Streams 3 camera feeds (head/ZED stand-in + left/right gripper stubs,
    see cameras.py) to any connected browser over one WebSocket, as
    multiplexed binary messages.
  - Serves a WebXR web page (static/) that renders those streams as
    head-locked screens inside an immersive-vr session.
  - Receives head pose + left/right controller poses from the headset
    over the same WebSocket (JSON text messages) and runs them through the
    teleop control/enable-state-machine (teleop.py) -- swap its dispatch
    stub for real robot calls once that interface exists.

Run:
    python server/gen_cert.py <this-pc-lan-ip>
    python server/server.py

Then, on the headset browser, visit:
    https://<this-pc-lan-ip>:8443
and accept the self-signed cert warning.
"""
import asyncio
import json
import logging
import ssl
from pathlib import Path

from aiohttp import web, WSMsgType

from cameras import MultiCameraStreamer
from teleop import TeleopController

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("vr_teleop")

ROOT = Path(__file__).resolve().parent.parent
STATIC_DIR = ROOT / "static"
CERT_PATH = ROOT / "certs" / "server.crt"
KEY_PATH = ROOT / "certs" / "server.key"

HOST = "0.0.0.0"
PORT = 8443


async def index(request):
    return web.FileResponse(STATIC_DIR / "index.html")


async def websocket_handler(request):
    ws = web.WebSocketResponse(max_msg_size=4 * 1024 * 1024)
    await ws.prepare(request)
    streamer: MultiCameraStreamer = request.app["streamer"]
    teleop: TeleopController = request.app["teleop"]
    streamer.add_client(ws)
    peer = request.remote
    log.info("Client connected: %s", peer)
    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                try:
                    payload = json.loads(msg.data)
                except ValueError:
                    continue
                commands = teleop.update(payload)
                status = {
                    "type": "teleop_state",
                    "state": commands["state"],
                    "arm_enabled": commands["arm_enabled"],
                    "warnings": commands["warnings"],
                }
                if not ws.closed:
                    await ws.send_str(json.dumps(status))
            elif msg.type == WSMsgType.ERROR:
                log.warning("WS error for %s: %s", peer, ws.exception())
    finally:
        streamer.remove_client(ws)
        log.info("Client disconnected: %s", peer)
    return ws


def make_app(loop):
    app = web.Application()
    app["streamer"] = MultiCameraStreamer(loop)
    app["streamer"].start()
    app["teleop"] = TeleopController()
    app.router.add_get("/", index)
    app.router.add_get("/ws", websocket_handler)
    app.router.add_static("/static/", STATIC_DIR, name="static")

    async def on_cleanup(app):
        app["streamer"].stop()

    app.on_cleanup.append(on_cleanup)
    return app


def main():
    if not CERT_PATH.exists() or not KEY_PATH.exists():
        log.error(
            "Missing TLS cert/key. Run: python server/gen_cert.py <this-pc-lan-ip> first."
        )
        return

    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_context.load_cert_chain(certfile=str(CERT_PATH), keyfile=str(KEY_PATH))

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    app = make_app(loop)

    log.info("Serving https://%s:%s  (use your LAN IP from a browser on the headset)", HOST, PORT)
    web.run_app(app, host=HOST, port=PORT, ssl_context=ssl_context, loop=loop)


if __name__ == "__main__":
    main()
