"""
VR teleop server:
  - Serves a WebXR web page (static/) for the headset.
  - Receives head pose + left/right controller poses from the headset over a
    WebSocket (JSON text messages) and hands each one to a payload handler.
  - Sends a small JSON status message back for the headset HUD.
  - Optionally (--cameras / cameras=True) streams camera feeds to the headset
    (see cameras.py). Off by default: the robot-side Pi publishes no video,
    and with cameras off nothing here imports OpenCV.

Run standalone:
    python server/gen_cert.py <this-machine-lan-ip>
    python server/server.py [--cameras]            # dev PC: old TeleopController logging
    python server/server.py --video-only --port 8444 --camera-config cams.json
                                                   # camera machine (Orin), see docs/ORIN_VIDEO_SERVER.md
GET /health returns JSON with connected clients and per-stream fps.
GET /config.json returns {"video": <default video host:port or null>}; the page
uses it when its URL has no ?video= (pass default_video= to run()).

From the robot, use ../robot/whole_body_teleop.py, which calls run() with its
own handler.

Then, on the headset browser, visit:
    https://<lan-ip>:8443
and accept the self-signed cert warning.
"""
import argparse
import asyncio
import json
import logging
import ssl
from pathlib import Path

from aiohttp import web, WSMsgType

log = logging.getLogger("vr_teleop")

ROOT = Path(__file__).resolve().parent.parent
STATIC_DIR = ROOT / "static"
CERT_PATH = ROOT / "certs" / "server.crt"
KEY_PATH = ROOT / "certs" / "server.key"

HOST = "0.0.0.0"
PORT = 8443


async def index(request):
    if request.app["on_payload"] is None:
        # Video-only server: nothing to render here. Visiting this page on the
        # headset is how its self-signed cert gets accepted.
        return web.Response(text="vr-teleop video server OK (cert accepted). See /health.\n")
    return web.FileResponse(STATIC_DIR / "index.html")


async def health(request):
    streamer = request.app["streamer"]
    body = {"clients": request.app["counters"]["clients"], "cameras": streamer is not None}
    if streamer is not None:
        body["streams"] = streamer.stats
        body["frames_dropped"] = streamer.stats_dropped
        body["stereo_info"] = streamer.stereo_info
    return web.json_response(body)


async def page_config(request):
    return web.json_response({"video": request.app["default_video"]})


async def websocket_handler(request):
    ws = web.WebSocketResponse(max_msg_size=4 * 1024 * 1024)
    await ws.prepare(request)
    streamer = request.app["streamer"]
    on_payload = request.app["on_payload"]
    on_connect = request.app["on_connect"]
    on_disconnect = request.app["on_disconnect"]
    if streamer is not None:
        streamer.add_client(ws)
    peer = request.remote
    request.app["counters"]["clients"] += 1
    log.info("Client connected: %s", peer)
    if on_connect:
        on_connect(peer)
    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                try:
                    payload = json.loads(msg.data)
                except ValueError:
                    continue
                if on_payload is None:
                    continue  # video-only server: ignore pose messages
                status = on_payload(payload)
                if status is not None and not ws.closed:
                    await ws.send_str(json.dumps(status))
            elif msg.type == WSMsgType.ERROR:
                log.warning("WS error for %s: %s", peer, ws.exception())
    finally:
        if streamer is not None:
            streamer.remove_client(ws)
        request.app["counters"]["clients"] -= 1
        log.info("Client disconnected: %s", peer)
        if on_disconnect:
            on_disconnect(peer)
    return ws


def make_app(loop, on_payload, on_connect=None, on_disconnect=None, cameras=False, camera_config=None,
             default_video=None):
    """on_payload(payload_dict) -> status dict for the HUD (or None). Called on
    the event loop for every headset message, so it must not block. Pass
    on_payload=None for a video-only server."""
    app = web.Application()
    app["streamer"] = None
    app["default_video"] = default_video
    app["counters"] = {"clients": 0}  # mutable: app state is frozen once started
    if cameras:
        from cameras import MultiCameraStreamer, load_config  # imports OpenCV; only when asked for

        app["streamer"] = MultiCameraStreamer(loop, load_config(camera_config))
        app["streamer"].start()
    app["on_payload"] = on_payload
    app["on_connect"] = on_connect
    app["on_disconnect"] = on_disconnect
    app.router.add_get("/", index)
    app.router.add_get("/ws", websocket_handler)
    app.router.add_get("/health", health)
    app.router.add_get("/config.json", page_config)
    app.router.add_static("/static/", STATIC_DIR, name="static")

    async def on_cleanup(app):
        if app["streamer"] is not None:
            app["streamer"].stop()

    app.on_cleanup.append(on_cleanup)
    return app


def run(on_payload, on_connect=None, on_disconnect=None, cameras=False, camera_config=None,
        default_video=None, host=HOST, port=PORT, cert_path=CERT_PATH, key_path=KEY_PATH):
    """Serve until Ctrl-C. Blocks."""
    if not Path(cert_path).exists() or not Path(key_path).exists():
        raise FileNotFoundError(f"Missing TLS cert/key ({cert_path}, {key_path})")

    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_context.load_cert_chain(certfile=str(cert_path), keyfile=str(key_path))

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    app = make_app(loop, on_payload, on_connect, on_disconnect, cameras=cameras, camera_config=camera_config,
                   default_video=default_video)

    log.info("Serving https://%s:%s  (cameras %s)", host, port, "on" if cameras else "off")
    web.run_app(app, host=host, port=port, ssl_context=ssl_context, loop=loop, print=None)


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    parser = argparse.ArgumentParser()
    parser.add_argument("--cameras", action="store_true", help="stream camera feeds to the headset")
    parser.add_argument("--video-only", action="store_true",
                        help="camera server only (implies --cameras); ignores pose data, sends no HUD")
    parser.add_argument("--camera-config", help="JSON file mapping streams to sources (see cameras.py)")
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    if not CERT_PATH.exists() or not KEY_PATH.exists():
        log.error("Missing TLS cert/key. Run: python server/gen_cert.py <this-pc-lan-ip> first.")
        return

    if args.video_only:
        run(None, cameras=True, camera_config=args.camera_config, port=args.port)
        return

    from teleop import TeleopController

    teleop = TeleopController()

    def on_payload(payload):
        commands = teleop.update(payload)
        return {
            "type": "teleop_state",
            "state": commands["state"],
            "arm_enabled": commands["arm_enabled"],
            "warnings": commands["warnings"],
        }

    run(on_payload, cameras=args.cameras, camera_config=args.camera_config, port=args.port)


if __name__ == "__main__":
    main()
