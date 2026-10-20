"""
Connect to a video server like the headset would and report frames per stream.

    python tools/video_probe.py 192.168.123.116:8444 [--seconds 5] [--save-dir /tmp/frames]

Exits non-zero if no frames arrive. Skips TLS verification (self-signed cert).
"""
import argparse
import asyncio
import ssl
import sys
import time
from pathlib import Path

import aiohttp

NAMES = {0: "head", 1: "gripper_left", 2: "gripper_right"}


async def probe(host, seconds, save_dir):
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    counts, sizes, saved = {}, {}, set()
    async with aiohttp.ClientSession() as session:
        async with session.get(f"https://{host}/health", ssl=ctx) as r:
            print("health:", await r.text())
        async with session.ws_connect(f"wss://{host}/ws", ssl=ctx, max_msg_size=8 * 1024 * 1024) as ws:
            end = time.time() + seconds
            while time.time() < end:
                try:
                    msg = await ws.receive(timeout=max(0.1, end - time.time()))
                except asyncio.TimeoutError:
                    break
                if msg.type != aiohttp.WSMsgType.BINARY or len(msg.data) < 2:
                    continue
                sid = msg.data[0]
                counts[sid] = counts.get(sid, 0) + 1
                sizes[sid] = sizes.get(sid, 0) + len(msg.data) - 1
                if save_dir and sid not in saved and msg.data[1:3] == b"\xff\xd8":
                    path = Path(save_dir) / f"{NAMES.get(sid, sid)}.jpg"
                    path.write_bytes(msg.data[1:])
                    saved.add(sid)
                    print("saved", path)
    for sid in sorted(set(NAMES) | set(counts)):
        n = counts.get(sid, 0)
        kb = sizes.get(sid, 0) / max(n, 1) / 1024
        print(f"{NAMES.get(sid, sid):14s} {n / seconds:5.1f} fps  avg {kb:5.1f} KB/frame")
    return sum(counts.values())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("host", help="host:port of the video server")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--save-dir", help="save the first JPEG of each stream here")
    args = ap.parse_args()
    if args.save_dir:
        Path(args.save_dir).mkdir(parents=True, exist_ok=True)
    total = asyncio.run(probe(args.host, args.seconds, args.save_dir))
    sys.exit(0 if total else 1)


if __name__ == "__main__":
    main()
