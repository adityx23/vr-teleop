#!/usr/bin/env bash
# Start the Orin video server for the VR headset: ZED stereo only (stream 3); the OAK wrist cams are not streamed.
# The ZED stream (per-eye 1920x1080 @ 30, commlink topic zed/view on 127.0.0.1:5551) comes from whichever
# process owns the ZED -- only one can:
#   recording:      cd ~/yor_ws && python3 collect.py --task "..."     (records all cameras + publishes zed/view)
#   not recording:  cd ~/yor_ws && python3 zed_rgbd.py --vr-only
#   ./start_video.sh                 uses camera_config.orin.json
#   ./start_video.sh my_cams.json    uses another camera config
# Ctrl-C to stop. The headset page reconnects by itself whenever this restarts.
set -euo pipefail
cd "$(dirname "$0")"

PY="${VR_TELEOP_PYTHON:-$HOME/micromamba/envs/vr_teleop/bin/python}"
CONFIG="${1:-camera_config.orin.json}"
PORT=8444
PI_IP=192.168.123.55
export PYTHONNOUSERSITE=1  # ignore ~/.local (its numpy 2 breaks things on the Orin)

if [ ! -f certs/server.crt ]; then
  echo "No TLS cert yet. Run: $PY server/gen_cert.py <this-orin-ip>" >&2
  exit 1
fi

IP="$(hostname -I | awk '{print $1}')"
echo "Video server on https://$IP:$PORT  (config: $CONFIG)"
echo "Headset, first time only:  https://$IP:$PORT/   (accept the cert warning)"
echo "Headset teleop page:       https://$PI_IP:8443/   (the Pi points it at $IP:$PORT by default; add ?video=$IP:$PORT if not)"
if ! ss -ltn 2>/dev/null | grep -q '127.0.0.1:5551 '; then
  echo "NOTE: nothing is publishing the ZED on :5551 yet. Start one of (the headset view appears once it runs):"
  echo "        cd ~/yor_ws && python3 collect.py --task \"...\"   # data collection"
  echo "        cd ~/yor_ws && python3 zed_rgbd.py --vr-only        # teleop without recording"
fi
exec "$PY" server/server.py --video-only --port "$PORT" --camera-config "$CONFIG"
