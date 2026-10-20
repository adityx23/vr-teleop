#!/usr/bin/env bash
# Start the Orin video server for the VR headset (ZED stereo on stream 3 + gripper stubs).
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
echo "Close collect.py / deploy_observer.py / the recorder first -- only one process can open the ZED."
exec "$PY" server/server.py --video-only --port "$PORT" --camera-config "$CONFIG"
