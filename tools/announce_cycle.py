"""
Stand-in for the Orin collector's cycle announcement (and the reference for it): reads a
cycles file, picks one cycle, and publishes it on collection/cycle once a second until
Ctrl-C. The bridge (yor_bridge.py --cycle-source HOST:PORT) then labels every episode with
that cycle's current subtask. Format: docs/EPISODE_FORMAT.md, "Cycles".

    python tools/announce_cycle.py cycles.config               # list the cycles, pick one
    python tools/announce_cycle.py cycles.config fridge_soda   # announce it on *:5559

cycles file: JSON, {"<cycle name>": ["<subtask prompt>", ...], ...}
"""
import argparse
import json
import sys
import time
import uuid

import zmq

TOPIC = "collection/cycle"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("cycles_file")
    parser.add_argument("cycle", nargs="?", help="cycle name (default: ask)")
    parser.add_argument("--bind", default="*:5559", help="host:port to publish on (default %(default)s)")
    args = parser.parse_args()

    with open(args.cycles_file) as f:
        cycles = json.load(f)
    name = args.cycle
    if name is None:
        names = list(cycles)
        for i, n in enumerate(names, 1):
            print(f"  {i}. {n}  ({len(cycles[n])} subtasks)")
        name = names[int(input("cycle #: ")) - 1]
    if name not in cycles:
        sys.exit(f"no cycle {name!r} in {args.cycles_file} (have: {', '.join(cycles)})")

    msg = {"cycle": name, "run_id": uuid.uuid4().hex[:12], "subtasks": cycles[name]}
    sock = zmq.Context().socket(zmq.PUB)
    sock.bind(f"tcp://{args.bind}")
    print(f"Announcing {name!r} on tcp://{args.bind} ({TOPIC}), run {msg['run_id']}:")
    for i, s in enumerate(msg["subtasks"], 1):
        print(f"  {i}. {s}")
    try:
        while True:
            sock.send_multipart([TOPIC.encode(), json.dumps({**msg, "timestamp": time.time()}).encode()])
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
