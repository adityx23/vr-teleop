# Cycles: what the Orin collector needs to do

The Pi side is done (`server/cycles.py`, `yor_bridge.py --cycle-source`). It steps through a
cycle of subtasks on the stick clicks and labels every episode with the current subtask, but
it only knows **which** cycle once the Orin announces one. Until then it falls back to
`--task`, so nothing changes. Wire format and click rules: [EPISODE_FORMAT.md, "Cycles"](EPISODE_FORMAT.md#cycles-orin---pi).

Three changes, all in `~/yor_ws` on the Orin.

## 1. `~/yor_ws/vr_teleop/cycles.config`

One entry per cycle: a name and its subtasks in order. Each subtask is the language prompt
for its episodes. JSON shown here; any format works, since only `collect.py` reads it:

```json
{
  "fridge_soda": [
    "open the fridge",
    "grab the soda",
    "close the fridge",
    "open the fridge",
    "place the soda in the fridge",
    "close the fridge"
  ]
}
```

A cycle should leave the scene the way it found it (self-resetting), so it can run lap after
lap. The Pi remembers the current subtask per cycle **name + subtask list**: editing a
cycle's subtasks restarts it at the first one.

## 2. `start_collection.sh`: pick a cycle instead of a task

List the cycles in `cycles.config`, let the operator pick one, and pass its name to
`collect.py` (e.g. `--cycle fridge_soda`) in place of the task prompt.

## 3. `collect.py`: announce it, and read the new fields

**Announce.** Bind a ZMQ PUB on `*:5559` and send the chosen cycle about once a second for as
long as the collector runs (the Pi connects to it; repeating lets either side restart):

```python
import json, threading, time, uuid
import zmq

def announce_cycle(name, subtasks, bind="tcp://*:5559"):
    msg = {"cycle": name, "run_id": uuid.uuid4().hex[:12], "subtasks": subtasks}
    sock = zmq.Context.instance().socket(zmq.PUB)
    sock.bind(bind)

    def loop():
        while True:
            sock.send_multipart([b"collection/cycle",
                                 json.dumps({**msg, "timestamp": time.time()}).encode()])
            time.sleep(1.0)

    threading.Thread(target=loop, daemon=True).start()
    return msg["run_id"]

# at startup, after loading cycles.config:
# run_id = announce_cycle(args.cycle, cycles[args.cycle])
```

Use raw JSON (`send_multipart`), not commlink's Publisher, which pickles. The same sketch
runs on the Pi as `tools/announce_cycle.py`.

**Read.** The `episode/signal` start message already carries `task`. In a cycle, that is
the subtask's prompt, so a collector that stores `task` per episode already labels them
right. It also carries:

| Field | Use |
|---|---|
| `cycle`, `cycle_run_id` | which cycle / collector run the episode belongs to |
| `subtask_index`, `num_subtasks` | 0-based position in the cycle (e.g. for folder names) |
| `subtask_attempt` | 1 = first try at this subtask, 2+ = retries after a fail |
| `cycle_lap` | completed passes through the cycle |

End messages (`episode/signal` with `active: false`, and `teleop/episode_result`) add
`cycle` and `subtask_index`. Keep failed episodes and record their result, since they are useful
for filtering. Very short episodes are expected: a fail started only to "pause", or a ~1 s
success from clicking forward again after going back a subtask. Filter them out later
(e.g. by `num_frames` or duration) rather than at collection time.

Nothing else changes: frames still arrive on the Pi's `:5558`, keyed by `episode_id`.

## Checking it

On the Orin, with `collect.py` running:

```bash
python - <<'EOF'
import zmq
s = zmq.Context().socket(zmq.SUB); s.connect("tcp://127.0.0.1:5559"); s.subscribe(b"collection/cycle")
print(s.recv_multipart())
EOF
```

On the Pi, `whole_body_teleop.py` logs `Cycle 'fridge_soda' (6 subtasks) from the collector`
and the headset HUD shows `1/6 · OPEN THE FRIDGE · READY`. The HUD shows
`ORIN COLLECTOR NOT HEARD` if the announcements stop. If nothing arrives at all, check that
the Pi can reach the Orin on 5559 (`CYCLE_SOURCE` in YOR's `robot/config.py`, default
`ORIN_IP:5559`).
