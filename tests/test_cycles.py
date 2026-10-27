"""
Scripted tests for cyclic multi-step tasks (server/cycles.py) and how EpisodePublisher
uses them. No robot, no headset, no Orin: the publisher binds a local ZMQ port and the
test reads its messages back. Run either way:

    python tests/test_cycles.py
    python -m pytest tests/
"""
import json
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "server"))

from cycles import CycleTracker, parse_announcement  # noqa: E402

FRIDGE = {"cycle": "fridge_soda", "run_id": "r1", "subtasks": [
    "open the fridge", "grab the soda", "close the fridge",
    "open the fridge", {"task": "place the soda"}, "close the fridge"]}


def tracker(path=None):
    c = CycleTracker(path)
    c.announce(FRIDGE, 0.0, episode_open=False)
    return c


def test_parse_accepts_strings_and_dicts():
    name, run_id, subs = parse_announcement(FRIDGE)
    assert name == "fridge_soda" and run_id == "r1" and subs[4] == "place the soda" and len(subs) == 6
    for bad in ({}, {"cycle": "x", "subtasks": []}, {"cycle": "x", "subtasks": [""]}, {"cycle": "", "subtasks": ["a"]}):
        try:
            parse_announcement(bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad}")


def test_success_advances_fail_retries_and_laps():
    c = tracker()
    c.started(); c.ended(True)
    assert c.idx == 1 and c.attempt == 0
    c.started(); c.ended(False)
    assert c.idx == 1 and c.last_result == "fail" and c.hud(0.0, False)["state"] == "retry"
    c.started()
    assert c.episode_meta()["subtask_attempt"] == 2 and c.episode_meta()["task"] == "grab the soda"
    c.ended(True)
    for _ in range(4):
        c.started(); c.ended(True)
    assert c.idx == 0 and c.lap == 1  # wrapped: one full lap


def test_back_wraps_and_undoes_lap():
    c = tracker()
    c.back()
    assert c.idx == 5 and c.lap == 0
    c.started(); c.ended(True)
    assert c.idx == 0 and c.lap == 1
    c.back()
    assert c.idx == 5 and c.lap == 0


def test_state_survives_restart_same_cycle_only():
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "cycle_state.json"
        c = tracker(path)
        c.started(); c.ended(True)
        c.started(); c.ended(True)
        assert tracker(path).idx == 2  # bridge restarted: resumes
        changed = dict(FRIDGE, subtasks=FRIDGE["subtasks"][:3])
        c2 = CycleTracker(path)
        c2.announce(changed, 0.0, episode_open=False)
        assert c2.idx == 0  # different subtask list: starts over


def test_cycle_change_waits_for_episode_end():
    c = tracker()
    other = {"cycle": "drawer", "run_id": "r2", "subtasks": ["open the drawer", "close the drawer"]}
    c.announce(other, 1.0, episode_open=True)
    assert c.name == "fridge_soda" and c.hud(1.0, True)["pending"] == "drawer"
    c.apply_pending()
    assert c.name == "drawer" and c.idx == 0 and c.pending is None


def test_stale_when_collector_quiet():
    c = tracker()
    assert not c.hud(1.0, False)["stale"] and c.hud(10.0, False)["stale"]


def test_publisher_signals_carry_the_subtask():
    import zmq
    from yor_bridge import EpisodePublisher

    pub = EpisodePublisher("127.0.0.1", "*", {"task": "fallback", "operator": "t"}, None, cycle=CycleTracker())
    port = pub.sock.getsockopt(zmq.LAST_ENDPOINT).decode().rsplit(":", 1)[1]
    sub = pub.ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.RCVTIMEO, 2000)
    sub.connect(f"tcp://127.0.0.1:{port}")
    time.sleep(0.3)  # slow joiner

    def recv():
        topic, body = sub.recv_multipart()
        return topic.decode(), json.loads(body)

    try:
        pub.handle("fail")  # no cycle yet, nothing open: ignored
        pub.handle("start_or_success")
        topic, msg = recv()
        assert msg["task"] == "fallback" and "cycle" not in msg  # before any announcement: --task
        pub.last_id = None
        pub.end(True)
        recv(); recv()

        pub.announce_cycle(FRIDGE)
        pub.last_id = None  # episode ids are per second; the test is faster than that
        pub.handle("start_or_success")
        topic, msg = recv()
        assert topic == "episode/signal" and msg["active"] and msg["task"] == "open the fridge"
        assert msg["cycle"] == "fridge_soda" and msg["subtask_index"] == 0 and msg["num_subtasks"] == 6
        assert msg["subtask_attempt"] == 1 and msg["operator"] == "t"
        assert pub.hud()["cycle"]["state"] == "rec"
        pub.started_mono -= 5  # past the double-click guard
        pub.handle("start_or_success")  # success -> next subtask, started at once
        _, end = recv()
        _, result = recv()
        assert not end["active"] and end["success"] and end["subtask_index"] == 0
        assert result["result"] == "success" and result["cycle"] == "fridge_soda"
        _, nxt = recv()
        assert nxt["active"] and nxt["task"] == "grab the soda" and nxt["subtask_index"] == 1
        assert pub.active and pub.hud()["task"] == "grab the soda"
        pub.handle("fail")  # open: fail it and stop
        _, end = recv()
        recv()
        assert not end["success"] and not pub.active and pub.hud()["task"] == "grab the soda"
        pub.handle("fail")  # nothing open: back one subtask
        assert pub.hud()["task"] == "open the fridge"
    finally:
        sub.close(linger=0)
        pub.close()


def test_success_chains_episodes_with_unique_ids_and_ignores_double_clicks():
    from yor_bridge import EpisodePublisher

    pub = EpisodePublisher("127.0.0.1", "*", {"task": "fallback"}, None, cycle=CycleTracker())
    try:
        pub.announce_cycle(FRIDGE)
        pub.handle("start_or_success")
        first = pub.episode_id
        pub.handle("start_or_success")  # within a second of the start: ignored
        assert pub.episode_id == first and pub.cycle.idx == 0
        ids = [first]
        for _ in range(6):  # a whole lap of successes, faster than one per second
            pub.started_mono -= 5
            pub.handle("start_or_success")
            assert pub.active
            ids.append(pub.episode_id)
        assert len(set(ids)) == len(ids)  # same-second chained starts get -2, -3, ... suffixes
        assert pub.cycle.idx == 0 and pub.cycle.lap == 1 and pub.episode_meta["subtask_index"] == 0
        assert pub.episode_meta["cycle_lap"] == 1 and pub.episode_meta["subtask_attempt"] == 1
        pub.handle("fail")
        assert not pub.active
        pub.handle("start_or_success")  # a plain start in the same second is still refused
        assert not pub.active
        pub.last_id = None  # a second later
        pub.handle("start_or_success")  # retry the failed subtask
        assert pub.active and pub.episode_meta["subtask_attempt"] == 2
    finally:
        pub.close()


def test_task_mode_success_still_stops():
    from yor_bridge import EpisodePublisher

    pub = EpisodePublisher("127.0.0.1", "*", {"task": "fallback"}, None, cycle=CycleTracker())
    try:
        pub.handle("start_or_success")  # no cycle announced: --task episode
        pub.handle("start_or_success")  # ends it at once (no guard outside a cycle)
        assert not pub.active
    finally:
        pub.close()


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"PASS  {name}")
        except Exception as e:  # noqa: BLE001
            failed += 1
            print(f"FAIL  {name}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
