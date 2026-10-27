"""
Cyclic multi-step tasks for data collection. Pure logic, no I/O: the bridge feeds it the
cycle the Orin collector announced (topic collection/cycle, docs/EPISODE_FORMAT.md) and the
stick clicks, and asks it what the next episode is.

A cycle is a self-resetting list of subtasks, e.g. open fridge, grab soda, close fridge,
open fridge, place soda, close fridge. Each subtask attempt is one episode:

  right click, no episode open   start an episode for the current subtask
  right click, episode open      end it as a success; move on to the next subtask
                                 (after the last one, back to the first: one more lap)
  left click, episode open       end it as a fail; the next right click retries the same subtask
  left click, no episode open    go back one subtask (fix a mis-click; right-click twice to
                                 skip forward again -- that leaves a ~1 s "success" to filter out)

The current subtask is kept in a small JSON file keyed by the cycle's name and subtask list,
so restarting the bridge or the collector mid-cycle resumes where the scene actually is.
"""
import json
import logging
import time
from pathlib import Path

log = logging.getLogger("whole_body_teleop")

STALE_S = 3.0  # no announcement for this long: the HUD warns that the collector is gone


def parse_announcement(msg):
    """collection/cycle JSON -> (name, run_id, [subtask str]); raises ValueError if malformed.
    Subtasks may be strings or {"task": str, ...}."""
    name, subtasks = msg.get("cycle"), msg.get("subtasks")
    if not isinstance(name, str) or not name or not isinstance(subtasks, list) or not subtasks:
        raise ValueError(f"need a non-empty 'cycle' name and 'subtasks' list, got {msg!r:.200}")
    out = []
    for s in subtasks:
        task = s.get("task") if isinstance(s, dict) else s
        if not isinstance(task, str) or not task.strip():
            raise ValueError(f"subtask {s!r:.100} has no task text")
        out.append(task.strip())
    return name, msg.get("run_id"), out


class CycleTracker:
    """Which subtask is next. Not thread-safe: EpisodePublisher calls it under its lock."""

    def __init__(self, state_path=None):
        self.state_path = Path(state_path) if state_path else None
        self.name = None
        self.run_id = None
        self.subtasks = []
        self.idx = 0
        self.lap = 0  # completed passes through the whole cycle
        self.attempt = 0  # episodes started on the current subtask since reaching it
        self.last_result = None  # "success" | "fail" | None, of the last episode
        self.pending = None  # announcement that arrived mid-episode, applied after it ends
        self.heard_at = None  # monotonic time of the last announcement

    @property
    def active(self):
        return bool(self.subtasks)

    @property
    def subtask(self):
        return self.subtasks[self.idx] if self.subtasks else None

    # --- announcements
    def announce(self, msg, now, episode_open):
        """One collection/cycle message. A changed cycle only takes effect between episodes."""
        name, run_id, subtasks = parse_announcement(msg)
        self.heard_at = now
        if (name, subtasks) == (self.name, self.subtasks):
            self.run_id = run_id
            self.pending = None
            return
        if episode_open:
            self.pending = (name, run_id, subtasks)
            return
        self._apply(name, run_id, subtasks)

    def apply_pending(self):
        if self.pending is not None:
            self._apply(*self.pending)
            self.pending = None

    def _apply(self, name, run_id, subtasks):
        self.name, self.run_id, self.subtasks = name, run_id, subtasks
        self.idx, self.lap, self.attempt, self.last_result = 0, 0, 0, None
        saved = self._load().get(name)
        if saved and saved.get("subtasks") == subtasks:
            self.idx = int(saved.get("idx", 0)) % len(subtasks)
            self.lap = int(saved.get("lap", 0))
            log.info("Cycle %r: resuming at subtask %d/%d %r (lap %d)", name, self.idx + 1,
                     len(subtasks), self.subtask, self.lap)
        else:
            log.info("Cycle %r (%d subtasks) from the collector, starting at %r", name,
                     len(subtasks), self.subtask)

    # --- clicks (EpisodePublisher.handle)
    def started(self):
        self.attempt += 1

    def ended(self, success):
        self.last_result = "success" if success else "fail"
        if success:
            self._step(+1)

    def back(self):
        self._step(-1)
        self.last_result = None
        log.info("Cycle %r: back to subtask %d/%d %r", self.name, self.idx + 1, len(self.subtasks), self.subtask)

    def _step(self, d):
        n = len(self.subtasks)
        new = (self.idx + d) % n
        if d > 0 and new == 0:
            self.lap += 1
        elif d < 0 and self.idx == 0:
            self.lap = max(0, self.lap - 1)
        self.idx, self.attempt = new, 0
        self._save()

    # --- what goes out
    def episode_meta(self):
        """Extra episode/signal (start) fields; task is the subtask's prompt."""
        return {"task": self.subtask, "cycle": self.name, "cycle_run_id": self.run_id,
                "subtask_index": self.idx, "num_subtasks": len(self.subtasks),
                "subtask_attempt": self.attempt, "cycle_lap": self.lap}

    def hud(self, now, episode_open):
        if not self.active:
            return None
        return {"name": self.name, "idx": self.idx, "n": len(self.subtasks), "subtask": self.subtask,
                "next": self.subtasks[(self.idx + 1) % len(self.subtasks)], "lap": self.lap,
                "attempt": self.attempt, "last_result": self.last_result,
                "state": "rec" if episode_open else "retry" if self.last_result == "fail" else "ready",
                "stale": self.heard_at is None or now - self.heard_at > STALE_S,
                "pending": self.pending[0] if self.pending else None}

    # --- persistence
    def _load(self):
        if self.state_path is None or not self.state_path.exists():
            return {}
        try:
            return json.loads(self.state_path.read_text())
        except (OSError, ValueError) as e:
            log.warning("Cycle state %s unreadable (%s): starting cycles from the first subtask", self.state_path, e)
            return {}

    def _save(self):
        if self.state_path is None:
            return
        state = self._load()
        state[self.name] = {"subtasks": self.subtasks, "idx": self.idx, "lap": self.lap, "saved_at": time.time()}
        try:
            self.state_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.state_path.with_suffix(".tmp")
            tmp.write_text(json.dumps(state, indent=1))
            tmp.replace(self.state_path)
        except OSError as e:
            log.warning("Cycle state NOT saved to %s: %s", self.state_path, e)
