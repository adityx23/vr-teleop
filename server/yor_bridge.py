"""
YOR bridge: whole-body VR teleop -> yor.py RPC.

Serves the WebXR page to the headset, receives head/controller data, runs the
clutch controller (whole_body.py) at 30 Hz, and drives the robot through
yor.py's commlink RPC (default localhost:5557). No video is published from
here; video comes from a separate video server (docs/ORIN_VIDEO_SERVER.md).
Anything that serves the same RPC calls can stand in for yor.py -- e.g. a
simulator. The calls used are listed in docs/YOR_RPC_INTERFACE.md.

On the robot, the YOR repo's robot/whole_body_teleop.py runs this with YOR's
defaults (Orin video address, neck calibration file). Standalone:

    python server/yor_bridge.py --rpc-host localhost --video localhost:8444
    python server/yor_bridge.py --dry-run           # no robot: print commands only
    # headset browser: open the URL this prints -> accept cert -> Enter VR

Teleop starts OFF. Controls:
  A + X (one per hand), tap   teleop on/off  (off: arms hold, base stops, neck holds)
  A + X, hold 1.5 s           home both arms + neck, teleop off
  grip (hold)                 move that arm; release and it holds
  B + Y (one per hand)        base mode on/off;  left stick drive, right stick turn
  head                  neck pan/tilt;  left stick click re-centers
  right trigger         SameHand fingers: 0 open .. full pull = closed (touching)
  A + Y (hold)          lift down;  X + B (hold) lift up;  release stops
The lift's automatic head-height retargeting stays off. The left trigger is unused (no left gripper).
The Pico lift keeps moving until it gets "stop": this script sends stop on release, teleop off,
stale/lost headset, home, and exit -- but a hard crash (kill -9) mid-move would not.

SameHand fingers: start the hand first (~/SameHand-UMI/device/software/run.sh start calib
bridge). They stay limp until the first right-arm clutch with teleop on, then hold their last
target while unclutched (so a grasp survives a re-clutch), and go limp again when teleop turns
off, on home, on headset disconnect, and when this script exits. --no-gripper leaves them alone.

Needs: aiohttp, mink (yor-nero env) and the openssl CLI (self-signed cert).
"""
import argparse
import json
import logging
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))  # this directory, however it's imported

import server as vr_server  # noqa: E402  (server/server.py)
from whole_body import SIDES, WholeBodyConfig, WholeBodyTeleop, _heading  # noqa: E402
import quat  # noqa: E402

DEFAULT_VIDEO = os.environ.get("VR_VIDEO", "localhost:8444")

log = logging.getLogger("whole_body_teleop")

# Payloads older than this are treated as "no data" (controller holds / releases).
STALE_S = 0.3

YOR_RPC = ("localhost", 5557)
NECK_LIMIT_MARGIN_DEG = 2.0  # stay this far inside the calibrated range
NECK_MIN_CHANGE_DEG = 0.2  # don't resend neck goals that barely changed
ARM_PREVIEW_S = 2.0 / 30.0  # default min duration nerolib gets per EE target (--arm-preview); was 2/15
STATE_REFRESH_S = 0.1  # how often measured EE / neck poses are re-read
HUD_REFRESH_S = 0.2  # how often the headset HUD's robot status is re-read
LIFT_RANGE_M = (0.0, 0.5)  # lift travel shown on the HUD (yor.py lift_to_height clamp)
BASE_STOP_TICKS = 5  # zero-velocity commands sent when leaving base mode
LIFT_STOP_TICKS = 3  # lift_stop commands sent after a lift button release
GRIPPER_SIDE = "right"  # the SameHand is on the right arm; the left has no gripper
GRIPPER_MIN_CHANGE = 0.01  # don't resend gripper targets that barely changed
GRIPPER_RETRY_S = 5.0  # wait this long before retrying a failed gripper enable (bridge down)
BUTTON_NAMES = {
    "left": {0: "trigger", 1: "grip", 3: "stick-click", 4: "X", 5: "Y"},
    "right": {0: "trigger", 1: "grip", 3: "stick-click", 4: "A", 5: "B"},
}

# HUD robot status for --dry-run, so the headset HUD can be checked without a robot.
FAKE_HUD = {
    "dry_run": True, "rpc_ok": True, "error": None, "homing": False,
    "neck": {"pan": 0.0, "tilt": 0.0, "pan_lim": [-60.0, 60.0], "tilt_lim": [-30.0, 30.0]},
    "gripper": {"left": None, "right": 0.6}, "ik_ok": {"left": True, "right": True},
    "lift": {"height": 0.10, "min": LIFT_RANGE_M[0], "max": LIFT_RANGE_M[1]},
    "base_vel": [0.0, 0.0, 0.0],
}

# Fake robot state for --dry-run: base-frame EE poses [w,x,y,z, x,y,z].
FAKE_STATE = {
    "left_ee": [1.0, 0.0, 0.0, 0.0, 0.30, 0.20, 0.90],
    "right_ee": [1.0, 0.0, 0.0, 0.0, 0.30, -0.20, 0.90],
    "lift_height": 0.10,
    "neck_deg": [0.0, 0.0],
}


# ------------------------------------------------------------------ TLS cert
def lan_ips():
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        return []
    return [ip for ip in out.split() if ":" not in ip]


def ensure_cert(extra_ips, regen=False):
    """Self-signed cert for this machine's IPs (WebXR needs https on the LAN)."""
    crt, key = vr_server.CERT_PATH, vr_server.KEY_PATH
    if crt.exists() and key.exists() and not regen:
        return
    crt.parent.mkdir(exist_ok=True)
    ips = sorted(set(lan_ips()) | set(extra_ips) | {"127.0.0.1"})
    san = ",".join(["DNS:localhost"] + [f"IP:{ip}" for ip in ips])
    subprocess.run(
        ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "825",
         "-keyout", str(key), "-out", str(crt), "-subj", "/CN=yor-vr-teleop",
         "-addext", f"subjectAltName={san}"],
        check=True, capture_output=True,
    )
    log.info("Generated TLS cert for %s", ", ".join(ips))


# ---------------------------------------------------------------- robot link
def _call_with_timeout(fn, timeout_s):
    """Run fn() in a thread; raise TimeoutError if it doesn't return in time.
    (commlink's RPCClient blocks forever when the server isn't up.)"""
    result = {}

    def target():
        try:
            result["value"] = fn()
        except Exception as e:  # noqa: BLE001
            result["error"] = e

    t = threading.Thread(target=target, daemon=True)
    t.start()
    t.join(timeout_s)
    if t.is_alive():
        raise TimeoutError
    if "error" in result:
        raise result["error"]
    return result["value"]


class RobotLink:
    """All RPC to yor.py, from one thread of its own, so a slow or hung call
    can't stall the controller. Each tick it sends the latest commands; it
    also keeps a cache of measured EE / neck poses for the controller."""

    def __init__(self, host, port, rate_hz, neck_hz, use_neck, use_arms, use_base, use_gripper=True, use_lift=True,
                 neck_calib=None, arm_preview_s=ARM_PREVIEW_S):
        from commlink import RPCClient

        self.rpc = RPCClient(host, port)
        self.period = 1.0 / rate_hz
        self.arm_preview_s = arm_preview_s
        self.neck_every = max(1, round(rate_hz / neck_hz))
        self.use_neck, self.use_arms, self.use_base = use_neck, use_arms, use_base
        self.use_gripper = use_gripper and use_arms
        self.use_lift = use_lift
        try:
            # Attribute lookups are RPC round trips too, so keep them inside the timeout.
            neck_limits = _call_with_timeout(lambda: self.rpc.get_neck_limits(), 5.0)
        except TimeoutError:
            print(f"ERROR: yor.py is not answering on {host}:{port} -- is it running and finished "
                  "starting up? (it opens its RPC server last)", file=sys.stderr, flush=True)
            os._exit(2)  # a normal exit would hang: zmq waits on the unanswered request

        if use_neck and neck_limits is None:
            raise SystemExit("yor.py reports no neck (started with --no-neck, or it failed to connect). "
                             "Run with --no-neck to go without it.")
        self.neck_limits = neck_limits
        try:
            home = json.loads(Path(neck_calib).read_text()).get("home_deg") or {}
        except (OSError, ValueError, TypeError):  # TypeError: no file given
            home = {}
        self.neck_home = {"pan": float(home.get("pan", 0.0)), "tilt": float(home.get("tilt", 0.0))}

        self.lock = threading.Lock()
        self.state = {"left_ee": None, "right_ee": None, "lift_height": None, "neck_deg": None}
        self._refresh_state()
        if use_arms and (self.state["left_ee"] is None or self.state["right_ee"] is None):
            raise SystemExit("yor.py returned no EE pose for an arm (started with --no-arms?). "
                             "Run with --no-arms to go without arms.")

        self.commands = None
        self.home_requested = False
        self.homing = False
        self.neck_sent = None
        self.base_stop_left = 0
        self.lift_sent = None  # "up" | "down" | None: last direction sent
        self.lift_stop_left = 0
        self.sent = {"arm": 0, "base": 0, "neck": 0, "grip": 0}
        self.gripper_on = False  # torque on (enable_right_gripper succeeded)
        self.gripper_sent = None  # last opening sent (1 open .. 0 closed)
        self.gripper_retry_at = 0.0
        self.error = None
        self.last_loop = time.monotonic()
        self.hud_state = {}  # robot status for the headset HUD, see hud()
        self.stop = threading.Event()
        log.info("Robot link up: neck %s, arms %s, base %s. Neck limits %s, home %s",
                 "on" if use_neck else "off", "on" if use_arms else "off", "on" if use_base else "off",
                 neck_limits, self.neck_home)

    def neck_limits_with_margin(self):
        return {k: (lo + NECK_LIMIT_MARGIN_DEG, hi - NECK_LIMIT_MARGIN_DEG) for k, (lo, hi) in self.neck_limits.items()}

    # --- called from the control loop ---
    def submit(self, commands):
        with self.lock:
            self.commands = commands
            if commands["home_all"]:
                self.home_requested = True

    def robot_state(self):
        with self.lock:
            return dict(self.state)

    def hud(self):
        """Robot status for the headset HUD (static/main.js refreshHud). JSON-safe."""
        with self.lock:
            out = dict(self.hud_state)
        out.update(dry_run=False, rpc_ok=self.error is None and time.monotonic() - self.last_loop <= 1.0,
                   error=self.error, homing=self.homing)
        return out

    def status(self):
        if time.monotonic() - self.last_loop > 1.0:
            return "ROBOT LINK STALLED (yor.py not answering?)"
        if self.homing:
            return "HOMING"
        s = f"sent arm {self.sent['arm']} base {self.sent['base']} neck {self.sent['neck']}"
        if self.use_gripper:
            s += f" grip {self.sent['grip']} ({'on' if self.gripper_on else 'limp'})"
        return s + (f"  RPC ERROR {self.error[:70]}" if self.error else "")

    # --- own thread ---
    def _call(self, name, *args, **kwargs):
        try:
            out = getattr(self.rpc, name)(*args, **kwargs)
            self.error = None
            return out
        except Exception as e:  # noqa: BLE001 -- keep going; shown in the status line
            msg = f"{name}: {e}"
            if msg != self.error:
                log.warning("RPC %s", msg)
            self.error = msg
            return None

    def _refresh_state(self):
        new = {}
        if self.use_arms:
            for side in SIDES:
                pose = self._call(f"get_{side}_ee_pose")
                new[f"{side}_ee"] = None if pose is None else np.asarray(pose.wxyz_xyz).tolist()
        if self.use_neck:
            pos = self._call("get_neck_position")
            new["neck_deg"] = None if pos is None else [float(pos["pan"]), float(pos["tilt"])]
        with self.lock:
            self.state.update(new)

    def _quiet_call(self, name):
        """RPC for HUD-only data: failures don't touch self.error or the control path."""
        try:
            return getattr(self.rpc, name)()
        except Exception:  # noqa: BLE001 -- HUD fields just go blank
            return None

    def _refresh_hud(self):
        def num(v):
            return None if v is None else float(v)

        cs = self._quiet_call("get_collection_state") or {}
        hud = {"gripper": {}, "ik_ok": {}}
        for side in SIDES:
            # 1 open .. 0 closed; measured when the gripper reports it, else last command
            g = cs.get(f"{side}_gripper")
            hud["gripper"][side] = num(g if g is not None else cs.get(f"{side}_gripper_cmd"))
            ok = cs.get(f"{side}_ik_ok")
            hud["ik_ok"][side] = None if ok is None else bool(ok)
        neck = cs.get("neck_deg") or self.state.get("neck_deg")
        if neck is not None and self.neck_limits:
            hud["neck"] = {"pan": num(neck[0]), "tilt": num(neck[1]),
                           "pan_lim": [num(v) for v in self.neck_limits["pan"]],
                           "tilt_lim": [num(v) for v in self.neck_limits["tilt"]]}
        else:
            hud["neck"] = None
        vel = cs.get("base_cmd_vel")
        hud["base_vel"] = None if vel is None else [float(v) for v in vel]
        h = self._quiet_call("get_lift_height")
        hud["lift"] = None if h is None else {"height": float(h), "min": LIFT_RANGE_M[0], "max": LIFT_RANGE_M[1]}
        with self.lock:
            self.hud_state = hud

    def _home_all(self):
        self.homing = True
        log.info("Homing arms + neck...")
        self._gripper_off()
        self._stop_lift()
        if self.use_base:
            self._call("set_base_velocity", np.zeros(3))
        if self.use_arms:
            self._call("home_left_arm")
            self._call("home_right_arm")
        if self.use_neck:
            self._call("home_neck")
            self.neck_sent = None
        self._refresh_state()
        self.homing = False
        log.info("Homing done")

    def _send_arms(self, cmds):
        import mink

        targets = {s: cmds["arms"][s]["target"] for s in SIDES if cmds["arms"][s]["target"] is not None}
        if not targets:
            return
        se3 = {s: mink.SE3(np.asarray(t, dtype=float)) for s, t in targets.items()}
        if len(se3) == 2:
            self._call("set_bimanual_ee_target", L_ee_target=se3["left"], R_ee_target=se3["right"],
                       L_preview_time=self.arm_preview_s, R_preview_time=self.arm_preview_s)
        else:
            (side, target), = se3.items()
            self._call(f"set_{side}_ee_target", ee_target=target, gripper_target=None,
                       preview_time=self.arm_preview_s)
        self.sent["arm"] += 1

    def _gripper_off(self):
        if self.gripper_on:
            self._call(f"disable_{GRIPPER_SIDE}_gripper")
            log.info("Gripper limp")
        self.gripper_on = False
        self.gripper_sent = None

    def _send_gripper(self, cmds):
        """Right trigger -> SameHand. Torque on at the first clutch with teleop on; held while
        unclutched (gripper is None); limp when teleop goes off."""
        if not cmds["enabled"]:
            self._gripper_off()
            return
        g = cmds["arms"][GRIPPER_SIDE]["gripper"]  # close amount 0..1, None when unclutched
        if g is None:
            return
        if not self.gripper_on:
            now = time.monotonic()
            if now < self.gripper_retry_at:
                return
            if not self._call(f"enable_{GRIPPER_SIDE}_gripper"):
                self.gripper_retry_at = now + GRIPPER_RETRY_S
                if self.error is None:
                    self.error = "gripper enable failed (SameHand bridge running? yor.py --no-samehand?)"
                log.warning("Gripper enable failed; retrying in %.0f s", GRIPPER_RETRY_S)
                return
            self.gripper_on = True
            self.gripper_sent = None
            log.info("Gripper on")
        pos = 1.0 - float(g)  # CanGripper convention: 1 open, 0 closed
        if self.gripper_sent is None or abs(pos - self.gripper_sent) >= GRIPPER_MIN_CHANGE:
            self._call(f"set_{GRIPPER_SIDE}_gripper", gripper_pos=pos)
            self.gripper_sent = pos
            self.sent["grip"] += 1

    def _send_base(self, cmds):
        if cmds["base_mode"]:
            self._call("set_base_velocity", np.asarray(cmds["base_velocity"], dtype=float))
            self.base_stop_left = BASE_STOP_TICKS
            self.sent["base"] += 1
        elif self.base_stop_left > 0:  # just left base mode / teleop off: make sure it stops
            self._call("set_base_velocity", np.zeros(3))
            self.base_stop_left -= 1

    def _call_ok(self, name):
        """_call for methods that return None: True if the RPC went through."""
        self._call(name)
        return self.error is None

    def _send_lift(self, cmds):
        """A + Y / X + B -> lift_down / lift_up, sent once per press (the Pico keeps moving
        until "stop"); on release, lift_stop a few times in case one is dropped."""
        want = cmds.get("lift_manual")  # "up" | "down" | None
        if want is not None:
            if want != self.lift_sent:
                if self.lift_sent is not None:
                    self._call("lift_stop")  # reversing: stop first
                if self._call_ok(f"lift_{want}"):  # failed -> retried next tick
                    self.lift_sent = want
                    log.info("Lift %s", want)
            self.lift_stop_left = LIFT_STOP_TICKS
        elif self.lift_stop_left > 0 or self.lift_sent is not None:
            if self._call_ok("lift_stop"):
                if self.lift_sent is not None:
                    log.info("Lift stop")
                self.lift_sent = None
                self.lift_stop_left -= 1

    def _stop_lift(self):
        if self.use_lift:
            self._call("lift_stop")
            self.lift_sent = None
            self.lift_stop_left = 0

    def _send_neck(self, cmds):
        neck = cmds["neck"]
        if neck is None:  # teleop off or no head data: hold
            return
        goal = (neck["pan"], neck["tilt"])
        if self.neck_sent is not None and max(abs(g - s) for g, s in zip(goal, self.neck_sent)) < NECK_MIN_CHANGE_DEG:
            return
        self._call("set_neck_position", pan=goal[0], tilt=goal[1])
        self.neck_sent = goal
        self.sent["neck"] += 1

    def run(self):
        tick = 0
        next_refresh = 0.0
        next_hud = 0.0
        while not self.stop.is_set():
            t0 = time.monotonic()
            self.last_loop = t0
            with self.lock:
                cmds = self.commands
                home = self.home_requested
                self.home_requested = False
            if home:
                self._home_all()
                with self.lock:
                    self.commands = None  # don't replay commands from before the home
                continue
            if cmds is not None:
                if self.use_arms:
                    self._send_arms(cmds)
                if self.use_gripper:
                    self._send_gripper(cmds)
                if self.use_base:
                    self._send_base(cmds)
                if self.use_lift:
                    self._send_lift(cmds)
                if self.use_neck and tick % self.neck_every == 0:
                    self._send_neck(cmds)
            if t0 >= next_refresh:
                next_refresh = t0 + STATE_REFRESH_S
                self._refresh_state()
            if t0 >= next_hud:
                next_hud = t0 + HUD_REFRESH_S
                self._refresh_hud()
            tick += 1
            time.sleep(max(0.0, self.period - (time.monotonic() - t0)))
        if self.use_base:
            self._call("set_base_velocity", np.zeros(3))
        self._stop_lift()
        if self.use_gripper:
            self._gripper_off()


# ------------------------------------------------------------------- runtime
class Runner:
    def __init__(self, rate_hz, print_period_s, cfg, link=None):
        self.rate_hz = rate_hz
        self.print_period_s = print_period_s
        self.teleop = WholeBodyTeleop(cfg)
        self.link = link
        self.lock = threading.Lock()
        self.latest = None  # (payload, monotonic receive time)
        self.rx_times = []
        self.status = self.teleop.status()
        self.clients = 0
        self.first_payload_logged = False
        self.prev_pressed = {side: set() for side in SIDES}
        self.stop = threading.Event()

    def _log_button_edges(self, payload):
        """Log every button press/release, so the mapping can be checked on the headset."""
        for side in SIDES:
            c = payload.get(side)
            if c is None:
                continue
            pressed = {int(k) for k, b in (c.get("buttons") or {}).items() if b.get("pressed")}
            for idx in sorted(pressed ^ self.prev_pressed[side]):
                name = BUTTON_NAMES[side].get(idx, "?")
                log.info("button %s %s[%d] %s", "press  " if idx in pressed else "release", side, idx, name)
            self.prev_pressed[side] = pressed

    # --- server callbacks (event loop thread; must not block) ---
    def on_payload(self, payload):
        now = time.monotonic()
        with self.lock:
            self.latest = (payload, now)
            self.rx_times.append(now)
            status = self.status
        self._log_button_edges(payload)
        if not self.first_payload_logged:
            self.first_payload_logged = True
            log.info("First headset message:\n%s", json.dumps(payload, indent=1)[:1500])
        return status

    def on_connect(self, peer):
        with self.lock:
            self.clients += 1

    def on_disconnect(self, peer):
        with self.lock:
            self.clients -= 1
            self.latest = None
            self.teleop.set_enabled(False)  # headset gone: teleop off; re-enable with A + X
            self.teleop.release_all()

    # --- control loop (own thread) ---
    def control_loop(self):
        period = 1.0 / self.rate_hz
        next_print = time.monotonic()
        was_enabled = self.teleop.enabled
        while not self.stop.is_set():
            t0 = time.monotonic()
            robot_state = self.link.robot_state() if self.link else FAKE_STATE
            robot_hud = self.link.hud() if self.link else FAKE_HUD
            with self.lock:
                latest = self.latest
                fresh = latest is not None and t0 - latest[1] <= STALE_S
                commands = self.teleop.update(latest[0] if fresh else None, robot_state, t0)
                status = self.teleop.status()
                if self.link is not None and self.link.homing:
                    status["state"] = "HOMING"
                status["robot"] = robot_hud
                self.status = status
                self.rx_times = [t for t in self.rx_times if t0 - t <= 1.0]
                rx_hz = len(self.rx_times)
                clients = self.clients
            if commands["enabled"] != was_enabled:
                was_enabled = commands["enabled"]
                log.info("Teleop %s", "ON" if was_enabled else "OFF")
            if commands["home_all"]:
                log.info("Home all requested (A + X held)")
            if self.link is not None:
                self.link.submit(commands)
            if t0 >= next_print:
                next_print = t0 + self.print_period_s
                print(self.format_line(latest, fresh, commands, rx_hz, clients, robot_state), flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    # --- printing ---
    @staticmethod
    def _ctrl_str(name, c):
        if not c or c.get("pos") is None:
            return f"{name}: --- "
        btns = [k for k, b in (c.get("buttons") or {}).items() if b.get("pressed")]
        axes = c.get("axes") or []
        sx, sy = (axes[2], axes[3]) if len(axes) >= 4 else (0.0, 0.0)
        p = c["pos"]
        return (f"{name}: ({p[0]:+.2f},{p[1]:+.2f},{p[2]:+.2f}) grip {c.get('grip', 0):.2f} "
                f"trig {c.get('trigger', 0):.2f} stick({sx:+.1f},{sy:+.1f}) btn{btns}")

    def format_line(self, latest, fresh, commands, rx_hz, clients, robot_state):
        ts = time.strftime("%H:%M:%S")
        link = f"  [{self.link.status()}]" if self.link else "  [dry run]"
        if latest is None:
            return f"[{ts}] clients {clients} | waiting for headset data...{link}"
        payload = latest[0]
        head = payload.get("head")
        if head and head.get("quat"):
            yaw = math.degrees(_heading(quat.to_matrix(head["quat"])))
            hp = head["pos"]
            head_s = f"head ({hp[0]:+.2f},{hp[1]:+.2f},{hp[2]:+.2f}) yaw {yaw:+4.0f}"
        else:
            head_s = "head ---"
        parts = [f"[{ts}] rx {rx_hz:3d} Hz{'' if fresh else ' STALE'}", head_s,
                 self._ctrl_str("L", payload.get("left")), self._ctrl_str("R", payload.get("right"))]

        out = [f"=> {self.status['state']}"]
        for side in SIDES:
            a = commands["arms"][side]
            anchor = robot_state.get(f"{side}_ee")
            if a["target"] is not None and anchor is not None:
                d = np.array(a["target"][4:]) - np.array(anchor[4:])
                out.append(f"{side[0].upper()} target-measured ({d[0]:+.3f},{d[1]:+.3f},{d[2]:+.3f})")
        if commands.get("lift_manual"):
            out.append(f"lift {commands['lift_manual']}")
        if commands["base_mode"]:
            out.append("base ({:+.2f},{:+.2f},{:+.2f})".format(*commands["base_velocity"]))
        if commands["neck"]:
            out.append(f"neck pan {commands['neck']['pan']:+.0f} tilt {commands['neck']['tilt']:+.0f}")
        if commands["warnings"]:
            out.append(f"warn {commands['warnings']}")
        return " | ".join(parts) + "\n    " + "  ".join(out) + link


def main(default_video=DEFAULT_VIDEO, neck_calib=None):
    """default_video / neck_calib: the YOR launcher passes the Orin address and
    robot/neck/neck_calibration.json (its "home_deg" is the neck's straight-ahead)."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dry-run", action="store_true", help="don't connect to the robot; print commands only")
    parser.add_argument("--no-arms", action="store_true", help="don't drive the arms")
    parser.add_argument("--no-base", action="store_true", help="don't drive the base")
    parser.add_argument("--no-neck", action="store_true", help="don't drive the neck")
    parser.add_argument("--no-gripper", action="store_true", help="don't drive the SameHand fingers")
    parser.add_argument("--no-lift", action="store_true", help="ignore the lift buttons (A + Y / X + B)")
    parser.add_argument("--arm-speed", type=float, default=1.4, help="max arm target speed, m/s")
    parser.add_argument("--arm-preview", type=float, default=ARM_PREVIEW_S,
                        help="min seconds the arm takes to reach each target; lower = less lag, but keep it "
                             "above ~1.5 / --rate or the arm stutters (default %(default).3f)")
    parser.add_argument("--arm-scale", type=float, default=1.0, help="hand motion -> arm motion scale")
    parser.add_argument("--base-speed", type=float, default=0.3, help="max base speed, m/s")
    parser.add_argument("--base-turn", type=float, default=0.8, help="max base turn rate, rad/s")
    parser.add_argument("--neck-hz", type=float, default=15.0, help="neck command rate")
    parser.add_argument("--neck-speed", type=float, default=120.0, help="max neck speed, deg/s")
    # Both -1 on this robot (verified on hardware 2026-10-20): the neck servos'
    # positive directions are opposite to head-left / head-up.
    parser.add_argument("--neck-pan-sign", type=float, default=-1.0, choices=(1.0, -1.0),
                        help="flip if the camera turns the opposite way to your head (default -1)")
    parser.add_argument("--neck-tilt-sign", type=float, default=-1.0, choices=(1.0, -1.0),
                        help="flip if the camera tilts the opposite way to your head (default -1)")
    parser.add_argument("--rpc-host", default=YOR_RPC[0], help="yor.py RPC host")
    parser.add_argument("--rpc-port", type=int, default=YOR_RPC[1], help="yor.py RPC port")
    parser.add_argument("--port", type=int, default=vr_server.PORT, help="headset page port")
    parser.add_argument("--ip", action="append", default=[], help="extra IP to put in the TLS cert")
    parser.add_argument("--regen-cert", action="store_true", help="regenerate the TLS cert (e.g. IP changed)")
    parser.add_argument("--rate", type=float, default=30.0, help="control loop rate, Hz")
    parser.add_argument("--print-period", type=float, default=0.5, help="seconds between status lines")
    parser.add_argument("--video", default=default_video,
                        help=f"host:port of the video server (default {default_video}; env VR_VIDEO)")
    parser.add_argument("--neck-calib", default=neck_calib,
                        help="neck_calibration.json; its home_deg is straight ahead (default: 0, 0)")
    parser.add_argument("--no-video", action="store_true", help="don't point the headset at a video server")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    logging.getLogger("aiohttp.access").setLevel(logging.WARNING)
    ensure_cert(args.ip, regen=args.regen_cert)

    cfg = WholeBodyConfig(
        max_ee_linear_mps=args.arm_speed,
        motion_scale_pos=args.arm_scale,
        max_base_linear_mps=args.base_speed,
        max_base_yaw_rps=args.base_turn,
        enable_base=not args.no_base,
        enable_neck=not args.no_neck,
        neck_pan_sign=args.neck_pan_sign,
        neck_tilt_sign=args.neck_tilt_sign,
        neck_max_speed_dps=args.neck_speed,
    )
    cfg.lift.enabled = False  # not driven yet: no lift height feedback, no non-blocking lift RPC

    link = None
    if not args.dry_run:
        link = RobotLink(args.rpc_host, args.rpc_port, args.rate, args.neck_hz,
                         use_neck=not args.no_neck, use_arms=not args.no_arms, use_base=not args.no_base,
                         use_gripper=not args.no_gripper, use_lift=not args.no_lift,
                         neck_calib=args.neck_calib, arm_preview_s=args.arm_preview)
        if link.neck_limits:
            cfg.neck_limits_deg = link.neck_limits_with_margin()
            cfg.neck_pan_offset_deg = link.neck_home["pan"]  # head straight ahead -> neck home
            cfg.neck_tilt_offset_deg = link.neck_home["tilt"]
        threading.Thread(target=link.run, daemon=True).start()

    runner = Runner(args.rate, args.print_period, cfg, link)
    threading.Thread(target=runner.control_loop, daemon=True).start()

    ips = [ip for ip in lan_ips() if ip.startswith("192.168.")] or lan_ips() or ["<pi-ip>"]
    video = None if args.no_video else args.video
    log.info("%s. Teleop starts OFF: tap A + X to turn it on. Headset URL: %s",
             "DRY RUN: no robot commands" if link is None else "LIVE",
             "  ".join(f"https://{ip}:{args.port}" for ip in ips))
    if video:
        log.info("Video from %s -- accept its cert once at https://%s/ on the headset", video, video)
    try:
        vr_server.run(runner.on_payload, runner.on_connect, runner.on_disconnect, cameras=False,
                      default_video=video, port=args.port)
    finally:
        runner.stop.set()
        if link is not None:
            link.stop.set()
            time.sleep(0.2)  # let the link send a final base stop


if __name__ == "__main__":
    main()
