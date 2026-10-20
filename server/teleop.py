"""
Teleop control logic: enable/disable state machine, controller-pose -> arm
end-effector target mapping, gripper/base/neck mapping, plus the precision/
safety layer around that mapping (motion scaling, pose smoothing, rate
limiting, recenter, dead-man's switch). No real hardware is wired up yet --
this module computes the exact commands that would be sent, and logs/
returns them; `_dispatch` is the single seam to replace with real robot
calls later (ROS publish, vendor SDK, serial protocol, whatever it ends up
being).

Input shape, one JSON message per XR frame (see static/main.js):
{
  "t": <client ms timestamp>,
  "head":  {"pos": [x,y,z], "quat": [x,y,z,w]} | null,
  "left":  {"pos":[...], "quat":[...], "trigger": 0..1, "grip": 0..1,
            "axes": [...], "buttons": {idx: {pressed, value}}} | null,
  "right": {...same shape...} | null,
}

Controller button/axis layout follows the WebXR "xr-standard" gamepad
mapping used by Quest Touch controllers:
  buttons[0] = trigger, buttons[1] = grip/squeeze, buttons[3] = thumbstick
  click, buttons[4] = A (right) / X (left), buttons[5] = B (right) / Y (left)
  axes[2], axes[3] = thumbstick x, y (axes[0:2] are an unused touchpad slot)

Enable state machine (confirmed with the user):
  IDLE: A alone (right) -> enable right arm; X alone (left) -> enable left
        arm; both state ARMS. A+X together -> enable BASE.
        (same logic from ARMS: pressing the *other* side's button alone
        enables that arm too, so you can bring the second arm up later.)
  ARMS: A+X together -> disable both arms, command them home, back to IDLE.
  BASE: A+X together -> disable base, back to IDLE.
        (arms and base are mutually exclusive -- only one driven at a time)
A "combo" (both together) is detected via a short time window rather than
requiring the exact same frame, since two physical button presses are never
perfectly simultaneous.

Precision/safety layer (this is what's actually tunable for "how accurate"):
  - MOTION_SCALE_POS/ROT: operator motion -> commanded motion scale. 1.0 is
    pass-through; <1.0 suppresses hand tremor and trades workspace reach
    for fine control, same idea as motion scaling on surgical teleop rigs.
  - POSE_FILTER_ALPHA: EMA smoothing on raw controller/head pose before any
    of the above mapping runs. This is the one deliberate, bounded source
    of latency in the pipeline -- everything else is pass-through.
  - MAX_EE_LINEAR_MPS/MAX_EE_ANGULAR_RPS: rate limiting on the *output*
    target, independent of input noise -- caps how fast a commanded target
    can move even if the input pose jumps (tracking glitch, re-acquisition).
  - Recenter (thumbstick click, per side, while that arm is enabled):
    re-baselines the controller-to-EE mapping at the arm's *current*
    commanded pose, so you can physically reposition without the arm
    jumping -- a "clutch", same as lifting a mouse.
  - Dead-man's switch: if an enabled arm's controller pose goes missing
    (tracking lost) for longer than DEADMAN_TIMEOUT_MS, that arm is
    disabled and sent home automatically rather than holding stale state.
"""
import logging
import math
import time

import numpy as np

import quat

log = logging.getLogger("vr_teleop.teleop")

PRIMARY_BUTTON_IDX = 4  # A (right controller) / X (left controller)
RECENTER_BUTTON_IDX = 3  # thumbstick click
COMBO_WINDOW_MS = 150  # how close together two single-button presses must be to count as "together"
DEADMAN_TIMEOUT_MS = 400  # how long an enabled arm tolerates a missing controller pose before auto-disabling

# Placeholder home poses (robot base frame, meters / [x,y,z,w] quat) --
# replace with the real arm home poses once the robot interface exists.
HOME_POSE = {
    "left": {"pos": [0.25, 0.0, 0.3], "quat": [0.0, 0.0, 0.0, 1.0]},
    "right": {"pos": [-0.25, 0.0, 0.3], "quat": [0.0, 0.0, 0.0, 1.0]},
}

# --- precision/safety tuning -- placeholders, retune against the real arms ---
MOTION_SCALE_POS = 1.0  # 1.0 = pass-through; <1.0 = finer control, less reach
MOTION_SCALE_ROT = 1.0
POSE_FILTER_ALPHA = 0.5  # EMA weight on each new sample; lower = smoother but laggier
MAX_EE_LINEAR_MPS = 0.5
MAX_EE_ANGULAR_RPS = 2.0  # radians/sec

NECK_YAW_LIMIT_DEG = 90.0
NECK_PITCH_LIMIT_DEG = 45.0

MAX_BASE_LINEAR_MPS = 0.5
MAX_BASE_YAW_RPS = 1.0

LOG_PERIOD_S = 0.5


def _btn(payload, idx):
    if not payload:
        return False, 0.0
    b = payload.get("buttons", {}).get(str(idx)) or payload.get("buttons", {}).get(idx)
    if not b:
        return False, 0.0
    return bool(b.get("pressed")), float(b.get("value", 0.0))


class PoseFilter:
    """Bounded EMA smoothing for a single tracked pose (position + quaternion).
    The *only* deliberate added-latency step in the pipeline -- everything
    downstream of this is pass-through rate-limiting, not smoothing."""

    def __init__(self, alpha=POSE_FILTER_ALPHA):
        self.alpha = alpha
        self._pos = None
        self._quat = None

    def update(self, pos, quat_):
        pos = np.asarray(pos, dtype=float)
        quat_ = np.asarray(quat_, dtype=float)
        if self._pos is None:
            self._pos = pos
            self._quat = quat_
        else:
            self._pos = (1 - self.alpha) * self._pos + self.alpha * pos
            self._quat = quat.slerp(self._quat, quat_, self.alpha)
        return self._pos, self._quat

    def reset(self):
        self._pos = None
        self._quat = None


class TeleopController:
    def __init__(self):
        self.state = "IDLE"  # IDLE | ARMS | BASE
        self.arm_enabled = {"left": False, "right": False}

        self._prev_primary_pressed = {"left": False, "right": False}
        self._prev_recenter_pressed = {"left": False, "right": False}
        self._pending_single = None  # {"side": ..., "t": ms}

        self._filters = {
            "head": PoseFilter(),
            "left": PoseFilter(),
            "right": PoseFilter(),
        }

        # baseline = head-relative controller pose captured at enable/recenter time
        self._baseline = {"left": None, "right": None}
        # reference = the EE pose that baseline's delta of zero maps to --
        # HOME_POSE at enable, the frozen current target at recenter
        self._reference = {"left": None, "right": None}
        self.ee_target = {"left": dict(HOME_POSE["left"]), "right": dict(HOME_POSE["right"])}

        self._last_seen_ms = {"left": None, "right": None}
        self._last_t = None

        self._neck_zero_quat = None  # calibrated to "straight ahead" on first head pose seen

        self._last_log = 0.0

    # ---------------------------------------------------------- public API
    def update(self, payload: dict) -> dict:
        t = payload.get("t", time.time() * 1000)
        dt_s = 0.0 if self._last_t is None else max(0.0, (t - self._last_t) / 1000.0)
        self._last_t = t

        head_raw = payload.get("head")
        left_raw = payload.get("left")
        right_raw = payload.get("right")

        head = self._filtered("head", head_raw)
        left = self._filtered("left", left_raw)
        right = self._filtered("right", right_raw)

        warnings = self._handle_deadman(t, left, right)

        newly_enabled, newly_disabled = self._handle_enable_buttons(t, left, right)
        for side in newly_enabled:
            self._enable_arm(side, {"left": left, "right": right}[side], head)
        for side in newly_disabled:
            self._disable_arm(side)

        self._handle_recenter(left, right, head)

        commands = {
            "state": self.state,
            "arm_enabled": dict(self.arm_enabled),
            "warnings": warnings,
        }

        if head is not None:
            commands["neck"] = self._compute_neck(head)

        if self.state == "ARMS":
            if self.arm_enabled["left"] and left is not None and head is not None:
                commands["left_ee_target"] = self._compute_ee_target("left", left, head, dt_s)
                commands["left_gripper"] = left.get("trigger", 0.0)
            if self.arm_enabled["right"] and right is not None and head is not None:
                commands["right_ee_target"] = self._compute_ee_target("right", right, head, dt_s)
                commands["right_gripper"] = right.get("trigger", 0.0)

        if self.state == "BASE":
            commands["base_velocity"] = self._compute_base_velocity(left, right)

        self._dispatch(commands)
        return commands

    # ----------------------------------------------------------- filtering
    def _filtered(self, name, raw_payload):
        if raw_payload is None:
            return None
        pos, q = self._filters[name].update(raw_payload["pos"], raw_payload["quat"])
        out = dict(raw_payload)
        out["pos"] = pos.tolist()
        out["quat"] = q.tolist()
        return out

    # -------------------------------------------------------- dead-man's switch
    def _handle_deadman(self, t, left, right):
        warnings = []
        for side, payload in (("left", left), ("right", right)):
            if payload is not None:
                self._last_seen_ms[side] = t
                continue
            if not self.arm_enabled[side]:
                continue
            last_seen = self._last_seen_ms[side]
            if last_seen is not None and (t - last_seen) <= DEADMAN_TIMEOUT_MS:
                continue
            log.warning("%s controller pose lost >%dms while enabled -- disabling and homing", side, DEADMAN_TIMEOUT_MS)
            warnings.append(f"{side}_tracking_lost")
            self._disable_arm(side)
            if not any(self.arm_enabled.values()) and self.state == "ARMS":
                self.state = "IDLE"
        return warnings

    # ------------------------------------------------------ button handling
    def _handle_enable_buttons(self, t, left, right):
        right_pressed, _ = _btn(right, PRIMARY_BUTTON_IDX)
        left_pressed, _ = _btn(left, PRIMARY_BUTTON_IDX)
        right_edge = right_pressed and not self._prev_primary_pressed["right"]
        left_edge = left_pressed and not self._prev_primary_pressed["left"]
        self._prev_primary_pressed["right"] = right_pressed
        self._prev_primary_pressed["left"] = left_pressed

        newly_enabled, newly_disabled = [], []

        # a pending single-button press that's aged out of the combo window
        # fires as a standalone action
        if self._pending_single and (t - self._pending_single["t"]) > COMBO_WINDOW_MS:
            side = self._pending_single["side"]
            self._pending_single = None
            e, d = self._on_single(side)
            newly_enabled += e
            newly_disabled += d

        if right_edge and left_edge:
            self._pending_single = None
            e, d = self._on_combo()
            newly_enabled += e
            newly_disabled += d
        elif right_edge:
            if self._pending_single and self._pending_single["side"] == "left":
                self._pending_single = None
                e, d = self._on_combo()
                newly_enabled += e
                newly_disabled += d
            else:
                self._pending_single = {"side": "right", "t": t}
        elif left_edge:
            if self._pending_single and self._pending_single["side"] == "right":
                self._pending_single = None
                e, d = self._on_combo()
                newly_enabled += e
                newly_disabled += d
            else:
                self._pending_single = {"side": "left", "t": t}

        return newly_enabled, newly_disabled

    def _on_single(self, side):
        """A (right) or X (left) pressed alone: enable that side's arm."""
        if self.state in ("IDLE", "ARMS") and not self.arm_enabled[side]:
            self.state = "ARMS"
            self.arm_enabled[side] = True
            log.info("single-press %s -> arm enabled (state=ARMS, enabled=%s)", side, self.arm_enabled)
            return [side], []
        return [], []

    def _on_combo(self):
        """A+X together: context-dependent -- enable base from idle, or
        disable+home from whichever mode is currently active."""
        if self.state == "IDLE":
            self.state = "BASE"
            log.info("combo -> BASE enabled")
            return [], []
        if self.state == "ARMS":
            disabled = [s for s in ("left", "right") if self.arm_enabled[s]]
            self.state = "IDLE"
            log.info("combo -> arms disabled, homing %s", disabled)
            return [], disabled
        if self.state == "BASE":
            self.state = "IDLE"
            log.info("combo -> base disabled")
            return [], []
        return [], []

    # ----------------------------------------------------------- EE mapping
    def _enable_arm(self, side, controller_payload, head_payload):
        self.arm_enabled[side] = True
        self._reference[side] = dict(HOME_POSE[side])
        self._set_baseline(side, controller_payload, head_payload)

    def _disable_arm(self, side):
        self.arm_enabled[side] = False
        self.ee_target[side] = dict(HOME_POSE[side])
        self._reference[side] = None
        self._baseline[side] = None
        self._filters[side].reset()

    def _set_baseline(self, side, controller_payload, head_payload):
        if controller_payload is None or head_payload is None:
            self._baseline[side] = None
            return
        rel_pos, rel_quat = quat.pose_in_frame(
            controller_payload["pos"], controller_payload["quat"],
            head_payload["pos"], head_payload["quat"],
        )
        self._baseline[side] = {"rel_pos": rel_pos, "rel_quat": rel_quat}
        log.info("%s arm baseline set (head-relative controller pose captured)", side)

    def _handle_recenter(self, left, right, head):
        for side, payload in (("left", left), ("right", right)):
            if not self.arm_enabled[side]:
                self._prev_recenter_pressed[side] = False
                continue
            pressed, _ = _btn(payload, RECENTER_BUTTON_IDX)
            edge = pressed and not self._prev_recenter_pressed[side]
            self._prev_recenter_pressed[side] = pressed
            if edge and payload is not None and head is not None:
                # Freeze the arm's current commanded pose as the new
                # reference, then re-baseline to the controller's current
                # pose -- so the arm doesn't move, only the mapping origin.
                self._reference[side] = dict(self.ee_target[side])
                self._set_baseline(side, payload, head)
                log.info("%s arm recentered at current pose", side)

    def _compute_ee_target(self, side, controller_payload, head_payload, dt_s):
        baseline = self._baseline[side]
        reference = self._reference[side]
        if baseline is None or reference is None:
            return self.ee_target[side]

        rel_pos, rel_quat = quat.pose_in_frame(
            controller_payload["pos"], controller_payload["quat"],
            head_payload["pos"], head_payload["quat"],
        )
        delta_pos = (rel_pos - baseline["rel_pos"]) * MOTION_SCALE_POS
        raw_delta_quat = quat.mul(rel_quat, quat.conj(baseline["rel_quat"]))
        delta_quat = quat.slerp(np.array([0.0, 0.0, 0.0, 1.0]), raw_delta_quat, MOTION_SCALE_ROT)

        target_pos = np.array(reference["pos"]) + delta_pos
        target_quat = quat.mul(delta_quat, np.array(reference["quat"]))

        target_pos, target_quat = self._rate_limit(side, target_pos, target_quat, dt_s)

        self.ee_target[side] = {"pos": target_pos.tolist(), "quat": target_quat.tolist()}
        return self.ee_target[side]

    def _rate_limit(self, side, target_pos, target_quat, dt_s):
        prev = self.ee_target[side]
        prev_pos = np.array(prev["pos"])
        prev_quat = np.array(prev["quat"])

        if dt_s <= 0:
            return prev_pos, prev_quat  # no time reference yet -- hold, don't move

        max_step = MAX_EE_LINEAR_MPS * dt_s
        step_vec = target_pos - prev_pos
        dist = float(np.linalg.norm(step_vec))
        if dist > max_step and dist > 0:
            target_pos = prev_pos + step_vec / dist * max_step

        max_angle = MAX_EE_ANGULAR_RPS * dt_s
        angle = quat.angle_between(prev_quat, target_quat)
        if angle > max_angle and angle > 0:
            target_quat = quat.slerp(prev_quat, target_quat, max_angle / angle)

        return target_pos, target_quat

    # --------------------------------------------------------- neck mapping
    def _compute_neck(self, head_payload):
        head_quat = head_payload["quat"]
        if self._neck_zero_quat is None:
            self._neck_zero_quat = head_quat  # calibrate "straight ahead" on first frame

        rel_quat = quat.mul(quat.conj(self._neck_zero_quat), head_quat)
        yaw, pitch = quat.yaw_pitch(rel_quat)
        yaw_deg = max(-NECK_YAW_LIMIT_DEG, min(NECK_YAW_LIMIT_DEG, math.degrees(yaw)))
        pitch_deg = max(-NECK_PITCH_LIMIT_DEG, min(NECK_PITCH_LIMIT_DEG, math.degrees(pitch)))
        return {"yaw_deg": yaw_deg, "pitch_deg": pitch_deg}

    # --------------------------------------------------------- base mapping
    @staticmethod
    def _compute_base_velocity(left, right):
        lx = ly = ry = 0.0
        if left and len(left.get("axes", [])) >= 4:
            lx, ly = left["axes"][2], left["axes"][3]
        if right and len(right.get("axes", [])) >= 4:
            ry = right["axes"][2]
        return {
            "vx": -ly * MAX_BASE_LINEAR_MPS,
            "vy": -lx * MAX_BASE_LINEAR_MPS,
            "yaw_rate": -ry * MAX_BASE_YAW_RPS,
        }

    # ------------------------------------------------------------- dispatch
    def _dispatch(self, commands):
        """Single seam for sending commands to the real robot. Right now:
        rate-limited logging only."""
        now = time.time()
        if commands["warnings"]:
            log.warning("teleop warnings: %s", commands["warnings"])
        if now - self._last_log < LOG_PERIOD_S:
            return
        self._last_log = now
        log.info(
            "state=%s enabled=%s neck=%s L=%s R=%s base=%s",
            commands.get("state"),
            commands.get("arm_enabled"),
            commands.get("neck"),
            commands.get("left_ee_target"),
            commands.get("right_ee_target"),
            commands.get("base_velocity"),
        )
