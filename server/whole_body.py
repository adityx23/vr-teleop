"""
Clutched relative-delta whole-body teleop for YOR. Pure logic, no robot I/O:
`WholeBodyTeleop.update()` takes one headset payload (see static/main.js) plus
a robot state snapshot, and returns the commands to send. The robot bridge
calls it at a fixed rate (~30 Hz) and does the RPC.

Controls (Quest Touch, WebXR "xr-standard" mapping):
  grip (hold)         clutch that arm: while held, hand motion moves the arm;
                      release and the arm holds where it is
  index trigger       gripper close amount (0 open .. 1 closed), sent while clutched
  B / Y               home that arm (only while not clutched)
  A + X together      toggle base mode
  left stick          base vx / vy (base mode)
  right stick x       base yaw rate (base mode)
  left stick click    recenter the neck's "straight ahead"
  head                neck pan/tilt, always on
  head height + arms  lift (whole-body retargeting, only while an arm is clutched)

Frames:
  XR    WebXR local-floor: x right, y up, -z forward, metres.
  base  robot base frame, lift-independent: x forward, y left, z up. All arm
        poses in and out of this module are base-frame [w,x,y,z, x,y,z]
        (mink SE3 wxyz_xyz order).
  body  the headset's heading (yaw only, gravity aligned) captured when an arm
        is clutched. Hand deltas are measured in this frame, so turning your
        head while clutched does not move the arm.

Clutch math, per arm, on grip press:
  anchor <- measured EE pose (or the last target if the arm is still there),
            controller pose, body heading
each tick:
  d_body = R_yaw0^T (p_ctrl - p_ctrl0)
  target_pos = anchor_pos + scale * R_ROBOT_FROM_XR d_body
  dR       = M (R_yaw0^T R_ctrl R_ctrl0^T R_yaw0) M^T,  M = R_ROBOT_FROM_XR
  target_R = dR(scaled) @ anchor_R
then the excursion clamp and the speed limit.
"""
import math
from dataclasses import dataclass, field

import numpy as np

import quat
from lift_retarget import LiftConfig, LiftRetargeter
from teleop import PoseFilter

SIDES = ("left", "right")

# XR axes -> robot base axes: robot x = -XR z, robot y = -XR x, robot z = XR y.
# Assumed from the conventions above -- VERIFY on hardware (build step 2).
R_ROBOT_FROM_XR = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

TRIGGER_BUTTON = 0
GRIP_BUTTON = 1
STICK_CLICK_BUTTON = 3
PRIMARY_BUTTON = 4  # A (right) / X (left)
SECONDARY_BUTTON = 5  # B (right) / Y (left)


@dataclass
class WholeBodyConfig:
    # arms
    motion_scale_pos: float = 1.0
    motion_scale_rot: float = 1.0
    pose_filter_alpha: float = 0.5  # EMA weight on each new pose sample; 1.0 = no smoothing
    max_ee_linear_mps: float = 0.5
    max_ee_angular_rps: float = 2.0
    max_clutch_excursion_m: float = 0.6  # how far one clutch may drag a target from its anchor
    reanchor_tol_m: float = 0.02  # re-clutch from the last target if the arm is this close to it
    reanchor_tol_rad: float = math.radians(5.0)
    grip_engage: float = 0.6  # grip hysteresis
    grip_release: float = 0.4
    tracking_timeout_s: float = 0.3  # controller pose missing this long while clutched -> release
    # base
    enable_base: bool = True
    stick_deadzone: float = 0.1
    max_base_linear_mps: float = 0.3
    max_base_yaw_rps: float = 0.8
    # neck (degrees; signs and offset are placeholders -- VERIFY on hardware)
    enable_neck: bool = True
    neck_pan_sign: float = 1.0  # +1: head turns left -> pan positive
    neck_tilt_sign: float = 1.0  # +1: head looks up -> tilt positive
    neck_tilt_offset_deg: float = 0.0  # neck tilt that points the camera at the horizon
    neck_limits_deg: dict = field(default_factory=lambda: {"pan": (-90.0, 90.0), "tilt": (-45.0, 45.0)})
    neck_max_speed_dps: float = 180.0
    # lift
    lift: LiftConfig = field(default_factory=LiftConfig)


# ------------------------------------------------------------------ helpers
def _button(ctrl, idx):
    """(pressed, value) of a controller button; buttons arrive keyed by str(idx)."""
    if not ctrl:
        return False, 0.0
    buttons = ctrl.get("buttons") or {}
    b = buttons.get(str(idx)) or buttons.get(idx)
    if not b:
        return False, 0.0
    return bool(b.get("pressed")), float(b.get("value", 0.0))


def _stick(ctrl):
    """Thumbstick (x, y); y is negative when pushed forward."""
    if not ctrl:
        return 0.0, 0.0
    axes = ctrl.get("axes") or []
    if len(axes) < 4:
        return 0.0, 0.0
    return float(axes[2]), float(axes[3])


def _deadzone(v, dz):
    if abs(v) <= dz:
        return 0.0
    return math.copysign((abs(v) - dz) / (1.0 - dz), v)


def _heading(R_head):
    """Yaw (radians, about XR +y, left positive) of the headset's horizontal forward."""
    f = R_head @ np.array([0.0, 0.0, -1.0])
    fx, fz = f[0], f[2]
    if math.hypot(fx, fz) < 0.2:
        # Looking nearly straight up/down: use the head's up vector instead,
        # which points forward when looking down and backward when looking up.
        u = R_head @ np.array([0.0, 1.0, 0.0])
        sign = 1.0 if f[1] < 0 else -1.0
        fx, fz = sign * u[0], sign * u[2]
    return math.atan2(-fx, -fz)


def _yaw_matrix(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _pose_from_wxyz_xyz(v):
    v = np.asarray(v, dtype=float)
    w, x, y, z = v[:4]
    return v[4:7].copy(), quat.to_matrix([x, y, z, w])


def _pose_to_wxyz_xyz(pos, R):
    x, y, z, w = quat.from_matrix(R)
    return [float(w), float(x), float(y), float(z), *map(float, pos)]


def _rot_angle(R):
    return float(np.linalg.norm(quat.matrix_to_rotvec(R)))


class _ArmState:
    def __init__(self):
        self.grip_held = False
        self.clutched = False
        self.anchor_pos = None
        self.anchor_R = None
        self.ctrl0_pos = None
        self.ctrl0_R = None
        self.yaw0 = None
        self.target_pos = None
        self.target_R = None
        self.last_seen_s = None
        self.prev_secondary = False
        self.need_regrip = False  # set after tracking loss: grip must be released before re-clutching


# --------------------------------------------------------------- controller
class WholeBodyTeleop:
    def __init__(self, cfg: WholeBodyConfig | None = None):
        self.cfg = cfg or WholeBodyConfig()
        self.arms = {side: _ArmState() for side in SIDES}
        self._filters = {name: PoseFilter(self.cfg.pose_filter_alpha) for name in ("head",) + SIDES}
        self.base_mode = False
        self._prev_both_primary = False
        self._prev_recenter = False
        self._neck_yaw0 = None
        self.neck_cmd = None  # {"pan", "tilt"} degrees, last commanded
        self.lift = LiftRetargeter(self.cfg.lift)
        self._last_now = None
        self.warnings = []

    # ------------------------------------------------------------ public API
    def update(self, payload, robot_state, now_s):
        """
        payload:     one headset message ({"head", "left", "right"}, see static/main.js),
                     or None if nothing fresh has arrived.
        robot_state: {"left_ee": wxyz_xyz | None, "right_ee": ..., (base frame)
                      "lift_height": m | None, "neck_deg": [pan, tilt] | None}
        now_s:       monotonic time, seconds.
        """
        dt = 0.0 if self._last_now is None else max(0.0, now_s - self._last_now)
        self._last_now = now_s
        self.warnings = []
        payload = payload or {}
        robot_state = robot_state or {}

        head = self._filtered("head", payload.get("head"))
        ctrls = {side: self._filtered(side, payload.get(side)) for side in SIDES}

        commands = {"arms": {}, "home": [], "warnings": self.warnings}
        for side in SIDES:
            commands["arms"][side] = self._update_arm(side, ctrls[side], head, robot_state, now_s, dt)
            if self._home_pressed(side, ctrls[side]):
                commands["home"].append(side)

        commands["base_mode"], commands["base_velocity"] = self._update_base(ctrls)
        commands["neck"] = self._update_neck(head, ctrls["left"], robot_state, dt)
        commands["lift_target"] = self._update_lift(head, robot_state, dt)
        return commands

    def release_all(self):
        """Connection lost / shutdown: unclutch everything, leave base mode."""
        for side in SIDES:
            self._release(side)
            self.arms[side].grip_held = False
        self.base_mode = False
        self.lift.reset()

    def status(self):
        """HUD message for static/main.js handleTeleopState()."""
        clutched = {side: self.arms[side].clutched for side in SIDES}
        modes = []
        if any(clutched.values()):
            modes.append("ARMS")
        if self.base_mode:
            modes.append("BASE")
        if self.lift.active:
            modes.append("LIFT")
        return {
            "type": "teleop_state",
            "state": "+".join(modes) or "IDLE",
            "arm_enabled": clutched,
            "warnings": list(self.warnings),
        }

    # ------------------------------------------------------------- filtering
    def _filtered(self, name, raw):
        if raw is None or raw.get("pos") is None or raw.get("quat") is None:
            return None
        pos, q = self._filters[name].update(raw["pos"], raw["quat"])
        out = dict(raw)
        out["pos"] = np.asarray(pos, dtype=float)
        out["R"] = quat.to_matrix(q)
        return out

    # ------------------------------------------------------------------ arms
    def _update_arm(self, side, ctrl, head, robot_state, now_s, dt):
        cfg = self.cfg
        arm = self.arms[side]

        _, grip = _button(ctrl, GRIP_BUTTON)
        if ctrl is not None:
            grip = max(grip, float(ctrl.get("grip", 0.0)))
            if arm.grip_held:
                arm.grip_held = grip > cfg.grip_release
            else:
                arm.grip_held = grip >= cfg.grip_engage
            if grip <= cfg.grip_release:
                arm.need_regrip = False
            if arm.need_regrip:
                arm.grip_held = False

        if ctrl is not None:
            arm.last_seen_s = now_s
        elif arm.clutched and arm.last_seen_s is not None and now_s - arm.last_seen_s > cfg.tracking_timeout_s:
            self.warnings.append(f"{side}_tracking_lost")
            self._release(side)
            arm.grip_held = False
            arm.need_regrip = True  # require a fresh grip press once tracking is back

        if arm.clutched and not arm.grip_held:
            self._release(side)
        if arm.grip_held and not arm.clutched and ctrl is not None:
            self._engage(side, ctrl, head, robot_state)

        out = {"clutched": arm.clutched, "target": None, "gripper": None}
        if not arm.clutched:
            return out
        if ctrl is not None:
            self._track(side, ctrl, dt)
            out["gripper"] = float(np.clip(ctrl.get("trigger", _button(ctrl, TRIGGER_BUTTON)[1]), 0.0, 1.0))
        out["target"] = _pose_to_wxyz_xyz(arm.target_pos, arm.target_R)
        return out

    def _engage(self, side, ctrl, head, robot_state):
        cfg = self.cfg
        arm = self.arms[side]
        measured = robot_state.get(f"{side}_ee")
        if measured is None:
            self.warnings.append(f"{side}_no_ee_pose")
            return
        m_pos, m_R = _pose_from_wxyz_xyz(measured)

        # Re-clutching while the arm still sits at its last target: anchor on the
        # target, so steady tracking error (e.g. gravity sag) doesn't creep in per clutch.
        if (
            arm.target_pos is not None
            and np.linalg.norm(arm.target_pos - m_pos) <= cfg.reanchor_tol_m
            and _rot_angle(arm.target_R.T @ m_R) <= cfg.reanchor_tol_rad
        ):
            a_pos, a_R = arm.target_pos.copy(), arm.target_R.copy()
        else:
            a_pos, a_R = m_pos, m_R

        arm.anchor_pos, arm.anchor_R = a_pos, a_R
        arm.target_pos, arm.target_R = a_pos.copy(), a_R.copy()
        arm.ctrl0_pos, arm.ctrl0_R = ctrl["pos"].copy(), ctrl["R"].copy()
        # Body heading from the headset; fall back to the controller's own heading.
        arm.yaw0 = _heading(head["R"] if head is not None else ctrl["R"])
        arm.clutched = True

    def _release(self, side):
        arm = self.arms[side]
        arm.clutched = False
        arm.anchor_pos = arm.anchor_R = arm.ctrl0_pos = arm.ctrl0_R = arm.yaw0 = None
        # target_pos/R kept: the arm holds there, and a quick re-clutch anchors on it

    def _track(self, side, ctrl, dt):
        cfg = self.cfg
        arm = self.arms[side]
        M = R_ROBOT_FROM_XR
        Ry = _yaw_matrix(arm.yaw0)

        d_body = Ry.T @ (ctrl["pos"] - arm.ctrl0_pos)
        d_pos = cfg.motion_scale_pos * (M @ d_body)
        if np.linalg.norm(d_pos) > cfg.max_clutch_excursion_m:
            d_pos *= cfg.max_clutch_excursion_m / np.linalg.norm(d_pos)
        goal_pos = arm.anchor_pos + d_pos

        dR_xr = ctrl["R"] @ arm.ctrl0_R.T
        dR = M @ (Ry.T @ dR_xr @ Ry) @ M.T
        dR = quat.rotvec_to_matrix(cfg.motion_scale_rot * quat.matrix_to_rotvec(dR))
        goal_R = dR @ arm.anchor_R

        # Speed limit on the commanded target (catches tracking glitches).
        step = goal_pos - arm.target_pos
        max_step = cfg.max_ee_linear_mps * dt
        dist = np.linalg.norm(step)
        if dist > max_step:
            step *= max_step / dist if dist > 0 else 0.0
        arm.target_pos = arm.target_pos + step

        rv = quat.matrix_to_rotvec(arm.target_R.T @ goal_R)  # remaining rotation, target-local
        angle = np.linalg.norm(rv)
        max_angle = cfg.max_ee_angular_rps * dt
        if angle > max_angle:
            rv *= max_angle / angle
        arm.target_R = arm.target_R @ quat.rotvec_to_matrix(rv)

    def _home_pressed(self, side, ctrl):
        arm = self.arms[side]
        pressed, _ = _button(ctrl, SECONDARY_BUTTON)
        edge = pressed and not arm.prev_secondary
        arm.prev_secondary = pressed
        if edge and not arm.clutched:
            arm.target_pos = arm.target_R = None  # home moves the arm; forget the old target
            return True
        return False

    # ------------------------------------------------------------------ base
    def _update_base(self, ctrls):
        cfg = self.cfg
        left, right = ctrls["left"], ctrls["right"]
        both = _button(left, PRIMARY_BUTTON)[0] and _button(right, PRIMARY_BUTTON)[0]
        if both and not self._prev_both_primary and cfg.enable_base:
            self.base_mode = not self.base_mode
        self._prev_both_primary = both

        if not self.base_mode:
            return False, [0.0, 0.0, 0.0]
        lx, ly = (_deadzone(a, cfg.stick_deadzone) for a in _stick(left))
        rx, _ = (_deadzone(a, cfg.stick_deadzone) for a in _stick(right))
        return True, [
            -ly * cfg.max_base_linear_mps,  # stick forward (y < 0) -> +x
            -lx * cfg.max_base_linear_mps,  # stick left (x < 0) -> +y
            -rx * cfg.max_base_yaw_rps,  # stick left -> turn left (+yaw)
        ]

    # ------------------------------------------------------------------ neck
    def _update_neck(self, head, left, robot_state, dt):
        cfg = self.cfg
        recenter = _button(left, STICK_CLICK_BUTTON)[0]
        recenter_edge = recenter and not self._prev_recenter
        self._prev_recenter = recenter
        if not cfg.enable_neck or head is None:
            return None

        yaw = _heading(head["R"])
        if self._neck_yaw0 is None or recenter_edge:
            self._neck_yaw0 = yaw
        f = head["R"] @ np.array([0.0, 0.0, -1.0])
        pitch = math.asin(float(np.clip(f[1], -1.0, 1.0)))  # against the horizon, up positive

        lim = cfg.neck_limits_deg
        pan = float(np.clip(cfg.neck_pan_sign * math.degrees(_wrap(yaw - self._neck_yaw0)), *lim["pan"]))
        tilt = float(np.clip(cfg.neck_tilt_sign * math.degrees(pitch) + cfg.neck_tilt_offset_deg, *lim["tilt"]))

        if self.neck_cmd is None:
            measured = robot_state.get("neck_deg")
            if measured is not None:
                self.neck_cmd = {"pan": float(measured[0]), "tilt": float(measured[1])}
        if self.neck_cmd is not None:
            max_step = cfg.neck_max_speed_dps * dt
            pan = self.neck_cmd["pan"] + float(np.clip(pan - self.neck_cmd["pan"], -max_step, max_step))
            tilt = self.neck_cmd["tilt"] + float(np.clip(tilt - self.neck_cmd["tilt"], -max_step, max_step))
        self.neck_cmd = {"pan": pan, "tilt": tilt}
        return dict(self.neck_cmd)

    # ------------------------------------------------------------------ lift
    def _update_lift(self, head, robot_state, dt):
        clutched = [s for s in SIDES if self.arms[s].clutched]
        arm_z = [float(self.arms[s].target_pos[2]) for s in clutched]
        head_y = None if head is None else float(head["pos"][1])
        target = self.lift.update(bool(clutched), head_y, arm_z, robot_state.get("lift_height"), dt)
        self.warnings.extend(self.lift.warnings)
        return target
