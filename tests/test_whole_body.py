"""
Scripted tests for the whole-body clutch controller and lift retargeting.
No robot, no headset. Run either way:

    python tests/test_whole_body.py
    python -m pytest tests/
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "server"))

import quat  # noqa: E402
from lift_retarget import LiftConfig, LiftRetargeter  # noqa: E402
from whole_body import WholeBodyConfig, WholeBodyTeleop  # noqa: E402

DT = 1.0 / 30.0
HOME_L = [1.0, 0.0, 0.0, 0.0, 0.10, 0.15, 0.90]  # wxyz_xyz, base frame
HOME_R = [1.0, 0.0, 0.0, 0.0, 0.10, -0.15, 0.90]


# ------------------------------------------------------------------ fixtures
def yaw_quat(deg):
    """XR orientation turned `deg` to the left (about +y), [x,y,z,w]."""
    h = math.radians(deg) / 2
    return [0.0, math.sin(h), 0.0, math.cos(h)]


def pitch_quat(deg):
    """XR orientation pitched `deg` up (about +x), [x,y,z,w]."""
    h = math.radians(deg) / 2
    return [math.sin(h), 0.0, 0.0, math.cos(h)]


def ctrl(pos, q=(0, 0, 0, 1), grip=0.0, trigger=0.0, buttons=(), axes=(0, 0, 0, 0)):
    btns = {str(i): {"pressed": False, "value": 0.0} for i in range(6)}
    btns["1"] = {"pressed": grip > 0.5, "value": grip}
    for b in buttons:
        btns[str(b)] = {"pressed": True, "value": 1.0}
    return {"pos": list(pos), "quat": list(q), "grip": grip, "trigger": trigger,
            "axes": list(axes), "buttons": btns}


def payload(head_pos=(0, 1.6, 0), head_q=(0, 0, 0, 1), left=None, right=None):
    return {"head": {"pos": list(head_pos), "quat": list(head_q)}, "left": left, "right": right}


def state(left_ee=HOME_L, right_ee=HOME_R, lift=0.1, neck=(0.0, 0.0)):
    return {"left_ee": left_ee, "right_ee": right_ee, "lift_height": lift, "neck_deg": list(neck)}


def make(**overrides):
    cfg = WholeBodyConfig(pose_filter_alpha=1.0, **overrides)
    return WholeBodyTeleop(cfg)


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        self.t += DT
        return self.t


def run(tele, clock, p, s=None, ticks=1):
    out = None
    for _ in range(ticks):
        out = tele.update(p, s if s is not None else state(), clock())
    return out


def target(out, side):
    return np.array(out["arms"][side]["target"])


def target_R(out, side):
    w, x, y, z = out["arms"][side]["target"][:4]
    return quat.to_matrix([x, y, z, w])


def close(a, b, tol=1e-6):
    return np.allclose(np.asarray(a, dtype=float), np.asarray(b, dtype=float), atol=tol)


# --------------------------------------------------------------- arm clutch
def test_engage_does_not_move_arm():
    tele, clk = make(), Clock()
    out = run(tele, clk, payload(right=ctrl([0.2, 1.2, -0.3], grip=1.0)), ticks=5)
    assert out["arms"]["right"]["clutched"]
    assert close(target(out, "right"), HOME_R)


def test_no_grip_no_target():
    tele, clk = make(), Clock()
    out = run(tele, clk, payload(right=ctrl([0.2, 1.2, -0.3], grip=0.0)), ticks=3)
    assert not out["arms"]["right"]["clutched"]
    assert out["arms"]["right"]["target"] is None


def test_hand_forward_maps_to_plus_x():
    tele, clk = make(), Clock()
    run(tele, clk, payload(right=ctrl([0.2, 1.2, -0.3], grip=1.0)))
    out = run(tele, clk, payload(right=ctrl([0.2, 1.2, -0.4], grip=1.0)), ticks=30)  # 0.1 m toward -z
    assert close(target(out, "right")[4:], np.array(HOME_R[4:]) + [0.1, 0, 0])


def test_axes_left_and_up():
    tele, clk = make(), Clock()
    run(tele, clk, payload(left=ctrl([0, 1.0, 0], grip=1.0)))
    out = run(tele, clk, payload(left=ctrl([-0.05, 1.08, 0], grip=1.0)), ticks=30)  # XR left + up
    assert close(target(out, "left")[4:], np.array(HOME_L[4:]) + [0, 0.05, 0.08])


def test_delta_in_body_frame_when_operator_faces_left():
    # Operator turned 90 deg left: their forward is XR -x. Pushing the hand that
    # way must still be robot +x.
    tele, clk = make(), Clock()
    hq = yaw_quat(90)
    run(tele, clk, payload(head_q=hq, right=ctrl([0, 1.2, 0], grip=1.0)))
    out = run(tele, clk, payload(head_q=hq, right=ctrl([-0.1, 1.2, 0], grip=1.0)), ticks=30)
    assert close(target(out, "right")[4:], np.array(HOME_R[4:]) + [0.1, 0, 0])


def test_head_turn_while_clutched_does_not_move_arm():
    tele, clk = make(), Clock()
    c = ctrl([0.2, 1.2, -0.3], grip=1.0)
    run(tele, clk, payload(right=c))
    for deg in range(0, 91, 5):
        out = run(tele, clk, payload(head_q=yaw_quat(deg), head_pos=(0.05, 1.55, 0.02), right=c))
    assert close(target(out, "right"), HOME_R)


def test_reclutch_has_no_jump():
    tele, clk = make(), Clock()
    run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0)))
    out = run(tele, clk, payload(right=ctrl([0, 1.2, -0.1], grip=1.0)), ticks=30)
    moved = target(out, "right")
    run(tele, clk, payload(right=ctrl([0, 1.2, -0.1], grip=0.0)))  # release
    run(tele, clk, payload(right=ctrl([0.3, 1.0, 0.2], grip=0.0)), ticks=5)  # reposition hand
    # the arm is at the target (measured == moved), so re-clutch anchors there
    s = state(right_ee=moved.tolist())
    out = run(tele, clk, payload(right=ctrl([0.3, 1.0, 0.2], grip=1.0)), s, ticks=5)
    assert close(target(out, "right"), moved)


def test_reclutch_uses_measured_pose_when_arm_moved_away():
    tele, clk = make(), Clock()
    run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0)))
    run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=0.0)))
    elsewhere = [1.0, 0, 0, 0, 0.3, -0.2, 0.7]
    out = run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0)), state(right_ee=elsewhere))
    assert close(target(out, "right"), elsewhere)


def test_grip_hysteresis():
    tele, clk = make(), Clock()
    p = lambda g: payload(right=ctrl([0, 1.2, 0], grip=g))  # noqa: E731
    assert not run(tele, clk, p(0.5))["arms"]["right"]["clutched"]  # below engage
    assert run(tele, clk, p(0.7))["arms"]["right"]["clutched"]
    assert run(tele, clk, p(0.5))["arms"]["right"]["clutched"]  # above release
    assert not run(tele, clk, p(0.3))["arms"]["right"]["clutched"]


def test_rotation_yaw_left_maps_to_robot_z():
    tele, clk = make(), Clock()
    run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0)))
    out = run(tele, clk, payload(right=ctrl([0, 1.2, 0], q=yaw_quat(30), grip=1.0)), ticks=30)
    rv = quat.matrix_to_rotvec(target_R(out, "right"))
    assert close(rv, [0, 0, math.radians(30)], 1e-4)


def test_rotation_is_relative_to_anchor_orientation():
    anchor = quat.to_matrix([0, math.sin(math.pi / 4), 0, math.cos(math.pi / 4)])  # arm already pitched 90
    w, x, y, z = (lambda q: [q[3], q[0], q[1], q[2]])(quat.from_matrix(anchor))
    ee = [w, x, y, z, 0.1, -0.15, 0.9]
    tele, clk = make(), Clock()
    run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0)), state(right_ee=ee))
    out = run(tele, clk, payload(right=ctrl([0, 1.2, 0], q=yaw_quat(20), grip=1.0)), state(right_ee=ee), ticks=30)
    expected = quat.rotvec_to_matrix([0, 0, math.radians(20)]) @ anchor  # base-frame yaw applied on top
    assert close(target_R(out, "right"), expected, 1e-4)


def test_motion_scale():
    tele, clk = make(motion_scale_pos=0.5), Clock()
    run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0)))
    out = run(tele, clk, payload(right=ctrl([0, 1.2, -0.2], grip=1.0)), ticks=30)
    assert close(target(out, "right")[4:], np.array(HOME_R[4:]) + [0.1, 0, 0])


def test_speed_limit():
    tele, clk = make(max_ee_linear_mps=0.3), Clock()
    run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0)))
    out = run(tele, clk, payload(right=ctrl([0, 1.2, -0.3], grip=1.0)))  # 0.3 m jump in one tick
    moved = np.linalg.norm(target(out, "right")[4:] - HOME_R[4:])
    assert abs(moved - 0.3 * DT) < 1e-9


def test_excursion_clamp():
    tele, clk = make(max_clutch_excursion_m=0.2, max_ee_linear_mps=100), Clock()
    run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0)))
    out = run(tele, clk, payload(right=ctrl([0, 1.2, -1.0], grip=1.0)), ticks=5)
    assert close(target(out, "right")[4:], np.array(HOME_R[4:]) + [0.2, 0, 0])


def test_tracking_loss_holds_then_releases():
    tele, clk = make(), Clock()
    run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0)))
    out = run(tele, clk, payload(right=ctrl([0, 1.2, -0.1], grip=1.0)), ticks=30)
    held = target(out, "right")
    out = run(tele, clk, payload(right=None), ticks=3)  # 0.1 s: still clutched, holding
    assert out["arms"]["right"]["clutched"] and close(target(out, "right"), held)
    out = run(tele, clk, payload(right=None), ticks=10)  # > 0.3 s
    assert not out["arms"]["right"]["clutched"]
    # tracking returns with grip still held: must not re-engage until grip is re-pressed
    out = run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0)))
    assert not out["arms"]["right"]["clutched"]
    run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=0.0)))
    assert run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0)))["arms"]["right"]["clutched"]


def test_no_ee_pose_refuses_engage():
    tele, clk = make(), Clock()
    out = run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0)), state(right_ee=None))
    assert not out["arms"]["right"]["clutched"] and "right_no_ee_pose" in out["warnings"]


def test_gripper_follows_trigger_only_while_clutched():
    tele, clk = make(), Clock()
    out = run(tele, clk, payload(right=ctrl([0, 1.2, 0], trigger=0.8)))
    assert out["arms"]["right"]["gripper"] is None
    out = run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0, trigger=0.8)))
    assert out["arms"]["right"]["gripper"] == 0.8


def test_home_only_when_not_clutched():
    tele, clk = make(), Clock()
    out = run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0, buttons=[5])))
    assert out["home"] == []
    run(tele, clk, payload(right=ctrl([0, 1.2, 0])))
    out = run(tele, clk, payload(right=ctrl([0, 1.2, 0], buttons=[5])))
    assert out["home"] == ["right"]
    out = run(tele, clk, payload(right=ctrl([0, 1.2, 0], buttons=[5])))  # held: no repeat
    assert out["home"] == []


# ---------------------------------------------------------------------- base
def test_base_toggle_and_sticks():
    tele, clk = make(), Clock()
    sticks_l, sticks_r = (0, 0, -0.5, -1.0), (0, 0, 1.0, 0)
    out = run(tele, clk, payload(left=ctrl([0, 1, 0], axes=sticks_l), right=ctrl([0, 1, 0], axes=sticks_r)))
    assert not out["base_mode"] and out["base_velocity"] == [0.0, 0.0, 0.0]
    both = lambda: payload(left=ctrl([0, 1, 0], buttons=[4], axes=sticks_l),  # noqa: E731
                           right=ctrl([0, 1, 0], buttons=[4], axes=sticks_r))
    out = run(tele, clk, both())
    assert out["base_mode"]
    vx, vy, wz = out["base_velocity"]
    cfg = tele.cfg
    assert abs(vx - cfg.max_base_linear_mps) < 1e-9  # full forward
    assert vy > 0  # stick left -> +y
    assert abs(wz + cfg.max_base_yaw_rps) < 1e-9  # right stick right -> turn right
    out = run(tele, clk, both(), ticks=5)  # still held: no re-toggle
    assert out["base_mode"]
    run(tele, clk, payload(left=ctrl([0, 1, 0]), right=ctrl([0, 1, 0])))
    out = run(tele, clk, both())
    assert not out["base_mode"] and out["base_velocity"] == [0.0, 0.0, 0.0]


def test_base_deadzone():
    tele, clk = make(), Clock()
    tele.base_mode = True
    out = run(tele, clk, payload(left=ctrl([0, 1, 0], axes=(0, 0, 0.05, -0.08)), right=ctrl([0, 1, 0])))
    assert out["base_velocity"] == [0.0, 0.0, 0.0]


def test_release_all():
    tele, clk = make(), Clock()
    tele.base_mode = True
    run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0)))
    tele.release_all()
    assert not tele.base_mode and not tele.arms["right"].clutched


# ---------------------------------------------------------------------- neck
def test_neck_follows_head():
    tele, clk = make(neck_max_speed_dps=1e6), Clock()
    run(tele, clk, payload())  # zero
    out = run(tele, clk, payload(head_q=yaw_quat(30)))
    assert abs(out["neck"]["pan"] - 30) < 1e-6 and abs(out["neck"]["tilt"]) < 1e-6
    out = run(tele, clk, payload(head_q=pitch_quat(-20)))
    assert abs(out["neck"]["tilt"] + 20) < 1e-6


def test_neck_recenter_and_limits():
    tele, clk = make(neck_max_speed_dps=1e6), Clock()
    run(tele, clk, payload())
    out = run(tele, clk, payload(head_q=yaw_quat(150)))
    assert out["neck"]["pan"] == 90.0  # clamped
    out = run(tele, clk, payload(head_q=yaw_quat(150), left=ctrl([0, 1, 0], buttons=[3])))
    assert abs(out["neck"]["pan"]) < 1e-6  # recentered


def test_neck_speed_limit_starts_from_measured():
    tele, clk = make(neck_max_speed_dps=90), Clock()
    out = run(tele, clk, payload(head_q=yaw_quat(0)), state(neck=(40.0, 0.0)))
    assert abs(out["neck"]["pan"] - 40) < 1e-6  # first tick (dt = 0): holds the measured pose
    out = run(tele, clk, payload(head_q=yaw_quat(0)), state(neck=(40.0, 0.0)))
    assert abs(out["neck"]["pan"] - (40 - 90 * DT)) < 1e-6  # then slews toward 0


# ---------------------------------------------------------------------- lift
def lift_cfg(**kw):
    base = dict(min_h=0.0, max_h=0.2, max_speed_mps=1e6, reach_lo=0.75, reach_hi=1.05)
    base.update(kw)
    return LiftConfig(**base)


def test_lift_inactive_returns_none():
    lr = LiftRetargeter(lift_cfg())
    assert lr.update(False, 1.6, [], 0.1, DT) is None


def test_lift_holds_on_activation():
    lr = LiftRetargeter(lift_cfg())
    assert lr.update(True, 1.6, [0.9], 0.1, DT) == 0.1
    assert abs(lr.update(True, 1.6, [0.9], 0.1, DT) - 0.1) < 1e-6


def test_lift_follows_crouch():
    lr = LiftRetargeter(lift_cfg())
    lr.update(True, 1.6, [0.95], 0.15, DT)
    h = lr.update(True, 1.5, [0.95], 0.15, DT)  # crouch 10 cm, arm still in band
    assert abs(h - 0.05) < 1e-6


def test_lift_rises_for_high_reach():
    lr = LiftRetargeter(lift_cfg(w_head=1.0, w_reach=4.0))
    lr.update(True, 1.6, [0.9], 0.05, DT)
    h = lr.update(True, 1.6, [1.20], 0.05, DT)  # target 0.15 above band at h=0.05
    # optimum of (h-0.05)^2 + 4*(1.20-h-1.05)^2 -> h = 0.13
    assert abs(h - 0.13) < 1e-4


def test_lift_clamped_and_slew_limited():
    lr = LiftRetargeter(lift_cfg(max_speed_mps=0.05))
    lr.update(True, 1.6, [], 0.15, DT)
    h = lr.update(True, 2.0, [], 0.15, DT)  # stand way up
    assert abs(h - (0.15 + 0.05 * DT)) < 1e-9
    for _ in range(300):
        h = lr.update(True, 2.0, [], 0.15, DT)
    assert abs(h - 0.2) < 1e-9


def test_lift_needs_feedback():
    lr = LiftRetargeter(lift_cfg())
    assert lr.update(True, 1.6, [0.9], None, DT) is None
    assert "lift_no_feedback" in lr.warnings


def test_lift_driven_by_clutch_in_controller():
    tele, clk = make(lift=lift_cfg()), Clock()
    out = run(tele, clk, payload(right=ctrl([0, 1.2, 0])), state(lift=0.1))
    assert out["lift_target"] is None  # nothing clutched
    run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0)), state(lift=0.1))
    out = run(tele, clk, payload(head_pos=(0, 1.55, 0), right=ctrl([0, 1.2, 0], grip=1.0)), state(lift=0.1))
    assert abs(out["lift_target"] - 0.05) < 1e-4  # crouched 5 cm, arm z 0.9 still in band
    assert tele.status()["state"] == "ARMS+LIFT"
    out = run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=0.0)), state(lift=0.1))
    assert out["lift_target"] is None


def test_lift_disabled():
    tele, clk = make(lift=lift_cfg(enabled=False)), Clock()
    run(tele, clk, payload(right=ctrl([0, 1.2, 0], grip=1.0)))
    out = run(tele, clk, payload(head_pos=(0, 1.4, 0), right=ctrl([0, 1.2, 0], grip=1.0)))
    assert out["lift_target"] is None


# ------------------------------------------------------------------- runner
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
