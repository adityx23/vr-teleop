# yor.py RPC interface (what `server/yor_bridge.py` calls)

`server/yor_bridge.py` drives the robot only through these calls on a
commlink `RPCServer` (default `localhost:5557`; `--rpc-host/--rpc-port`).
On the robot they are served by `robot/yor.py` in the YOR repo. Anything that
serves the same calls -- e.g. a MuJoCo sim -- can stand in for it, and the
bridge, `whole_body.py`, the page and the HUD run unchanged.

commlink's `RPCServer` handles **one call at a time**, so a slow method stalls
every other call (arm targets included). Keep every method fast; never block
waiting on hardware.

## Frames and units
- **Base frame**: robot base, x forward, y left, z up, metres. Lift-independent:
  on the robot the IK model pins the lift joint at 0, so EE poses do not move
  when the lift does (`whole_body.py` / `lift_retarget.py` assume this).
- **EE pose**: `mink.SE3`; the bridge reads `.wxyz_xyz` = `[w, x, y, z, x, y, z]`.
  EE sites are `left_arm_ee` / `right_arm_ee` in the IK model
  (YOR `robot/yor-description/nero-welded-base-and-lift.mjcf`).
- **Neck**: degrees from the centre of each joint's calibrated range. On this
  robot positive pan = head right, positive tilt = head down (the bridge's
  `--neck-pan-sign/--neck-tilt-sign` default to -1 for that, verified on
  hardware 2026-10-20). "Straight ahead" is `home_deg` from the calibration
  file (`--neck-calib`); without one the bridge uses 0, 0.
- **Gripper opening**: 1.0 = open, 0.0 = closed.

## Calls

Startup (the bridge exits if these fail, unless `--no-neck` / `--no-arms`):

| Call | Returns / does |
|---|---|
| `get_neck_limits()` | `{"pan": (lo, hi), "tilt": (lo, hi)}` degrees, or `None` = no neck. Robot: about ±125.5 pan, ±72.5 tilt |
| `get_left_ee_pose()`, `get_right_ee_pose()` | `mink.SE3` in the base frame (`None` = no arms) |

Every control tick (30 Hz), only for parts that are active:

| Call | Notes |
|---|---|
| `set_bimanual_ee_target(L_ee_target=SE3, R_ee_target=SE3, L_preview_time=, R_preview_time=)` | both arms clutched; preview 2/15 s |
| `set_left_ee_target(ee_target=SE3, gripper_target=None, preview_time=)` / `set_right_...` | one arm clutched |
| `set_base_velocity(np.array([vx, vy, wz]))` | base frame, m/s and rad/s; zeros on leaving base mode |
| `set_neck_position(pan=deg, tilt=deg)` | ~15 Hz, only when the goal changed by 0.2 deg or more |
| `enable_{side}_gripper()` -> `bool` | on that arm's first clutch with teleop on; retried every 5 s if False. Must return fast |
| `set_{side}_gripper(gripper_pos=0..1)` | while that arm is clutched |
| `disable_{side}_gripper()` | teleop off, home, disconnect, exit |
| `lift_up()`, `lift_down()`, `lift_stop()` | from A+Y / X+B. On the robot the lift **keeps moving until `lift_stop()`**; a sim should behave the same (move the setpoint at a fixed rate until stopped) |

Periodic reads:

| Call | Rate | Returns |
|---|---|---|
| `get_left_ee_pose()`, `get_right_ee_pose()`, `get_neck_position()` -> `{"pan", "tilt"}` | 10 Hz | measured state for the clutch anchors and neck |
| `get_collection_state()` | 5 Hz (HUD only; failures just blank the HUD) | dict, see below |
| `get_lift_height()` | 5 Hz (HUD only) | metres, or `None` |

`get_collection_state()` keys the HUD reads: `neck_deg` `[pan, tilt]`,
`{left,right}_gripper` (measured opening, `None` when feedback drops),
`{left,right}_gripper_cmd` (commanded opening, `None` while limp),
`{left,right}_gripper_error` (`None` while the measurement is live, else why it isn't;
the bridge never fills one gripper value in from the other), `{left,right}_ik_ok` (last IK solve
converged, or `None` if never asked), `base_cmd_vel` `[vx, vy, wz]`.
(The robot also returns `timestamp`, `enc_vel`, `neck_cmd_deg`, `{side}_ee`,
`{side}_joints`, `{side}_joints_cmd`; the bridge ignores them.)

Homing (A+X held 1.5 s): `disable_{side}_gripper()`, `lift_stop()`,
`set_base_velocity(zeros)`, `home_left_arm()`, `home_right_arm()`, `home_neck()`.

## How the robot does IK (to mirror it in sim)
`robot/arm/ik_solver.py` `SingleArmIK`, one per arm, on
`nero-welded-base-and-lift.mjcf`: mink + daqp, up to 10 iterations of
`solver_dt` per target, position/rotation tolerance 1e-3. Before each solve the
IK configuration is re-synced to the measured joints. The lift joint is pinned
at 0 inside IK. After 10 consecutive failed solves the arm holds its last
converged command instead of sending the unconverged one.

## Lift travel (unresolved)
- IK model: `lift_joint` range 0..0.208 m; the `Lift` actuator uses
  `inheritrange`, so 0..0.208 as well. The `telescope` default class's ctrlrange
  0..0.416 is a leftover from the old two-stage `robot.mjcf` (see the comment in
  the MJCF); models built from it may still carry the wrong range.
- Robot: the Pico lift reports height in mm; `yor.py` clamps `lift_to_height`
  to 0..0.5 m, and the HUD assumes 0..0.5. Real travel is not yet measured.
- `lift_retarget.py` `LiftConfig` (0..0.20, reach band 0.75..1.05 m) is
  unverified; its automatic retargeting is off on the robot.

## Hardware differences to keep in mind
- The gripper calls go to the arms in `--gripper-sides` (default both). Stock
  setup: AgileX CAN grippers on both arms. SameHand setup (YOR `umami` branch):
  left gripper removed (`get_collection_state` reports `None` for it), right is
  a SameHand hand over Zenoh, and `enable_right_gripper()` fails while its
  bridge is down.
