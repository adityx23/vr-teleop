"""
Whole-body lift retargeting: picks a lift height setpoint from the operator's
head height and the clutched arms' targets. Pure logic, no robot I/O.

Frames: arm targets are in the robot *base* frame (lift-independent, z up).
The arms are mounted on the lift carriage, so an arm target's height relative
to the carriage is `z_base - lift_height`.

The setpoint minimises, over h in [min_h, max_h]:

    w_head  * (h - h_head)^2
  + w_reach * sum_i dist(z_i - h, [reach_lo, reach_hi])^2

where
  h_head = h0 + head_gain * (head_y - head_y0)   (crouch -> lift goes down;
           the neck camera rides on the lift, so this keeps the view at the
           operator's eye height), and
  the reach term pulls the lift so each clutched arm's target sits inside a
  comfortable carriage-relative height band (reach high -> lift goes up).

h0/head_y0 are captured when the retargeter becomes active (first grip held);
while inactive it returns None, meaning "don't drive the lift".
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class LiftConfig:
    enabled: bool = True
    # Lift travel, metres. The IK model's lift_joint is 0..0.208 m -- VERIFY
    # against the real Pico lift (build step 3).
    min_h: float = 0.0
    max_h: float = 0.20
    head_gain: float = 1.0
    w_head: float = 1.0
    w_reach: float = 4.0
    # Comfortable EE height band relative to the carriage (= base-frame z at
    # lift 0). Home EE z is ~0.89 m in the IK model. Placeholder -- VERIFY.
    reach_lo: float = 0.75
    reach_hi: float = 1.05
    # Setpoint slew limit. The real lift's speed is unknown -- VERIFY.
    max_speed_mps: float = 0.05


def _interval_dist(x, lo, hi):
    if x < lo:
        return lo - x
    if x > hi:
        return x - hi
    return 0.0


class LiftRetargeter:
    def __init__(self, cfg: LiftConfig | None = None):
        self.cfg = cfg or LiftConfig()
        self.active = False
        self._h0 = None
        self._head_y0 = None
        self.setpoint = None
        self.warnings = []

    def reset(self):
        self.active = False
        self._h0 = None
        self._head_y0 = None
        self.setpoint = None

    def desired_height(self, head_y, arm_z):
        """Unconstrained-by-slew optimum for the current inputs."""
        c = self.cfg
        h_head = None
        if head_y is not None and self._head_y0 is not None:
            h_head = self._h0 + c.head_gain * (head_y - self._head_y0)

        def cost(h):
            total = 0.0
            if h_head is not None:
                total += c.w_head * (h - h_head) ** 2
            for z in arm_z:
                total += c.w_reach * _interval_dist(z - h, c.reach_lo, c.reach_hi) ** 2
            return total

        if h_head is None and not arm_z:
            return self.setpoint
        # The cost is convex in h, so a ternary search over the travel is exact enough.
        lo, hi = c.min_h, c.max_h
        for _ in range(60):
            m1 = lo + (hi - lo) / 3.0
            m2 = hi - (hi - lo) / 3.0
            if cost(m1) <= cost(m2):
                hi = m2
            else:
                lo = m1
        return 0.5 * (lo + hi)

    def update(self, active, head_y, arm_z, lift_measured, dt_s):
        """
        active:        True while the lift may move (some arm clutched).
        head_y:        headset height (WebXR local-floor y, metres) or None.
        arm_z:         base-frame z of each clutched arm's target.
        lift_measured: measured lift height (m) or None if no feedback.
        Returns the lift height setpoint, or None to leave the lift alone.
        """
        self.warnings = []
        c = self.cfg
        if not c.enabled or not active:
            self.reset()
            return None

        if not self.active:
            if lift_measured is None:
                self.warnings.append("lift_no_feedback")
                return None
            self.active = True
            self._h0 = float(np.clip(lift_measured, c.min_h, c.max_h))
            self._head_y0 = head_y
            self.setpoint = self._h0
            return self.setpoint

        if self._head_y0 is None and head_y is not None:
            self._head_y0 = head_y  # head pose arrived after activation

        desired = self.desired_height(head_y, list(arm_z))
        max_step = c.max_speed_mps * max(dt_s, 0.0)
        step = float(np.clip(desired - self.setpoint, -max_step, max_step))
        self.setpoint = float(np.clip(self.setpoint + step, c.min_h, c.max_h))
        return self.setpoint
