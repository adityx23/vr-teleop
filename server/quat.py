"""
Minimal quaternion helpers, [x, y, z, w] convention (matches WebXR / three.js).

Just enough to express one pose relative to another frame and to pull a
yaw/pitch out of an orientation -- not a general-purpose math library.
"""
import numpy as np


def conj(q):
    x, y, z, w = q
    return np.array([-x, -y, -z, w])


def mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


def rotate_vec(q, v):
    qv = np.array([v[0], v[1], v[2], 0.0])
    return mul(mul(q, qv), conj(q))[:3]


def pose_in_frame(pos, quat, frame_pos, frame_quat):
    """Express (pos, quat) -- both in world space -- relative to `frame`."""
    frame_conj = conj(frame_quat)
    rel_pos = rotate_vec(frame_conj, np.array(pos) - np.array(frame_pos))
    rel_quat = mul(frame_conj, quat)
    return rel_pos, rel_quat

def pose_from_frame(rel_pos, rel_quat, frame_pos, frame_quat):
    """Inverse of pose_in_frame: map a frame-relative pose back to world space."""
    world_quat = mul(frame_quat, rel_quat)
    world_pos = np.array(frame_pos) + rotate_vec(frame_quat, rel_pos)
    return world_pos, world_quat


def slerp(q0, q1, t):
    """Spherical-linear interpolate q0 -> q1 by fraction t in [0,1]."""
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    dot = np.dot(q0, q1)
    if dot < 0:  # take the short way around the double-cover
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)
    theta0 = np.arccos(np.clip(dot, -1.0, 1.0))
    theta = theta0 * t
    q_ortho = q1 - q0 * dot
    q_ortho = q_ortho / np.linalg.norm(q_ortho)
    return q0 * np.cos(theta) + q_ortho * np.sin(theta)


def angle_between(q1, q2):
    """Angular distance between two orientations, radians, double-cover safe."""
    dot = abs(np.dot(np.asarray(q1, dtype=float), np.asarray(q2, dtype=float)))
    return 2.0 * np.arccos(np.clip(dot, -1.0, 1.0))


def yaw_pitch(q):
    """Yaw (around Y) and pitch (around X) of a y-up orientation, radians.
    Ignores roll -- fine for a 2-DOF pan/tilt neck, not a general Euler
    decomposition."""
    x, y, z, w = q
    yaw = np.arctan2(2 * (w * y + x * z), 1 - 2 * (y * y + x * x))
    pitch = np.arcsin(np.clip(2 * (w * x - y * z), -1.0, 1.0))
    return float(yaw), float(pitch)
