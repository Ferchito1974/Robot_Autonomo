"""
kinematics.py - Kinematics of the 3-wheel omnidirectional base.

Inverse:  v_i = -sin(a_i) vx + cos(a_i) vy + L w        (v_wheels = A [vx, vy, w])
Forward:  [vx, vy, w] = A^-1 v_wheels

For a forward speed v (vy = v, vx = 0) and yaw rate w:
  wheel 1 (270 deg, rear):        v1 = L w
  wheel 2 (30 deg, front-right):  v2 = +sqrt(3)/2 v + L w
  wheel 3 (150 deg, front-left):  v3 = -sqrt(3)/2 v + L w
This is the omni equivalent of the differential rule v_R,L = v +/- w b/2 of the
assignment: each wheel reference is the projection of v on the wheel direction
plus the rotation term L w.
"""
import math

import numpy as np

import config

A = np.array([[-math.sin(a), math.cos(a), config.L_M] for a in config.WHEEL_ANGLES_RAD])
A_INV = np.linalg.inv(A)
MS_TO_RPM = 60.0 / (2 * math.pi * config.WHEEL_RADIUS_M)


def wheel_speeds(v, w, k_lin=1.0, k_rot=1.0):
    """Forward speed v [m/s], yaw rate w [rad/s] -> wheel tangential speeds [m/s] (firmware sign).
    k_lin, k_rot: real / predicted motion (slip, wheel size); the request is divided by them."""
    return A @ np.array([0.0, v / k_lin, w / k_rot])


def wheel_rpm(v, w, k_lin=1.0, k_rot=1.0):
    return wheel_speeds(v, w, k_lin, k_rot) * MS_TO_RPM


def body_velocity(v_wheels, k_lin=1.0, k_rot=1.0):
    """Wheel tangential speeds [m/s] -> (vx right, vy forward, w CCW), corrected by k_lin, k_rot."""
    vx, vy, w = A_INV @ np.asarray(v_wheels, dtype=float)
    return k_lin * vx, k_lin * vy, k_rot * w


def wheel_rpm_body(fwd, left, w, k_lin=1.0, k_rot=1.0):
    """Body velocity (forward, left [m/s], w [rad/s]) -> wheel RPM (firmware sign). Omni mode."""
    return A @ np.array([-left / k_lin, fwd / k_lin, w / k_rot]) * MS_TO_RPM
