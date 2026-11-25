"""
estimator.py - Wheel speeds (Kalman filter per wheel) and odometry of the 3-wheel robot.

  * wheel angle from encoder counts -> constant-velocity Kalman filter -> wheel speed
    (same filter and tuning as the odometry assignment);
  * body velocity with the omni forward kinematics: (vx, vy, w) = A^-1 v_wheels;
  * heading from the encoders (w_enc) and from the gyroscope (bias removed, sign corrected);
    the odometry uses the source chosen in the settings (gyroscope by default, encoders if the
    gyroscope is not valid);
  * world frame: X = forward direction at the start, Y = left, theta CCW.
"""
import math

import numpy as np

import config
import kinematics


class WheelKalman:
    """x = [angle, rate]; z = angle from counts (quantized)."""

    def __init__(self, accel_std=config.KF_ACCEL_STD, meas_std=config.KF_MEAS_STD):
        self.q, self.r = accel_std ** 2, meas_std ** 2
        self.x, self.P = None, np.diag([meas_std ** 2, 1.0])

    def step(self, z, dt):
        if self.x is None:
            self.x = np.array([z, 0.0])
            return 0.0
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = self.q * np.array([[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        S = self.P[0, 0] + self.r
        K = self.P[:, 0] / S
        self.x = self.x + K * (z - self.x[0])
        self.P = self.P - np.outer(K, self.P[0, :])
        return self.x[1]


class Estimator:
    def __init__(self, gyro_bias_dps=0.0, k_lin=1.0, k_rot=1.0, use_gyro=True):
        self.kf = [WheelKalman() for _ in range(3)]
        self.k_lin, self.k_rot, self.use_gyro = k_lin, k_rot, use_gyro
        self.bias = gyro_bias_dps
        self.x = self.y = self.th = 0.0
        self.th_enc = self.th_gyro = 0.0
        self.prev = self.enc0 = self.t0 = None

    def step(self, s):
        if self.prev is None:
            self.enc0, self.t0 = s.enc, s.t_ms
        dt = 0.0 if self.prev is None else (s.t_ms - self.prev.t_ms) / 1000.0
        ang = [(s.enc[i] - self.enc0[i]) * 2 * math.pi / config.COUNTS_PER_REV for i in range(3)]
        wr = [self.kf[i].step(ang[i], dt if dt > 0 else config.DT) for i in range(3)]     # rad/s
        vw = [w * config.WHEEL_RADIUS_M for w in wr]
        vx, vy, w_enc = kinematics.body_velocity(vw, self.k_lin, self.k_rot)
        gyro_ok = s.gyro_age_ms <= config.IMU_MAX_AGE_MS
        w_gyro = math.radians(config.GYRO_Z_SIGN * (s.gyro[2] - self.bias)) if gyro_ok else float("nan")
        w = w_gyro if (gyro_ok and self.use_gyro) else w_enc
        if dt > 0:
            mid = self.th + 0.5 * w * dt
            fwd, left = vy, -vx
            self.x += (fwd * math.cos(mid) - left * math.sin(mid)) * dt
            self.y += (fwd * math.sin(mid) + left * math.cos(mid)) * dt
            self.th += w * dt
            self.th_enc += w_enc * dt
            if gyro_ok:
                self.th_gyro += w_gyro * dt
        self.prev = s
        rpm = [x * 60 / (2 * math.pi) for x in wr]
        fwd, left = vy, -vx
        vxw = fwd * math.cos(self.th) - left * math.sin(self.th)
        vyw = fwd * math.sin(self.th) + left * math.cos(self.th)
        return {"t": (s.t_ms - self.t0) / 1000.0, "dt": dt, "x": self.x, "y": self.y, "th": self.th,
                "th_enc": self.th_enc, "th_gyro": self.th_gyro, "v": vy, "v_lat": -vx, "vxw": vxw, "vyw": vyw, "w": w,
                "w_enc": w_enc, "w_gyro": w_gyro, "rpm1": rpm[0], "rpm2": rpm[1], "rpm3": rpm[2],
                "pwm1": s.pwm[0], "pwm2": s.pwm[1], "pwm3": s.pwm[2], "gyro_ok": int(gyro_ok)}


def gyro_bias(samples):
    vals = [s.gyro[2] for s in samples if s.gyro_age_ms <= config.IMU_MAX_AGE_MS]
    return float(np.mean(vals)) if vals else 0.0
