"""
fake_link.py - Virtual 3-wheel robot with the same interface as link.RobotLink (raw PWM only).

Plant per wheel: first-order motor above a dead zone, rpm' = (K_i eff - rpm) / tau,
eff = sign(pwm) max(|pwm| - offset_i, 0). The parameters differ a little from the
feedforward calibration on purpose (a real calibration is never exact).
Kinematics: omni forward kinematics, 8 % extra rotation (wheel slip measured on the floor).
Streams telemetry Samples at 50 Hz in real time. Used by ./start.sh demo and for tests.
"""
import collections
import math
import threading
import time

import numpy as np

import config
import kinematics
from link import Sample


class FakeLink:
    virtual = True

    def __init__(self, offsets=(18.0, 19.0, 14.0), gains=(2.30, 2.40, 2.25), tau=config.MOTOR_TAU_S,
                 turn_slip=1.08, realtime=True, delay_ticks=2, yaw_disturbance=None, grip_acc=None,
                 slip_yaw_gain=0.8):
        self.connected, self.firmware, self.realtime = False, config.FIRMWARE, realtime
        self.log = collections.deque(maxlen=200)
        self.last_sample, self.last_rx, self.bad_lines = None, 0.0, 0
        self._subs, self._lock = [], threading.Lock()
        self.off, self.K, self.tau, self.slip = np.array(offsets), np.array(gains), tau, turn_slip
        self.yaw_dist = yaw_disturbance          # f(t_s) -> extra yaw rate [rad/s] not seen by the wheels (slip test)
        # acceleration-induced slip: above grip_acc [m/s^2] the robot yaws by slip_yaw_gain per m/s^2 of excess
        self.grip_acc, self.slip_k, self.v_prev = grip_acc, slip_yaw_gain, (0.0, 0.0)
        self.raw = np.zeros(3)
        self.queue = [np.zeros(3)] * delay_ticks       # measured on the robot: applied PWM lags ~40 ms
        self.last_cmd = 0.0
        self.rpm = np.zeros(3)
        self.counts = np.zeros(3)
        self.x = self.y = self.th = 0.0
        self.t_ms = 0

    def connect(self, wait_ready_s=0):
        self.connected = True
        self.log.append("  Firmware: FutBot_fw_v5 (VIRTUAL ROBOT, demo mode)")
        if self.realtime:
            threading.Thread(target=self._loop, daemon=True).start()

    def close(self):
        self.stop()
        self.connected = False

    def subscribe(self, fn):
        with self._lock:
            self._subs.append(fn)

    def unsubscribe(self, fn):
        with self._lock:
            if fn in self._subs:
                self._subs.remove(fn)

    def send(self, text):
        if text.strip() == "s":
            self.stop()

    def telemetry(self, on=True, period_ms=20):
        pass

    def raw_pwm(self, p1, p2, p3):
        self.raw = np.array([p1, p2, p3], dtype=float)
        self.last_cmd = self.t_ms / 1000.0 if not self.realtime else time.time()

    def stop(self):
        self.raw[:] = 0

    def tick(self):
        """Advance 20 ms and publish one telemetry sample."""
        now = time.time() if self.realtime else self.t_ms / 1000.0
        if now - self.last_cmd > 0.5:                      # firmware timeout
            self.raw[:] = 0
        self.queue.append(np.clip(np.round(self.raw), -255, 255))
        pwm = self.queue.pop(0)
        h = 0.002
        for _ in range(10):
            eff = np.sign(pwm) * np.maximum(np.abs(pwm) - self.off, 0)
            self.rpm += (self.K * eff - self.rpm) / self.tau * h
            self.rpm[pwm == 0] *= 0.9
            self.counts += self.rpm / 60.0 * config.COUNTS_PER_REV * h
            vx, vy, w = kinematics.body_velocity(self.rpm / kinematics.MS_TO_RPM)
            w *= self.slip
            if self.yaw_dist is not None:
                w += self.yaw_dist(self.t_ms / 1000.0)
            if self.grip_acc is not None:
                acc = math.hypot(vx - self.v_prev[0], vy - self.v_prev[1]) / h
                self.v_prev = (vx, vy)
                w += self.slip_k * max(0.0, acc - self.grip_acc)
            fwd, left = vy, -vx
            self.x += (fwd * math.cos(self.th) - left * math.sin(self.th)) * h
            self.y += (fwd * math.sin(self.th) + left * math.cos(self.th)) * h
            self.th += w * h
        self.t_ms += 20
        gz = -math.degrees(w) + np.random.normal(0, 0.2)   # IMU upside down
        s = Sample(self.t_ms, tuple(int(c) for c in self.counts), tuple(int(p) for p in pwm),
                   (0.0, 0.0, gz), (0.1, -0.3, -9.9), math.degrees(self.th) % 360, 10, 10, 30, 2, time.time())
        self.last_sample, self.last_rx = s, s.rx_time
        with self._lock:
            subs = list(self._subs)
        for fn in subs:
            fn(s)

    def _loop(self):
        while self.connected:
            t0 = time.time()
            self.tick()
            time.sleep(max(0.0, 0.02 - (time.time() - t0)))
