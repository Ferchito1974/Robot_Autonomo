"""
trajectories.py - Velocity references (v, w) for the three movements.

Each trajectory is a list of segments (duration, v, w). Each segment is executed with a
trapezoidal profile (ramps limited by ACC_MAX, ALPHA_MAX) whose area is exact, so the
distance and the angle of every segment are kept. The ideal path is the integration of the
same references (what a perfect velocity controller would produce).
No position or heading feedback is used (not required in this assignment).
"""
import math

import numpy as np

import config


def segments(kind, p, acc=config.ACC_MAX, alpha=config.ALPHA_MAX):
    """Return [(duration_s, v, w), ...] for 'line', 'circle' or 'square' (durations at constant speed).
    Each segment is later executed as a trapezoid (see Reference), so a turn never starts while the
    robot is still moving forward."""
    if kind == "spin":
        w = float(p["w"])
        return [(2 * math.pi * p["turns"] / w, 0.0, w)]
    v = float(p["v"])
    if kind == "line":
        return [(p["distance"] / v, v, 0.0)]
    if kind == "circle":
        R = float(p["radius"])
        return [(2 * math.pi * R * p.get("turns", 1.0) / v, v, v / R)]       # CCW circle
    if kind == "square":
        wt = float(p["w_turn"])
        segs = []
        for _ in range(4):
            segs.append((p["side"] / v, v, 0.0))
            segs.append((0.15, 0.0, 0.0))                                     # settle
            segs.append(((math.pi / 2) / wt, 0.0, wt))                        # 90 deg CCW in place
            segs.append((0.15, 0.0, 0.0))
        return segs
    raise ValueError(kind)


class Reference:
    """Trapezoidal (v, w) reference. A segment (d, v, w) ramps up in tr = max(v/acc, w/alpha)
    (at most d), holds, and ramps down in tr: the area is exactly v d and w d (distance and angle
    of the segment), and the segment lasts d + tr."""

    def __init__(self, kind, params, acc=config.ACC_MAX, alpha=config.ALPHA_MAX):
        self.kind, self.params, self.acc, self.alpha = kind, dict(params), acc, alpha
        self.segs, t = [], 0.0
        for d, v, w in segments(kind, params, acc, alpha):
            tr = min(d, max(abs(v) / acc, abs(w) / alpha, 1e-3))
            self.segs.append((t, d, tr, v, w))
            t += d + tr
        self.T = t
        self.v = self.w = 0.0

    def at(self, t):
        for t0, d, tr, v, w in self.segs:
            tau = t - t0
            if 0 <= tau < d + tr:
                f = tau / tr if tau < tr else 1.0 if tau < d else max(0.0, 1.0 - (tau - d) / tr)
                return v * f, w * f
        return 0.0, 0.0

    def step(self, t, dt=None):
        """(v, w) reference at time t (s since start)."""
        self.v, self.w = self.at(t)
        return self.v, self.w


def ideal_path(kind, params, dt=config.DT, acc=config.ACC_MAX, alpha=config.ALPHA_MAX):
    """Integrate the limited references: arrays t, v, w, x, y, phi (x forward at start, y left)."""
    ref = Reference(kind, params, acc, alpha)
    n = int((ref.T + 1.0) / dt)
    out = np.zeros((n, 6))
    x = y = phi = 0.0
    for k in range(n):
        t = k * dt
        v, w = ref.step(t, dt)
        out[k] = (t, v, w, x, y, phi)
        x += v * math.cos(phi + 0.5 * w * dt) * dt
        y += v * math.sin(phi + 0.5 * w * dt) * dt
        phi += w * dt
    return {k: out[:, i] for i, k in enumerate(("t", "v", "w", "x", "y", "phi"))}


class OmniReference:
    """World-frame velocity reference (Vx, Vy) for the omni mode: the robot keeps its initial heading
    and translates. X = initial forward direction, Y = left.
      line:   (v, 0) for distance / v
      square: four sides in the directions 0, 90, 180, 270 deg, no turns
      circle: speed v tangent to a CCW circle of radius R (centre at (0, R)), heading kept at 0
    Each segment is a trapezoid of the speed with ramps v / acc, so the distances are exact."""

    def __init__(self, kind, params, acc=config.ACC_MAX):
        self.kind, self.params, self.acc = kind, dict(params), acc
        v = float(params["v"])
        if kind == "line":
            parts = [(params["distance"] / v, 0.0)]
        elif kind == "square":
            parts = []
            for k in range(4):
                parts += [(params["side"] / v, k * math.pi / 2), (0.15, None)]
        elif kind == "circle":
            parts = [(2 * math.pi * params["radius"] * params.get("turns", 1.0) / v, "circle")]
        else:
            raise ValueError(kind)
        self.segs, t = [], 0.0
        for d, direction in parts:
            tr = min(d, v / acc) if direction is not None else 0.0
            self.segs.append((t, d, tr, v if direction is not None else 0.0, direction))
            t += d + tr
        self.T = t

    def speed(self, t):
        for t0, d, tr, v, direction in self.segs:
            tau = t - t0
            if 0 <= tau < d + tr:
                if v == 0:
                    return 0.0, None
                f = tau / tr if tau < tr else 1.0 if tau < d else max(0.0, 1.0 - (tau - d) / tr)
                return v * f, direction
        return 0.0, None

    def _arc(self, t):
        """Distance travelled on the circle at time t (exact integral of the trapezoid)."""
        t0, d, tr, v, _ = self.segs[0]
        tau = max(0.0, t - t0)
        if tau < tr:
            return v * tau ** 2 / (2 * tr)
        if tau < d:
            return v * tr / 2 + v * (tau - tr)
        u = min(tau - d, tr)
        return v * (d - tr / 2) + v * u - v * u ** 2 / (2 * tr)

    def at(self, t):
        """World velocity (Vx, Vy) at time t (pure function of t)."""
        sp, direction = self.speed(t)
        if direction is None:
            return 0.0, 0.0
        ang = self._arc(t) / self.params["radius"] if direction == "circle" else direction
        return sp * math.cos(ang), sp * math.sin(ang)

    def step(self, t, dt=None):
        return self.at(t)


def ideal_path_omni(kind, params, dt=config.DT, acc=config.ACC_MAX):
    ref = OmniReference(kind, params, acc)
    n = int((ref.T + 1.0) / dt)
    x = y = 0.0
    out = np.zeros((n, 3))
    for k in range(n):
        vx, vy = ref.step(k * dt, dt)
        out[k] = (k * dt, x, y)
        x += vx * dt
        y += vy * dt
    return {"t": out[:, 0], "x": out[:, 1], "y": out[:, 2]}
