"""
experiment.py - One experiment: rest (gyro bias) -> velocity references -> wheel control -> log.

Runs inside the telemetry callback (50 Hz): every telemetry line gives one estimate,
one reference step and one PWM command for the three wheels ({PWM,p1,p2,p3}).
Safety: STOP, telemetry lost, encoder jump, time limit. The firmware also stops the
motors by itself if no command arrives for 500 ms.
"""
import csv
import json
import math
import os
import threading
import time

import numpy as np

import config
import controller
import estimator
import kinematics
import settings
import trajectories

BASE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(BASE, "runs")
os.makedirs(RUNS, exist_ok=True)

COLS = ["t", "tau", "scale", "slip", "v_ref", "w_ref", "w_cmd", "vx_ref", "vy_ref", "vxw", "vyw", "v", "v_lat", "w", "w_enc", "w_gyro", "x", "y", "th", "th_enc", "th_gyro",
        "x_ideal", "y_ideal", "th_ideal",
        "ref1", "ref2", "ref3", "rpm1", "rpm2", "rpm3", "u1", "u2", "u3", "pwm1", "pwm2", "pwm3",
        "ff2", "p2", "i2", "ff3", "p3", "i3", "gyro_ok"]


def save_json(p, d):
    with open(p, "w") as f:
        json.dump(d, f, indent=2, allow_nan=False, default=float)


def load_json(p):
    if os.path.exists(p):
        with open(p) as f:
            return json.load(f)
    return None


def _finite(o):
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {k: _finite(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_finite(v) for v in o]
    return o


class Experiment:
    def __init__(self, link, kind, params, ctrl, gains, label="", demo=False, mode="diff", acc=None):
        self.link, self.kind, self.params, self.ctrl, self.gains = link, kind, dict(params), ctrl, dict(gains)
        self.mode = mode                 # "diff": turns like a differential robot; "omni": keeps heading 0
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.run_id = f"{'demo_' if demo else ''}{kind}_{'omni_' if mode == 'omni' else ''}{ctrl}_{stamp}"
        self.label = label or f"{kind} {ctrl}"
        self.dir = os.path.join(RUNS, self.run_id)
        self.phase = "rest"          # rest -> move -> done
        self.result = ""
        self.rows, self.rest = [], []
        self.st = settings.load()
        if acc is not None:
            self.st["acc_max"] = float(acc)          # speed profile of this run
        self.rev_n, self.sat_n = [0, 0, 0], [0, 0, 0]
        self.head_i = 0.0                                # integral of the heading error (omni mode)
        self.tau, self.scale, self.t_end = 0.0, 1.0, None    # traction control: reference time and its rate
        smooth = self.st["s_curve"] >= 0.5
        self.ref = (trajectories.OmniReference(kind, params, self.st["acc_max"], smooth) if mode == "omni" else
                    trajectories.Reference(kind, params, self.st["acc_max"], self.st["alpha_max"], smooth))
        self.ctrls = controller.make(ctrl, gains["kp"], gains.get("ki", 0.0), gains.get("kd", 0.0), gains.get("ff", True),
                                     self.st)
        self.est = None
        self.t_start = None
        self.prev_enc = None
        self.ideal = [0.0, 0.0, 0.0]
        self.finished = threading.Event()
        self._stop_req = False
        self.live = None

    # ── control ──────────────────────────────────────────────────────
    def start(self):
        self.t0 = time.time()
        self.link.subscribe(self.on_sample)
        threading.Thread(target=self._watchdog, daemon=True).start()

    def request_stop(self, why="ABORTADA: STOP pulsado"):
        self._stop_req = True
        self.link.stop()
        if self.phase != "done":
            self._finish(why)

    def _watchdog(self):
        while not self.finished.is_set():
            time.sleep(0.05)
            if time.time() - self.link.last_rx > config.TELEMETRY_TIMEOUT_S and time.time() - self.t0 > 1.0:
                self._finish("ABORTADA: sin telemetria")
            elif time.time() - self.t0 > config.MAX_RUN_TIME_S:
                self._finish("ABORTADA: tiempo maximo")

    def on_sample(self, s):
        if self.phase == "done":
            return
        if self.prev_enc is not None and max(abs(a - b) for a, b in zip(s.enc, self.prev_enc)) > config.MAX_COUNTS_PER_SAMPLE:
            self._finish("ABORTADA: salto de encoder (dato invalido)")
            return
        self.prev_enc = s.enc
        if self.phase == "rest":
            self.link.stop() if len(self.rest) % 10 == 0 else None
            self.rest.append(s)
            if len(self.rest) * config.DT >= self.st["rest_before_s"]:
                self.est = estimator.Estimator(estimator.gyro_bias(self.rest), self.st["k_lin"], self.st["k_rot"],
                                               self.st["heading_source"] == "gyro")
                self.est.step(s)
                self.t_start = s.t_ms
                self.phase = "move"
            return
        e = self.est.step(s)
        dt = e["dt"] if e["dt"] > 0 else config.DT
        t = (s.t_ms - self.t_start) / 1000.0
        x, y, th = self.ideal
        # traction control: slip = |yaw rate of the wheels - yaw rate of the gyroscope|. While the wheels
        # slip, the reference time advances slower (scale < 1); the path geometry does not change.
        slip = abs(e["w_enc"] - e["w_gyro"]) if e["gyro_ok"] else 0.0
        if self.mode == "omni" and self.st["traction"] >= 0.5:
            over = slip - self.st["slip_on"]
            target = 1.0 if over <= 0 else max(config.TC_SCALE_MIN, 1.0 - config.TC_GAIN * over)
            rate = config.TC_DOWN if target < self.scale else config.TC_UP
            self.scale += max(-rate * dt, min(rate * dt, target - self.scale))
        else:
            self.scale = 1.0
        self.tau += self.scale * dt
        tr = self.tau
        if self.mode == "omni":
            # world velocity -> body frame with the measured heading; heading held at 0 (initial)
            vx_ref, vy_ref = self.ref.step(tr, dt)
            vx_ref, vy_ref = vx_ref * self.scale, vy_ref * self.scale
            v_ref, w_ref = math.hypot(vx_ref, vy_ref), 0.0
            th_m = e["th"]
            e_th = 0.0 - th_m
            i_new = self.head_i + self.st["ki_head"] * e_th * dt
            w_raw = self.st["k_head"] * e_th + i_new
            if abs(w_raw) < config.W_HOLD_MAX:                  # anti-windup
                self.head_i = max(-1.0, min(1.0, i_new))
            w_cmd = max(-config.W_HOLD_MAX, min(config.W_HOLD_MAX, self.st["k_head"] * e_th + self.head_i))
            fwd = vx_ref * math.cos(th_m) + vy_ref * math.sin(th_m)
            left = -vx_ref * math.sin(th_m) + vy_ref * math.cos(th_m)
            refs = kinematics.wheel_rpm_body(fwd, left, w_cmd, self.st["k_lin"], self.st["k_rot"])
            fx, fy = self.ref.at(tr + self.st["ff_lookahead"])
            fx, fy = fx * self.scale, fy * self.scale
            refs_ff = kinematics.wheel_rpm_body(fx * math.cos(th_m) + fy * math.sin(th_m),
                                                -fx * math.sin(th_m) + fy * math.cos(th_m), w_cmd,
                                                self.st["k_lin"], self.st["k_rot"])
            self.ideal = [x + vx_ref * dt, y + vy_ref * dt, 0.0]
        else:
            v_ref, w_ref = self.ref.step(t, dt)
            w_cmd = w_ref
            vx_ref, vy_ref = v_ref * math.cos(th + 0.5 * w_ref * dt), v_ref * math.sin(th + 0.5 * w_ref * dt)
            # ideal path (integration of the same references)
            self.ideal = [x + vx_ref * dt, y + vy_ref * dt, th + w_ref * dt]
            refs = kinematics.wheel_rpm(v_ref, w_ref, self.st["k_lin"], self.st["k_rot"])
            refs_ff = kinematics.wheel_rpm(*self.ref.at(t + self.st["ff_lookahead"]), self.st["k_lin"], self.st["k_rot"])
        meas = (e["rpm1"], e["rpm2"], e["rpm3"])
        moving = self.tau <= self.ref.T
        if moving:
            u = [c.step(r, m, dt, rf) for c, r, m, rf in zip(self.ctrls, refs, meas, refs_ff)]
            self.link.raw_pwm(*[int(round(x)) for x in u])
        else:
            u = [0.0, 0.0, 0.0]
            self.link.raw_pwm(0, 0, 0)
        # safety: wheel turning against its PWM (encoder direction fault) or saturated too long
        for i in range(3):
            p, r = s.pwm[i], meas[i]
            bad = abs(p) > config.REVERSE_PWM and abs(r) > config.REVERSE_RPM and (p > 0) != (r > 0)
            self.rev_n[i] = self.rev_n[i] + 1 if bad else 0
            self.sat_n[i] = self.sat_n[i] + 1 if abs(p) >= config.PWM_MAX - 1 else 0
            if self.rev_n[i] * config.DT >= config.REVERSE_S:
                self._finish(f"ABORTADA: rueda {i + 1} gira contra su PWM (revisar encoder)")
                return
            if self.sat_n[i] * config.DT >= config.SAT_ABORT_S:
                self._finish(f"ABORTADA: rueda {i + 1} saturada mas de {config.SAT_ABORT_S} s")
                return
        c2, c3 = self.ctrls[1].terms, self.ctrls[2].terms
        if self.mode == "omni":
            e = dict(e, v=math.hypot(e["vxw"], e["vyw"]))           # speed in the world frame
        row = dict(e, slip=slip, scale=self.scale, tau=self.tau, v_ref=v_ref, w_ref=w_ref, w_cmd=w_cmd, vx_ref=vx_ref, vy_ref=vy_ref, x_ideal=self.ideal[0], y_ideal=self.ideal[1], th_ideal=self.ideal[2],
                   ref1=refs[0], ref2=refs[1], ref3=refs[2], u1=u[0], u2=u[1], u3=u[2],
                   ff2=c2[0], p2=c2[1], i2=c2[2], ff3=c3[0], p3=c3[1], i3=c3[2])
        self.rows.append(row)
        self.live = row
        if not moving and self.t_end is None:
            self.t_end = t
        if self.t_end is not None and t > self.t_end + config.REST_AFTER_S:
            self._finish("completada")

    def _finish(self, result):
        if self.phase == "done":
            return
        self.phase = "done"
        self.result = result
        self.link.unsubscribe(self.on_sample)
        self.link.stop()
        try:
            self.save()
        finally:
            self.finished.set()

    # ── output ───────────────────────────────────────────────────────
    def save(self):
        os.makedirs(self.dir, exist_ok=True)
        with open(os.path.join(self.dir, "data.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=COLS, extrasaction="ignore")
            w.writeheader()
            for r in self.rows:
                w.writerow({k: (round(v, 5) if isinstance(v, float) else v) for k, v in r.items()})
        self.summary = _finite(dict(summarize(self.rows, self.ref), run_id=self.run_id, kind=self.kind,
                                    params=self.params, ctrl=self.ctrl, mode=self.mode, gains=self.gains, label=self.label, settings=self.st,
                                    result=self.result, real=not getattr(self.link, "virtual", False),
                                    date=time.strftime("%Y-%m-%dT%H:%M:%S")))
        save_json(os.path.join(self.dir, "summary.json"), self.summary)
        if self.rows:
            import plots
            plots.plot_run(self.rows, self.summary, self.dir)


def summarize(rows, ref):
    if not rows:
        return {}
    c = {k: np.array([r[k] for r in rows], dtype=float) for k in rows[0] if isinstance(rows[0][k], (int, float))}
    mov = c["tau"] <= ref.T if "tau" in c else c["t"] <= ref.T
    out = {"duration_s": float(c["t"][-1])}
    if "scale" in c:
        out["traction_min_scale"] = float(c["scale"].min())
        out["traction_active_pct"] = float(100 * (c["scale"] < 0.99).mean())
        out["slip_max"] = float(np.nanmax(c["slip"]))
    for i in (1, 2, 3):
        e = (c[f"ref{i}"] - c[f"rpm{i}"])[mov]
        act = np.abs(c[f"ref{i}"][mov]) > 5
        out[f"wheel{i}_err_rms_rpm"] = float(np.sqrt(np.mean(e ** 2)))
        # steady state: reference constant during the last 0.3 s
        r = c[f"ref{i}"]
        steady = mov & (np.abs(np.gradient(r)) < 1e-3) & (np.abs(r) > 5)
        steady &= np.convolve(steady, np.ones(15) / 15, mode="same") > 0.99
        out[f"wheel{i}_ss_err_rpm"] = float(np.mean(e[steady[mov]])) if steady.any() else None
        out[f"wheel{i}_ref_mean_abs"] = float(np.mean(np.abs(r[mov][act]))) if act.any() else 0.0
    sat = np.abs(np.stack([c["pwm1"], c["pwm2"], c["pwm3"]])) >= config.PWM_MAX - 1
    out["pwm_saturated_pct"] = float(100 * sat[:, mov].any(axis=0).mean())
    out["v_err_rms"] = float(np.sqrt(np.mean((c["v_ref"] - c["v"])[mov] ** 2)))
    out["w_err_rms"] = float(np.sqrt(np.mean((c["w_ref"] - c["w"])[mov] ** 2)))
    out["v_lat_rms"] = float(np.sqrt(np.mean(c["v_lat"][mov] ** 2)))
    # trajectory: final point and distance to the ideal path
    P = np.stack([c["x_ideal"], c["y_ideal"]], 1)
    Q = np.stack([c["x"], c["y"]], 1)
    d = np.array([np.min(np.hypot(*(P - q).T)) for q in Q[::5]])
    out["path_dev_rms_m"] = float(np.sqrt(np.mean(d ** 2)))
    out["path_dev_max_m"] = float(d.max())
    out["final_x"], out["final_y"] = float(c["x"][-1]), float(c["y"][-1])
    out["final_x_ideal"], out["final_y_ideal"] = float(c["x_ideal"][-1]), float(c["y_ideal"][-1])
    out["final_pos_err_m"] = float(math.hypot(c["x"][-1] - c["x_ideal"][-1], c["y"][-1] - c["y_ideal"][-1]))
    out["final_th_deg"] = float(math.degrees(c["th"][-1]))
    out["final_th_ideal_deg"] = float(math.degrees(c["th_ideal"][-1]))
    out["final_th_enc_deg"] = float(math.degrees(c["th_enc"][-1]))
    out["final_th_gyro_deg"] = float(math.degrees(c["th_gyro"][-1]))
    out["distance_m"] = float(np.sum(np.hypot(np.diff(c["x"]), np.diff(c["y"]))))
    return out


class FloorCalibration:
    """Feedforward calibration ON THE FLOOR: the robot rotates in place (the three wheels get the
    same speed), holding each level of config.CAL_RPM CCW and CW with a PID. The mean PWM of each
    wheel at each level gives the line PWM = offset + RPM / K, saved in settings.json."""

    def __init__(self, link):
        self.link = link
        self.phase, self.result = "rest", ""
        self.st = settings.load()
        self.ctrls = controller.make("PID", 0.3, 4.0, 0.0, True, self.st)
        self.levels = [(r, d) for r in config.CAL_RPM for d in (1, -1)]
        self.k, self.t_level, self.n = 0, None, 0
        self.acc = {lv: [] for lv in self.levels}
        self.est = estimator.Estimator()
        self.finished = threading.Event()
        self.points, self.fit = [], None
        self.rows, self.run_id = [], "calibration"

    def start(self):
        self.t0 = time.time()
        self.link.subscribe(self.on_sample)

    def request_stop(self, why="ABORTADA: STOP pulsado"):
        self.link.stop()
        self._finish(why)

    def on_sample(self, s):
        if self.phase == "done":
            return
        e = self.est.step(s)
        self.n += 1
        if time.time() - self.link.last_rx > config.TELEMETRY_TIMEOUT_S or time.time() - self.t0 > 30:
            self._finish("ABORTADA: telemetria o tiempo")
            return
        if self.n < 25:                                   # 0.5 s still
            self.link.raw_pwm(0, 0, 0)
            return
        self.phase = "calibrando"
        if self.t_level is None:
            self.t_level = s.t_ms
        rpm, d = self.levels[self.k]
        t = (s.t_ms - self.t_level) / 1000.0
        meas = (e["rpm1"], e["rpm2"], e["rpm3"])
        u = [c.step(rpm * d, m, config.DT) for c, m in zip(self.ctrls, meas)]
        self.link.raw_pwm(*[int(round(x)) for x in u])
        if t > config.CAL_HOLD_S / 2:
            self.acc[(rpm, d)].append((meas, s.pwm))
        if t > config.CAL_HOLD_S:
            self.k += 1
            self.t_level = s.t_ms
            for c in self.ctrls:
                c.reset()
            if self.k >= len(self.levels):
                self._compute()
                self._finish("completada")

    def _compute(self):
        fit = {"offset": [], "k": []}
        for i in range(3):
            pts = []
            for rpm in config.CAL_RPM:
                r = [abs(m[i]) for lv in ((rpm, 1), (rpm, -1)) for m, _ in self.acc[lv]]
                p = [abs(q[i]) for lv in ((rpm, 1), (rpm, -1)) for _, q in self.acc[lv]]
                pts.append((float(np.mean(r)), float(np.mean(p))))
            (r1, p1), (r2, p2) = pts
            k = (r2 - r1) / (p2 - p1) if p2 > p1 else config.MOTOR_K_RPM_PER_PWM
            fit["k"].append(round(k, 3))
            fit["offset"].append(round(max(0.0, p1 - r1 / k), 1))
            self.points.append({"wheel": i + 1, "points_rpm_pwm": pts})
        self.fit = fit
        settings.save({"ff_offset": fit["offset"], "ff_k": fit["k"]})
        log = load_json(os.path.join(BASE, "calibration_log.json")) or []
        log.append({"date": time.strftime("%Y-%m-%dT%H:%M:%S"), "points": self.points, "fit": fit,
                    "real": not getattr(self.link, "virtual", False)})
        save_json(os.path.join(BASE, "calibration_log.json"), log)

    def _finish(self, result):
        if self.phase == "done":
            return
        self.phase, self.result = "done", result
        self.link.unsubscribe(self.on_sample)
        self.link.stop()
        self.finished.set()
