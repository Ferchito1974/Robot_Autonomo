#!/usr/bin/env python3
"""
app.py - Web interface of AMR Assessment 1 Unit 3 (wheel velocity control + trajectories).

    ./start.sh        real robot (/dev/ttyACM0, firmware FutBot_fw_v5)
    ./start.sh demo   virtual robot (no hardware)
Open http://<pi>:5002 from the laptop.
"""
import collections
import math
import os
import threading
import time

from flask import Flask, jsonify, request, send_from_directory
from flask.json.provider import DefaultJSONProvider

import config
import experiment
import settings


def _finite(o):
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {k: _finite(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_finite(v) for v in o]
    return o


class FiniteJSONProvider(DefaultJSONProvider):
    def dumps(self, obj, **kw):
        return super().dumps(_finite(obj), **kw)


BASE = os.path.dirname(os.path.abspath(__file__))
app = Flask(__name__)
app.json = FiniteJSONProvider(app)
DEMO = os.environ.get("ROBOT_DEMO") == "1"
if DEMO:
    from fake_link import FakeLink
    link = FakeLink()
else:
    from link import RobotLink
    link = RobotLink()
run = None
connect_error = ""


class TelemetryMonitor:
    """Always-on telemetry view: sample rate and raw wheel speed (counts over 100 ms)."""

    def __init__(self):
        self.hist = collections.deque(maxlen=6)
        self.times = collections.deque(maxlen=60)

    def __call__(self, s):
        self.hist.append(s)
        self.times.append(time.time())

    def snapshot(self):
        if not self.hist:
            return None
        s, a = self.hist[-1], self.hist[0]
        dt = (s.t_ms - a.t_ms) / 1000.0
        rpm = [((s.enc[i] - a.enc[i]) / config.COUNTS_PER_REV / dt * 60.0) if dt > 0 else 0.0 for i in range(3)]
        span = self.times[-1] - self.times[0] if len(self.times) > 1 else 0
        return {"hz": (len(self.times) - 1) / span if span > 0 else 0.0, "t_ms": s.t_ms, "enc": s.enc, "pwm": s.pwm,
                "rpm_raw": [round(r, 1) for r in rpm], "gyro_z": s.gyro[2], "acc": s.acc, "yaw": s.yaw_deg,
                "gyro_age": s.gyro_age_ms, "acc_age": s.acc_age_ms, "yaw_age": s.yaw_age_ms, "mode": s.mode,
                "bad_lines": getattr(link, "bad_lines", 0)}


telmon = TelemetryMonitor()
lock = threading.Lock()


def connect():
    global connect_error
    try:
        link.connect()
        link.telemetry(True, 20)
        link.subscribe(telmon)
        connect_error = ""
    except Exception as e:                       # noqa: BLE001
        connect_error = f"error de conexion: {e}"
        print(connect_error, flush=True)


@app.route("/")
def index():
    return send_from_directory(BASE, "ui.html")


@app.route("/runs/<path:p>")
def runs_files(p):
    return send_from_directory(experiment.RUNS, p)


@app.route("/api/status")
def api_status():
    live = run.rows[-400:] if run is not None else []
    step = max(1, len(live) // 200)
    keys = ("t", "x", "y", "x_ideal", "y_ideal", "v", "v_ref", "w", "w_ref", "ref1", "ref2", "ref3",
            "rpm1", "rpm2", "rpm3", "pwm1", "pwm2", "pwm3")
    runs = []
    for d in sorted(os.listdir(experiment.RUNS), reverse=True)[:30]:
        sm = experiment.load_json(os.path.join(experiment.RUNS, d, "summary.json"))
        if sm:
            runs.append({k: sm.get(k) for k in ("run_id", "label", "result", "kind", "ctrl", "gains",
                                                 "wheel2_err_rms_rpm", "wheel2_ss_err_rpm", "path_dev_rms_m",
                                                 "final_pos_err_m", "final_th_deg", "final_th_ideal_deg", "final_x",
                                                 "final_y", "final_th_enc_deg", "final_th_gyro_deg", "params", "mode")})
    s = link.last_sample
    return jsonify({
        "demo": DEMO, "connected": bool(link.connected), "error": connect_error,
        "telemetry_age": (time.time() - link.last_rx) if link.last_rx else None,
        "gyro_ok": (s.gyro_age_ms <= config.IMU_MAX_AGE_MS) if s else None,
        "running": run is not None and not run.finished.is_set(),
        "phase": run.phase if run else None, "result": run.result if run else "",
        "run_id": run.run_id if run else None,
        "full_path": [[r["x_ideal"], r["y_ideal"], r["x"], r["y"]] for r in (run.rows[::5] if run else [])],
        "live": [{k: r.get(k) for k in keys} for r in live[::step]],
        "runs": runs, "defaults": {"gains": config.GAINS, "traj": config.TRAJ, "traj_omni": config.TRAJ_OMNI},
        "settings": settings.load(), "limits": {"v": [config.MIN_V, config.MAX_V]},
        "profiles": config.PROFILES, "tel": telmon.snapshot(),
        "calibration": ({"phase": run.phase, "result": run.result, "fit": run.fit}
                        if isinstance(run, experiment.FloorCalibration) else None),
        "log": list(link.log)[-6:],
    })


@app.route("/api/connect", methods=["POST"])
def api_connect():
    if not link.connected:
        connect()
    return jsonify({"ok": link.connected, "error": connect_error})


@app.route("/api/run", methods=["POST"])
def api_run():
    global run
    d = request.get_json(force=True)
    with lock:
        if not link.connected:
            return jsonify({"ok": False, "error": "robot no conectado"})
        if run is not None and not run.finished.is_set():
            return jsonify({"ok": False, "error": "ya hay una corrida en curso"})
        kind, ctrl, mode = d.get("kind"), d.get("ctrl"), d.get("mode", "diff")
        table = config.TRAJ_OMNI if mode == "omni" else config.TRAJ
        if kind not in table or ctrl not in config.GAINS or mode not in ("diff", "omni"):
            return jsonify({"ok": False, "error": "trayectoria o controlador invalido"})
        try:
            params = {k: float(d["params"][k]) for k in table[kind]}
            g = d["gains"]
            gains = {"kp": float(g["kp"]), "ki": float(g.get("ki", 0)), "kd": float(g.get("kd", 0)),
                     "ff": bool(g.get("ff", True))}
        except (KeyError, TypeError, ValueError):
            return jsonify({"ok": False, "error": "parametros invalidos"})
        vals = list(params.values()) + [gains["kp"], gains["ki"], gains["kd"]]
        if not all(math.isfinite(v) for v in vals):
            return jsonify({"ok": False, "error": "valores no numericos"})
        if "v" in params and not config.MIN_V <= params["v"] <= config.MAX_V:
            return jsonify({"ok": False, "error": f"v entre {config.MIN_V} y {config.MAX_V} m/s"})
        if not (0 <= gains["kp"] <= 5 and 0 <= gains["ki"] <= 50 and 0 <= gains["kd"] <= 0.5):
            return jsonify({"ok": False, "error": "ganancias fuera de rango (kp 0-5, ki 0-50, kd 0-0.5)"})
        if kind == "circle" and not 0.1 <= params["radius"] <= 1.5:
            return jsonify({"ok": False, "error": "radio entre 0.1 y 1.5 m"})
        acc = d.get("acc")
        if acc is not None and not (isinstance(acc, (int, float)) and 0.1 <= acc <= 3):
            return jsonify({"ok": False, "error": "aceleracion entre 0.1 y 3 m/s2"})
        run = experiment.Experiment(link, kind, params, ctrl, gains, d.get("label", ""), demo=DEMO, mode=mode, acc=acc)
        run.start()
    return jsonify({"ok": True, "run_id": run.run_id})


@app.route("/api/settings", methods=["POST"])
def api_settings():
    d = request.get_json(force=True)
    try:
        vals = [d[k] for k in d if k != "heading_source"]
        flat = [float(x) for v in vals for x in (v if isinstance(v, list) else [v])]
        if not all(math.isfinite(x) and x >= 0 for x in flat):
            raise ValueError
        old = settings.load()
        st = settings.save(d)
    except (ValueError, TypeError):
        return jsonify({"ok": False, "error": "valores invalidos"})
    if not (st["traction"] in (0.0, 1.0) and 0.2 <= st["slip_on"] <= 5 and st["s_curve"] in (0.0, 1.0) and 0 <= st["ff_lookahead"] <= 0.3 and 0 <= st["k_head"] <= 15 and 0 <= st["ki_head"] <= 20 and 0.7 <= st["k_rot"] <= 1.4 and 0.7 <= st["k_lin"] <= 1.4 and 0.1 <= st["acc_max"] <= 3 and 0.5 <= st["alpha_max"] <= 20 and st["start_pwm"] <= 120
            and st["i_max"] <= 255 and st["rest_before_s"] <= 5 and all(0.5 <= k <= 5 for k in st["ff_k"])
            and all(o <= 120 for o in st["ff_offset"])):
        settings.save(old)                        # keep the previous valid values
        return jsonify({"ok": False, "error": "fuera de rango (k_rot y k_lin 0.7-1.4, acc 0.1-3, alpha 0.5-20, start<=120, imax<=255, "
                                              "K 0.5-5, offset<=120)", "settings": settings.load()})
    return jsonify({"ok": True, "settings": st})


@app.route("/api/settings/reset", methods=["POST"])
def api_settings_reset():
    if os.path.exists(settings.FILE):
        os.remove(settings.FILE)
    return jsonify({"ok": True, "settings": settings.load()})


@app.route("/api/krot", methods=["POST"])
def api_krot():
    """k_rot from the saved real runs: real rotation (gyroscope) / rotation of the wheel kinematics."""
    vals = []
    for d in os.listdir(experiment.RUNS):
        sm = experiment.load_json(os.path.join(experiment.RUNS, d, "summary.json"))
        if not sm or not sm.get("real") or sm.get("result") != "completada" or d.startswith("lifted"):
            continue
        g, e = sm.get("final_th_gyro_deg"), sm.get("final_th_enc_deg")
        if g and e and abs(e) > 150:
            used = (sm.get("settings") or {}).get("k_rot", 1.0)
            vals.append(g / e * used)
    if len(vals) < 2:
        return jsonify({"ok": False, "error": "hacen falta al menos 2 corridas reales con giro (circulo o cuadrado)"})
    k = float(sorted(vals)[len(vals) // 2])
    st = settings.save({"k_rot": round(k, 3)})
    return jsonify({"ok": True, "k_rot": st["k_rot"], "n": len(vals), "values": [round(v, 3) for v in sorted(vals)],
                    "settings": st})


@app.route("/api/odocal/apply", methods=["POST"])
def api_odocal_apply():
    """Odometry calibration from a calibration run.
    line: k_lin_new = k_lin_used * measured distance (tape) / odometry distance.
    spin: k_rot_new = k_rot_used * real angle / encoder angle; real angle = gyroscope or the angle typed."""
    d = request.get_json(force=True)
    sm = experiment.load_json(os.path.join(experiment.RUNS, os.path.basename(d.get("run_id", "")), "summary.json"))
    if not sm or sm.get("result") != "completada":
        return jsonify({"ok": False, "error": "corrida de calibracion no encontrada o incompleta"})
    used = sm.get("settings") or {}
    try:
        if sm["kind"] == "line":
            meas = float(d["measured"])
            odo = math.hypot(sm["final_x"], sm["final_y"])
            if not (0.05 < meas < 5 and odo > 0.05):
                raise ValueError
            new = {"k_lin": round(used.get("k_lin", 1.0) * meas / odo, 4)}
            info = f"odometria {odo:.3f} m, medido {meas:.3f} m"
        elif sm["kind"] == "spin":
            enc = sm["final_th_enc_deg"]
            real = float(d["measured"]) if d.get("measured") not in (None, "") else sm["final_th_gyro_deg"]
            if not (abs(enc) > 90 and math.isfinite(real) and abs(real) > 90):
                raise ValueError
            new = {"k_rot": round(used.get("k_rot", 1.0) * real / enc, 4)}
            info = f"encoders {enc:.1f} deg, real {real:.1f} deg ({'tecleado' if d.get('measured') else 'giroscopio'})"
        else:
            return jsonify({"ok": False, "error": "la corrida no es de calibracion (recta o giro)"})
    except (KeyError, TypeError, ValueError):
        return jsonify({"ok": False, "error": "valor medido invalido"})
    k = list(new.values())[0]
    if not 0.7 <= k <= 1.4:
        return jsonify({"ok": False, "error": f"factor fuera de rango ({k}); revisa la medicion"})
    st = settings.save(new)
    return jsonify({"ok": True, "settings": st, "new": new, "info": info})


@app.route("/api/calibrate", methods=["POST"])
def api_calibrate():
    global run
    with lock:
        if not link.connected:
            return jsonify({"ok": False, "error": "robot no conectado"})
        if run is not None and not run.finished.is_set():
            return jsonify({"ok": False, "error": "ya hay una corrida en curso"})
        run = experiment.FloorCalibration(link)
        run.start()
    return jsonify({"ok": True})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    link.stop()
    if run is not None and not run.finished.is_set():
        run.request_stop()
    return jsonify({"ok": True})


def shutdown(*_):
    try:
        link.stop()
    except Exception:                             # noqa: BLE001
        pass


if __name__ == "__main__":
    import atexit
    atexit.register(shutdown)
    connect()
    app.run(host=config.WEB_HOST, port=config.WEB_PORT, threaded=True)
