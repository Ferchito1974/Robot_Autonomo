"""plots.py - Figures of one experiment (saved next to data.csv)."""
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402
import numpy as np                # noqa: E402

WCOL = ["#7c3aed", "#dc2626", "#2563eb"]
WNAME = ["Wheel 1 (rear)", "Wheel 2 (front-right)", "Wheel 3 (front-left)"]


def col(rows, k):
    return np.array([r[k] if r[k] is not None else np.nan for r in rows], dtype=float)


def _save(fig, p):
    fig.tight_layout()
    fig.savefig(p, dpi=130)
    plt.close(fig)


def plot_run(rows, s, d):
    t = col(rows, "t")
    lab = s.get("label", "")
    # 1. wheels: reference vs measured, PWM
    fig, axs = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    for i in range(3):
        axs[i].plot(t, col(rows, f"ref{i + 1}"), "k--", lw=1.1, label="reference")
        axs[i].plot(t, col(rows, f"rpm{i + 1}"), color=WCOL[i], lw=1.5, label="measured (Kalman)")
        axs[i].set_ylabel(f"{WNAME[i]}\n[RPM]", fontsize=9); axs[i].grid(alpha=.3); axs[i].legend(fontsize=8, loc="upper right")
        axs[3].plot(t, col(rows, f"pwm{i + 1}"), color=WCOL[i], lw=1.2, label=f"PWM {i + 1}")
    axs[3].axhline(255, color="0.5", ls=":"); axs[3].axhline(-255, color="0.5", ls=":")
    axs[3].set_ylabel("PWM"); axs[3].set_xlabel("Time [s]"); axs[3].grid(alpha=.3); axs[3].legend(fontsize=8, ncol=3)
    axs[0].set_title(f"Wheel speeds, reference vs measured: {lab}")
    _save(fig, os.path.join(d, "wheels.png"))
    # 2. robot velocities
    if s.get("mode") == "omni":
        fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
        for a, k, n in ((axs[0], "x", "Vx (world)"), (axs[1], "y", "Vy (world)")):
            a.plot(t, col(rows, f"v{k}_ref"), "k--", lw=1.1, label=f"{n} reference")
            a.plot(t, col(rows, f"v{k}w"), color="#16a34a", lw=1.5, label=f"{n} measured")
            a.set_ylabel(f"{n} [m/s]")
        axs[2].plot(t, col(rows, "w_cmd"), "k--", lw=1.1, label="ω command (heading hold)")
        axs[2].plot(t, col(rows, "w_gyro"), color="#dc2626", lw=1.0, label="ω gyroscope")
        axs[2].set_ylabel("ω [rad/s]"); axs[2].set_xlabel("Time [s]")
        for a in axs:
            a.grid(alpha=.3); a.legend(fontsize=8, loc="upper right")
        axs[0].set_title(f"World-frame velocity, omni mode: {lab}")
        _save(fig, os.path.join(d, "velocity.png"))
        _xy_and_heading(rows, t, lab, d)
        return
    fig, axs = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)
    axs[0].plot(t, col(rows, "v_ref"), "k--", lw=1.1, label="v reference")
    axs[0].plot(t, col(rows, "v"), color="#16a34a", lw=1.5, label="v measured (encoders)")
    axs[0].plot(t, col(rows, "v_lat"), color="#f59e0b", lw=1, label="lateral velocity (should be 0)")
    axs[0].set_ylabel("Linear velocity [m/s]")
    axs[1].plot(t, col(rows, "w_ref"), "k--", lw=1.1, label="ω reference")
    axs[1].plot(t, col(rows, "w_enc"), color="#2563eb", lw=1.3, label="ω encoders")
    axs[1].plot(t, col(rows, "w_gyro"), color="#dc2626", lw=1.0, alpha=.8, label="ω gyroscope")
    axs[1].set_ylabel("Angular velocity [rad/s]"); axs[1].set_xlabel("Time [s]")
    for a in axs:
        a.grid(alpha=.3); a.legend(fontsize=8, loc="upper right")
    axs[0].set_title(f"Robot linear and angular velocity: {lab}")
    _save(fig, os.path.join(d, "velocity.png"))
    _xy_and_heading(rows, t, lab, d)


def _xy_and_heading(rows, t, lab, d):
    # 3. trajectory
    fig, ax = plt.subplots(figsize=(6.2, 6.2))
    ax.plot(col(rows, "x_ideal"), col(rows, "y_ideal"), "k--", lw=1.2, label="ideal (references)")
    ax.plot(col(rows, "x"), col(rows, "y"), color="#16a34a", lw=2, label="odometry")
    ax.plot(0, 0, "ko", ms=5)
    ax.set_aspect("equal"); ax.grid(alpha=.3); ax.legend(fontsize=8)
    ax.set_xlabel("X [m] (initial forward direction)"); ax.set_ylabel("Y [m]")
    ax.set_title(f"Estimated trajectory: {lab}")
    _save(fig, os.path.join(d, "xy.png"))
    # 4. orientation
    fig, ax = plt.subplots(figsize=(9, 3.6))
    ax.plot(t, np.degrees(col(rows, "th_ideal")), "k--", lw=1.1, label="ideal")
    ax.plot(t, np.degrees(col(rows, "th_enc")), color="#2563eb", lw=1.4, label="encoders")
    ax.plot(t, np.degrees(col(rows, "th_gyro")), color="#dc2626", lw=1.4, label="gyroscope")
    ax.set_ylabel("Orientation θ [deg]"); ax.set_xlabel("Time [s]"); ax.grid(alpha=.3); ax.legend(fontsize=8)
    ax.set_title(f"Robot orientation: {lab}")
    _save(fig, os.path.join(d, "orientation.png"))
