#!/usr/bin/env python3
"""
build_report.py - Technical report (PDF) of AMR Assessment 1 Unit 3: wheel velocity control
and trajectory generation with the 3-wheel FutBot Omni. Same format as the AMR Final Test
Unit 2 report (cover page with logo, serif text, grid tables).

All numbers in the tables and figures are computed from the saved runs (../runs).

Usage (from tarea_velocidad_u3/):
    ../tarea_odometria/venv/bin/python reporte/build_report.py
Output: reporte/AMR_ASS1_U3_Fernando_Euan.pdf
"""
import csv
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
RUNS = os.path.join(PROJ, "runs")
sys.path.insert(0, PROJ)

import matplotlib                                                    # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                      # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Rectangle  # noqa: E402

import config                                                        # noqa: E402
import trajectories                                                  # noqa: E402

from reportlab.lib import colors                                     # noqa: E402
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY                # noqa: E402
from reportlab.lib.pagesizes import letter                           # noqa: E402
from reportlab.lib.styles import ParagraphStyle                      # noqa: E402
from reportlab.lib.units import cm                                   # noqa: E402
from reportlab.lib.utils import ImageReader                          # noqa: E402
from reportlab.pdfbase import pdfmetrics                             # noqa: E402
from reportlab.pdfbase.pdfmetrics import registerFontFamily          # noqa: E402
from reportlab.pdfbase.ttfonts import TTFont                         # noqa: E402
from reportlab.platypus import (CondPageBreak, Image, KeepTogether, PageBreak, Paragraph,  # noqa: E402
                                SimpleDocTemplate, Spacer, Table, TableStyle)

# ── cover data ───────────────────────────────────────────────────────────
TITLE = "Velocity Control and Trajectory Generation for an Omnidirectional Mobile Robot"
COURSE = "Advanced Mobile Robotics"
GROUP = "Group 9A"
PROFESSOR = "Professor: Adrián Puerto"
AUTHORS = ["Enrique de Jesus Narvaez Varguez", "Fernando Patricio Euan Maas", "Paola Sofia Rivera Medina",
           "Rennata Gonzalez Ruiz", "Salvador Camarillo Rivera", "Samuel Diaz Ríos"]
DATE = "November 25, 2026"
FIG = os.path.join(HERE, "fig")
OUT = os.path.join(HERE, "AMR_ASS1_U3_Fernando_Euan.pdf")
LOGO = os.path.join(HERE, "logo_upy.png")
os.makedirs(FIG, exist_ok=True)

FD = "/usr/share/fonts/truetype/liberation2/"
for n, f in (("TR", "LiberationSerif-Regular.ttf"), ("TR-B", "LiberationSerif-Bold.ttf"),
             ("TR-I", "LiberationSerif-Italic.ttf"), ("TR-BI", "LiberationSerif-BoldItalic.ttf"),
             ("MONO", "LiberationMono-Regular.ttf")):
    pdfmetrics.registerFont(TTFont(n, FD + f))
registerFontFamily("TR", normal="TR", bold="TR-B", italic="TR-I", boldItalic="TR-BI")
plt.rcParams.update({"font.family": "serif", "font.serif": ["Liberation Serif", "DejaVu Serif"],
                     "mathtext.fontset": "stix", "axes.titlesize": 11, "axes.labelsize": 10})

WCOL = ["#7c3aed", "#dc2626", "#2563eb"]
WNAME = ["Wheel 1 (rear)", "Wheel 2 (front-right)", "Wheel 3 (front-left)"]


# ── data ─────────────────────────────────────────────────────────────────
def load_json(p):
    with open(p) as f:
        return json.load(f)


def run(rid):
    """(summary, columns as numpy arrays) of a saved run."""
    d = os.path.join(RUNS, rid)
    s = load_json(os.path.join(d, "summary.json"))
    with open(os.path.join(d, "data.csv")) as f:
        rows = list(csv.DictReader(f))
    c = {}
    for k in rows[0]:
        try:
            c[k] = np.array([float(r[k]) if r[k] not in ("", "nan") else np.nan for r in rows])
        except ValueError:
            pass
    return s, c


def max_heading(c):
    return float(np.degrees(np.nanmax(np.abs(c["th"]))))


def wheel_err(s):
    return float(np.mean([s[f"wheel{i}_err_rms_rpm"] for i in (1, 2, 3)]))


# runs used in the report (all real, on the floor, 25-nov-2026)
R_LINE = "line_omni_PID_20261125_154748"
R_CIRCLE = "circle_omni_PID_20261125_160159"
R_SQUARE = "square_omni_PID_20261125_160230"
R_SQ_P = ["square_omni_P_20261125_155807", "square_omni_P_20261125_155817"]
R_SQ_PID = ["square_omni_PID_20261125_155938", "square_omni_PID_20261125_155950", "square_omni_PID_20261125_160001",
            "square_omni_PID_20261125_160020", "square_omni_PID_20261125_160035"]
R_CI_P = ["circle_omni_P_20261125_155758"]
R_CI_PID = ["circle_omni_PID_20261125_155840", "circle_omni_PID_20261125_155847", "circle_omni_PID_20261125_155917",
            "circle_omni_PID_20261125_155927"]
R_FF = "square_omni_PID_20261125_030221"
R_NOFF = ["square_omni_PID_20261125_030721", "square_omni_PID_20261125_030737"]
R_DIFF_BEFORE = "square_P_20261125_020754"
R_DIFF_AFTER = "square_P_20261125_022250"
R_SLIP = "circle_omni_PID_20261125_155256"
R_FAULT = "square_omni_PID_20261125_024535"
R_LIFT_P, R_LIFT_PID = "lifted_line_P_20261125_015638", "lifted_line_PID_20261125_015647"
R_TC_ON = {"square": R_SQ_PID, "circle": R_CI_PID}
R_TC_OFF = {"square": ["square_omni_PID_20261125_160208", "square_omni_PID_20261125_160218", R_SQUARE],
            "circle": ["circle_omni_PID_20261125_160151", R_CIRCLE]}


# ── figures ──────────────────────────────────────────────────────────────
def save(fig, name):
    p = os.path.join(FIG, name)
    fig.tight_layout()
    fig.savefig(p, dpi=170)
    plt.close(fig)
    return p


def fig_architecture():
    fig, ax = plt.subplots(figsize=(10, 3.0))
    ax.set_xlim(0, 10); ax.set_ylim(0, 3); ax.axis("off")
    boxes = [(0.1, "References", "line / circle / square\n(Vx, Vy) in the world\nS-curve profile", "#e0f2fe"),
             (2.1, "World → robot", "rotation by the\ngyroscope heading;\nheading PI (θ = 0)", "#dcfce7"),
             (4.1, "Inverse kinematics", "3-wheel omni:\n(vx, vy, ω) → RPM\nof the 3 wheels", "#dcfce7"),
             (6.1, "Wheel P / PID ×3", "feedforward + PID\n50 Hz, Python\n(Raspberry Pi 5)", "#fef3c7"),
             (8.1, "Robot", "Arduino Mega: PWM\nencoders, IMU BNO085\n(FutBot_fw_v5)", "#fee2e2")]
    for x, t, s, c in boxes:
        ax.add_patch(FancyBboxPatch((x, 0.95), 1.75, 1.55, boxstyle="round,pad=0.04", fc=c, ec="0.3"))
        ax.text(x + 0.875, 2.38, t, ha="center", va="top", fontsize=11, weight="bold")
        ax.text(x + 0.875, 1.92, s, ha="center", va="top", fontsize=9)
    for x in (1.85, 3.85, 5.85, 7.85):
        ax.add_patch(FancyArrowPatch((x, 1.72), (x + 0.25, 1.72), arrowstyle="-|>", mutation_scale=12))
    ax.add_patch(FancyArrowPatch((9.0, 0.95), (3.0, 0.95), connectionstyle="arc3,rad=0.3", arrowstyle="-|>",
                                 mutation_scale=12, color="#6b7280"))
    ax.text(5.0, 0.08, "feedback: Kalman filter per wheel (encoder counts) and gyroscope heading; odometry for the logs",
            ha="center", fontsize=8.5, color="#374151")
    return save(fig, "architecture.png")


def fig_geometry():
    fig, ax = plt.subplots(figsize=(5.4, 5.4))
    ax.add_patch(Circle((0, 0), 1.25, fc="#f3f4f6", ec="0.4"))
    for i, a_deg in enumerate(config.WHEEL_ANGLES_DEG):
        a = math.radians(a_deg)
        px, py = math.cos(a), math.sin(a)
        ax.add_patch(Rectangle((px - 0.32, py - 0.1), 0.64, 0.2, fc="0.25",
                               transform=matplotlib.transforms.Affine2D().rotate_deg_around(px, py, a_deg + 90)
                               + ax.transData))
        d = (-math.sin(a), math.cos(a))          # positive tangential direction (CCW)
        ax.add_patch(FancyArrowPatch((px, py), (px + 0.5 * d[0], py + 0.5 * d[1]), arrowstyle="-|>",
                                     mutation_scale=13, color=WCOL[i], lw=1.8))
        ax.text(1.72 * px, 1.72 * py, f"Wheel {i + 1}\n{a_deg:.0f}°", ha="center", va="center", fontsize=9.5,
                color=WCOL[i])
        ax.plot([0, px], [0, py], color="0.6", ls=":", lw=1)
    ax.add_patch(FancyArrowPatch((0, 0), (0, 0.75), arrowstyle="-|>", mutation_scale=14, color="#16a34a", lw=2))
    ax.text(0.06, 0.5, "+vy (front, camera)", color="#16a34a", fontsize=8.5)
    ax.add_patch(FancyArrowPatch((0, 0), (0.75, 0), arrowstyle="-|>", mutation_scale=14, color="#f59e0b", lw=2))
    ax.text(0.35, -0.16, "+vx", color="#f59e0b", fontsize=8.5)
    ax.text(-0.7, -0.35, "L = 95 mm", fontsize=8.5, color="0.35")
    ax.text(-2.05, -2.05, "Colored arrows: positive direction of each wheel (+ω, CCW)", fontsize=8, color="0.3")
    ax.set_xlim(-2.1, 2.1); ax.set_ylim(-2.2, 2.1); ax.set_aspect("equal"); ax.axis("off")
    return save(fig, "geometry.png")


def fig_references():
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.6))
    p = dict(config.TRAJ_OMNI["square"], v=0.75)
    for smooth, ls, lab in ((False, "--", "trapezoid (linear ramps)"), (True, "-", "S-curve ramps")):
        ref = trajectories.OmniReference("square", p, 1.8, smooth)
        t = np.arange(0, ref.T + 0.3, 0.005)
        v = np.array([ref.at(x) for x in t])
        axs[0].plot(t, np.hypot(v[:, 0], v[:, 1]), ls=ls, lw=1.5, label=lab)
        if smooth:
            axs[1].plot(t, v[:, 0], color="#2563eb", label="Vx reference")
            axs[1].plot(t, v[:, 1], color="#dc2626", label="Vy reference")
    axs[0].set_xlim(0, 2.2); axs[0].set_xlabel("Time [s]"); axs[0].set_ylabel("Speed [m/s]")
    axs[0].set_title("First side of the square (0.75 m/s, 1.8 m/s² peak)"); axs[0].legend(fontsize=8)
    axs[1].set_xlabel("Time [s]"); axs[1].set_ylabel("World velocity [m/s]")
    axs[1].set_title("Omni square: four sides in +X, +Y, −X, −Y"); axs[1].legend(fontsize=8)
    for a in axs:
        a.grid(alpha=.3)
    return save(fig, "references.png")


def fig_lifted():
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.4), sharey=True)
    for ax, rid, lab in ((axs[0], R_LIFT_P, "P (Kp 0.4) + FF"), (axs[1], R_LIFT_PID, "PID (Kp 0.3, Ki 4) + FF")):
        s, c = run(rid)
        for i, col in ((2, WCOL[1]), (3, WCOL[2])):
            ax.plot(c["t"], c[f"ref{i}"], "k--", lw=1)
            ax.plot(c["t"], c[f"rpm{i}"], color=col, lw=1.5, label=f"wheel {i} measured")
        ax.set_title(f"{lab}: steady-state error {s['wheel2_ss_err_rpm']:.1f} / {s['wheel3_ss_err_rpm']:.1f} RPM")
        ax.set_xlabel("Time [s]"); ax.grid(alpha=.3); ax.legend(fontsize=8)
    axs[0].set_ylabel("Wheel speed [RPM] (dashed: reference)")
    return save(fig, "lifted.png")


def fig_experiment(rid, name):
    s, c = run(rid)
    t = c["t"]
    fig = plt.figure(figsize=(11, 8.2))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.35, 1])
    for i in range(3):
        a = fig.add_subplot(gs[i, 0])
        a.plot(t, c[f"ref{i + 1}"], "k--", lw=1.1, label="reference")
        a.plot(t, c[f"rpm{i + 1}"], color=WCOL[i], lw=1.5, label="measured")
        a.set_ylabel(f"{WNAME[i]}\n[RPM]", fontsize=9); a.grid(alpha=.3); a.legend(fontsize=7.5, loc="upper right")
        if i == 0:
            a.set_title("Wheel speeds: reference vs measured")
        if i == 2:
            a.set_xlabel("Time [s]")
    a = fig.add_subplot(gs[0:2, 1])
    a.plot(c["x_ideal"], c["y_ideal"], "k--", lw=1.2, label="ideal (references)")
    a.plot(c["x"], c["y"], color="#16a34a", lw=2, label="odometry")
    a.plot(0, 0, "ko", ms=4)
    if name == "line":
        a.set_ylim(-0.25, 0.25); a.set_xlim(-0.05, 0.9)
    else:
        a.set_aspect("equal")
    a.grid(alpha=.3); a.legend(fontsize=8); a.set_title("Estimated trajectory")
    a.set_xlabel("X [m]"); a.set_ylabel("Y [m]")
    a = fig.add_subplot(gs[2, 1])
    a.plot(t, c["vx_ref"], "k--", lw=1)
    a.plot(t, c["vxw"], color="#2563eb", lw=1.3, label="Vx")
    a.plot(t, c["vy_ref"], "k:", lw=1)
    a.plot(t, c["vyw"], color="#dc2626", lw=1.3, label="Vy")
    a.set_ylabel("World velocity [m/s]"); a.set_xlabel("Time [s]"); a.grid(alpha=.3); a.legend(fontsize=7.5)
    a.set_title("Linear velocity (dashed: reference)")
    p1 = save(fig, f"exp_{name}.png")
    fig, ax = plt.subplots(figsize=(11, 2.9))
    ax.plot(t, np.degrees(c["th_gyro"]), color="#dc2626", lw=1.4, label="orientation θ (gyroscope)")
    ax.plot(t, np.degrees(c["th_enc"]), color="#2563eb", lw=1.2, label="orientation θ (encoders)")
    ax.axhline(0, color="k", ls="--", lw=1, label="reference θ = 0")
    ax2 = ax.twinx()
    ax2.plot(t, c["w_gyro"], color="0.55", lw=0.9, label="ω gyroscope")
    ax2.set_ylabel("ω [rad/s]", color="0.4")
    ax.set_ylabel("θ [deg]"); ax.set_xlabel("Time [s]"); ax.grid(alpha=.3); ax.legend(fontsize=8, loc="upper left")
    ax2.legend(fontsize=8, loc="upper right"); ax.set_title("Orientation and angular velocity")
    p2 = save(fig, f"exp_{name}_heading.png")
    return s, c, p1, p2


def fig_p_vs_pid():
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.6))
    sp, cp = run(R_SQ_P[0])
    si, ci = run(R_SQ_PID[0])
    axs[0].plot(ci["x_ideal"], ci["y_ideal"], "k--", lw=1.1, label="ideal")
    axs[0].plot(cp["x"], cp["y"], color="#f59e0b", lw=1.8, label="P (Kp 0.4)")
    axs[0].plot(ci["x"], ci["y"], color="#16a34a", lw=1.8, label="PID (Kp 0.3, Ki 4)")
    axs[0].set_aspect("equal"); axs[0].grid(alpha=.3); axs[0].legend(fontsize=8)
    axs[0].set_xlabel("X [m]"); axs[0].set_ylabel("Y [m]"); axs[0].set_title("Square 0.6 m at 0.75 m/s")
    axs[1].plot(ci["t"], ci["ref1"], "k--", lw=1, label="reference")
    axs[1].plot(cp["t"], cp["rpm1"], color="#f59e0b", lw=1.4, label="P")
    axs[1].plot(ci["t"], ci["rpm1"], color="#16a34a", lw=1.4, label="PID")
    axs[1].set_xlim(2.0, 4.5); axs[1].set_xlabel("Time [s]"); axs[1].set_ylabel("Wheel 1 [RPM]")
    axs[1].set_title("Wheel 1 on the second side (+Y)"); axs[1].grid(alpha=.3); axs[1].legend(fontsize=8)
    return save(fig, "p_vs_pid.png")


def fig_ff():
    fig, ax = plt.subplots(figsize=(10, 3.4))
    s1, c1 = run(R_FF)
    s2, c2 = run(R_NOFF[0])
    ax.plot(c1["t"], c1["ref2"], "k--", lw=1, label="reference")
    ax.plot(c1["t"], c1["rpm2"], color="#16a34a", lw=1.5, label="PID with feedforward")
    ax.plot(c2["t"], c2["rpm2"], color="#dc2626", lw=1.5, label="PID without feedforward")
    ax.set_xlabel("Time [s]"); ax.set_ylabel("Wheel 2 [RPM]"); ax.grid(alpha=.3); ax.legend(fontsize=8)
    ax.set_title("Effect of the feedforward (omni square, 0.65 m/s)")
    return save(fig, "ff.png")


def fig_diff_vs_omni():
    fig, axs = plt.subplots(1, 3, figsize=(12, 4.2))
    for ax, rid, title in ((axs[0], R_DIFF_BEFORE, "Turning, before k_rot (0.3 m/s)"),
                           (axs[1], R_DIFF_AFTER, "Turning, with k_rot = 1.09 (0.5 m/s)"),
                           (axs[2], R_SQUARE, "Omni, heading fixed (0.75 m/s)")):
        s, c = run(rid)
        ax.plot(c["x_ideal"], c["y_ideal"], "k--", lw=1.1)
        ax.plot(c["x"], c["y"], color="#16a34a", lw=1.8)
        ax.set_aspect("equal"); ax.grid(alpha=.3); ax.set_xlabel("X [m]")
        ax.set_title(f"{title}\nfinal error {s['final_pos_err_m'] * 100:.1f} cm", fontsize=10)
    axs[0].set_ylabel("Y [m]")
    return save(fig, "diff_vs_omni.png")


def fig_slip():
    s, c = run(R_SLIP)
    fig, ax = plt.subplots(figsize=(10, 3.2))
    ax.plot(c["t"], np.degrees(c["th_gyro"]), color="#dc2626", lw=1.5, label="θ gyroscope (real rotation)")
    ax.plot(c["t"], np.degrees(c["th_enc"]), color="#2563eb", lw=1.5, label="θ from the wheel encoders")
    ax.axhline(0, color="k", ls="--", lw=1)
    ax.set_xlabel("Time [s]"); ax.set_ylabel("θ [deg]"); ax.grid(alpha=.3); ax.legend(fontsize=8)
    ax.set_title("Wheel slip in a circle (0.75 m/s): the wheels turn the robot −200°, the robot turns +60°")
    return save(fig, "slip.png")


def fig_fault():
    s, c = run(R_FAULT)
    fig, ax = plt.subplots(figsize=(10, 3.2))
    m = c["t"] < 4.8
    ax.plot(c["t"][m], c["ref3"][m], "k--", lw=1, label="wheel 3 reference")
    ax.plot(c["t"][m], c["rpm3"][m], color=WCOL[2], lw=1.5, label="wheel 3 measured")
    ax.plot(c["t"][m], c["pwm3"][m], color="#f59e0b", lw=1.2, label="wheel 3 PWM")
    ax.set_xlabel("Time [s]"); ax.set_ylabel("RPM / PWM"); ax.grid(alpha=.3); ax.legend(fontsize=8)
    ax.set_title("Encoder fault: positive PWM, negative measured speed (positive feedback)")
    return save(fig, "fault.png")


# ── document helpers (same format as the AMR Final Test Unit 2 report) ──
BODY = ParagraphStyle("b", fontName="TR", fontSize=11, leading=16.5, alignment=TA_JUSTIFY, spaceAfter=7)
H1 = ParagraphStyle("h1", parent=BODY, fontName="TR-B", fontSize=11.5, spaceBefore=12, spaceAfter=6, alignment=0,
                    keepWithNext=1)
H2 = ParagraphStyle("h2", parent=H1, fontSize=11, spaceBefore=8, spaceAfter=4)
CAP = ParagraphStyle("c", parent=BODY, fontSize=10, leading=13, alignment=TA_CENTER, spaceAfter=10)
CELL = ParagraphStyle("cell", parent=BODY, fontSize=9.5, leading=12, alignment=0, spaceAfter=0)
COVER = ParagraphStyle("cov", parent=BODY, fontSize=13, leading=20, alignment=TA_CENTER, spaceAfter=10)
EQ = ParagraphStyle("eq", parent=BODY, fontName="TR-I", alignment=TA_CENTER, spaceBefore=2, spaceAfter=8)
story, nfig, ntab = [], [0], [0]


def P(t, st=BODY):
    after_h1 = story and isinstance(story[-1], Paragraph) and story[-1].style is H1
    if st in (H1, H2) and not after_h1:
        story.append(CondPageBreak(6 * cm))
    story.append(Paragraph(t, st))


def bullets(items):
    for i in items:
        story.append(Paragraph(i, ParagraphStyle("bl", parent=BODY, leftIndent=20, bulletIndent=6, spaceAfter=4),
                               bulletText="•"))


def figure(path, cap, w=15.5):
    iw, ih = ImageReader(path).getSize()
    nfig[0] += 1
    story.append(KeepTogether([Spacer(1, 4), Image(path, width=w * cm, height=w * cm * ih / iw), Spacer(1, 4),
                               Paragraph(f"Figure {nfig[0]}. {cap}", CAP)]))
    return nfig[0]


def table(data, widths, cap):
    rows = [[Paragraph(str(c), CELL) for c in r] for r in data]
    t = Table(rows, colWidths=[w * cm for w in widths], hAlign="CENTER")
    t.setStyle(TableStyle([("GRID", (0, 0), (-1, -1), 0.6, colors.black),
                           ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e5e7eb")),
                           ("VALIGN", (0, 0), (-1, -1), "MIDDLE"), ("TOPPADDING", (0, 0), (-1, -1), 3),
                           ("BOTTOMPADDING", (0, 0), (-1, -1), 3)]))
    ntab[0] += 1
    story.append(KeepTogether([Paragraph(f"Table {ntab[0]}. {cap}", CAP), t, Spacer(1, 10)]))
    return ntab[0]


def f(x, d=1):
    return "-" if x is None or (isinstance(x, float) and not math.isfinite(x)) else f"{x:.{d}f}"


def rng(vals, d=1, k=1.0):
    vals = [v * k for v in vals]
    lo, hi = min(vals), max(vals)
    return f"{lo:.{d}f}" if abs(hi - lo) < 10 ** (-d) else f"{lo:.{d}f} to {hi:.{d}f}"


def stats(rids):
    out = {"fin": [], "dev": [], "th": [], "we": [], "T": []}
    for r in rids:
        s, c = run(r)
        out["fin"].append(s["final_pos_err_m"])
        out["dev"].append(s["path_dev_rms_m"])
        out["th"].append(max_heading(c))
        out["we"].append(wheel_err(s))
        out["T"].append(s["duration_s"])
    return out


# ── cover ────────────────────────────────────────────────────────────────
story.append(Spacer(1, 1.2 * cm))
iw, ih = ImageReader(LOGO).getSize()
story.append(Image(LOGO, width=6.2 * cm, height=6.2 * cm * ih / iw, hAlign="LEFT"))
story.append(Spacer(1, 2.2 * cm))
for line in ("Universidad Politécnica de Yucatán", COURSE, GROUP, PROFESSOR):
    P(line, COVER)
story.append(Spacer(1, 0.8 * cm))
for a in AUTHORS:
    P(a, ParagraphStyle("au", parent=COVER, spaceAfter=4))
story.append(Spacer(1, 0.8 * cm))
P("AMR Assessment 1 Unit 3: Velocity Control and Trajectory Generation", COVER)
story.append(Spacer(1, 0.4 * cm))
P(DATE, COVER)
story.append(PageBreak())

# ── body ─────────────────────────────────────────────────────────────────
P(f"Assessment 1 Unit 3: {TITLE}", ParagraphStyle("t", parent=BODY, fontName="TR-B", fontSize=13,
                                                    alignment=TA_CENTER, spaceAfter=6))
P("Advanced Mobile Robotics (AMR), Prof. Adrián Puerto. Three-wheel omnidirectional robot (FutBot Omni)",
  ParagraphStyle("st", parent=BODY, alignment=TA_CENTER, spaceAfter=14))

sl, cl = run(R_LINE)
sc, cc = run(R_CIRCLE)
ss, csq = run(R_SQUARE)
P("1. Objective and scope", H1)
P("The objective is to design and implement a velocity control system for the wheels of a mobile robot using "
  "encoder measurements and its kinematic model, and to use it to execute straight-line, circular and square "
  "trajectories from predefined linear and angular velocity references. As required, the focus is the regulation "
  "of the wheel speeds: the trajectories are executed from the references, without a trajectory-tracking "
  "controller with continuous position correction. A proportional (P) controller was implemented for each wheel "
  "and, as the additional challenge, a PID controller, and both were compared on the real robot.")
P("The platform is the team's three-wheel omnidirectional robot instead of a differential robot. The three "
  "wheels are controlled, and the reference generation uses the omnidirectional inverse kinematics, which is the "
  "three-wheel version of the differential rule of the assignment (Section 3.1). Because the robot is "
  "holonomic, the square is executed by translating along its four sides while the heading is kept at its "
  "initial value, so no turns in place are needed. The ESP32 of the assignment is replaced by an Arduino Mega "
  "2560, which only acquires the sensors and drives the motors; all calculations, control and data recording "
  "run in Python on a Raspberry Pi 5.")

P("2. System description", H1)
figure(fig_architecture(), "System architecture. The Arduino only reads the sensors and applies the PWM; the "
       "references, kinematics, wheel controllers, estimation and logging run in Python at 50 Hz.", w=16)
table([["Component", "Description"],
       ["Motors", "3 × BEMONOC 25GA370 12 V, 600 RPM, gearbox 1:10, Hall encoder 12 PPR"],
       ["Encoders", "120 counts per wheel revolution (rising edge of channel A, channel B gives the direction)"],
       ["Wheels", "Omnidirectional, at 270°, 30° and 150°, L = 95 mm from the center, diameter 38 mm (firmware)"],
       ["Driver", "Adafruit Motor Shield V2, PWM 0 to 255, LiPo 3S (11.1 V)"],
       ["IMU", "BNO085, gyroscope at 50 Hz (mounted upside down, Z sign corrected)"],
       ["Low level", "Arduino Mega 2560, firmware FutBot_fw_v5 (raw PWM command {PWM,p1,p2,p3})"],
       ["High level", "Raspberry Pi 5, Python 3.11 (numpy, matplotlib, Flask, pyserial)"]],
      [3.0, 13.0], "Hardware of the experimental platform.")
P("The Arduino streams one telemetry line every 20 ms with the cumulative encoder counts, the applied PWM and "
  "the IMU data. Every telemetry line triggers one control step in Python: estimation of the wheel speeds, "
  "computation of the references, wheel controllers and a new PWM command. The firmware stops the motors by "
  "itself if no command arrives for 500 ms. The experiments are run from a web interface on the laptop, which "
  "shows the plots and the telemetry in real time and stores every run (CSV, metrics and figures).")
P("<b>Safety.</b> A run is stopped if the STOP button is pressed, the telemetry is lost for 300 ms, an encoder "
  "jumps more than 60 counts in one sample, a wheel turns against its PWM for 0.15 s (encoder fault, Section 6.6) "
  "or a wheel stays saturated for 1 s. Before every run the robot stays still for 1.5 s to estimate the "
  "gyroscope bias.")

P("3. Mathematical models", H1)
P("3.1 Kinematic model", H2)
figure(fig_geometry(), "Wheel layout and robot frame. Each wheel is located at angle a<sub>i</sub> and pushes "
       "tangentially; +vy is the front of the robot.", w=8.5)
P("For a robot-frame velocity (v<sub>x</sub>, v<sub>y</sub>, ω), the tangential speed of wheel i is")
P("v<sub>i</sub> = −sin(a<sub>i</sub>) v<sub>x</sub> + cos(a<sub>i</sub>) v<sub>y</sub> + L ω,     "
  "a = 270°, 30°, 150°", EQ)
P("and the robot velocity is recovered from the three measured wheel speeds with the inverse of this 3 × 3 "
  "matrix (forward kinematics). The assignment computes the wheel references of a differential robot as "
  "v<sub>R,ref</sub> = v<sub>ref</sub> + ω<sub>ref</sub> b/2 and v<sub>L,ref</sub> = v<sub>ref</sub> − "
  "ω<sub>ref</sub> b/2: each wheel reference is the projection of the linear velocity on the direction of the "
  "wheel plus the contribution of the rotation. The omnidirectional equation has exactly the same structure "
  "with three wheels: for a forward motion (v<sub>y</sub> = v) the references are v<sub>1</sub> = Lω, "
  "v<sub>2</sub> = (√3/2) v + Lω and v<sub>3</sub> = −(√3/2) v + Lω, and for a lateral motion the rear wheel 1 "
  "does most of the work, so the three wheels are used.")
P("3.2 Motor model and loop delay", H2)
P("Each wheel was identified with PWM steps (wheels lifted) as a first-order system above a small dead zone, "
  f"rpm(s)/pwm(s) = K/(τs + 1) with <b>K = {config.MOTOR_K_RPM_PER_PWM} RPM/PWM</b> and <b>τ = "
  f"{config.MOTOR_TAU_S * 1000:.0f} ms</b>. On the floor the dead zone and the slope are obtained with a floor "
  "calibration in the interface: the robot rotates in place holding 80 and 160 RPM on the three wheels in both "
  "directions, and the mean PWM gives the line PWM = offset + RPM/K of each wheel (on the smooth floor used: "
  "offsets of 6 to 11 PWM, K of 2.2 to 2.4 RPM/PWM). The first test on the real robot showed that the PWM "
  "applied by the Arduino lags the PWM computed in Python by two samples (40 ms, USB and loop latency). With the "
  "lag of the motor and of the speed estimate this delay limits the usable gains (Section 4.3).")
P("3.3 Estimation and odometry", H2)
P("The wheel speed is estimated from the encoder counts with one Kalman filter per wheel (constant-velocity "
  "model, state = wheel angle and angular speed, measurement = quantized angle), as in the odometry assignment. "
  "The robot velocity is obtained with the forward kinematics and integrated in the world frame. The heading is "
  "measured with the gyroscope (bias removed at rest) and also integrated from the encoders for comparison. Two "
  "kinematic correction factors were calibrated: <b>k<sub>lin</sub> = 1.06</b> (real distance / distance of the "
  "wheels, from straight runs measured with a tape; it absorbs the uncertainty of the wheel diameter) and "
  "<b>k<sub>rot</sub> = 1.09</b> (real rotation measured by the gyroscope / rotation predicted by the wheels, "
  "median of six runs; the same 8 % found in the odometry assignment, caused by wheel slip).")

P("4. Velocity controller", H1)
P("4.1 Wheel controller", H2)
P("One controller per wheel runs at 50 Hz. The error is the difference between the reference speed of the "
  "wheel (from the inverse kinematics) and the speed estimated by the Kalman filter:")
P("e = RPM<sub>ref</sub> − RPM<sub>meas</sub>,     u = FF(RPM<sub>ref</sub>(t + t<sub>a</sub>)) + K<sub>p</sub> e "
  "+ K<sub>i</sub> ∫ e dt − K<sub>d</sub> dRPM<sub>meas</sub>/dt,     |u| ≤ 255", EQ)
bullets([
    "<b>P controller</b> (required): K<sub>i</sub> = K<sub>d</sub> = 0, K<sub>p</sub> = 0.4 PWM/RPM.",
    "<b>PID controller</b> (additional challenge): K<sub>p</sub> = 0.3, K<sub>i</sub> = 4, K<sub>d</sub> = 0, "
    "with anti-windup (the integral is frozen while the output saturates, |I| ≤ 120 PWM).",
    "<b>Feedforward</b> FF = sign · (offset + |RPM|/K), the static motor model from the floor calibration, so "
    "the controller only corrects the residual. It uses the reference t<sub>a</sub> = 0.1 s ahead to compensate "
    "the actuation delay and the motor lag (the references are known in advance).",
    "<b>Start-up kick</b>: a stopped wheel with a reference receives at least 35 PWM (static friction).",
])
P("4.2 Gains in other conventions", H2)
P("The gains are expressed in PWM per RPM and the integral is computed as K<sub>i</sub> e dt, so K<sub>i</sub> "
  "is per second. The same PID written in other common forms is shown in Table 2. K<sub>i</sub> is larger than "
  "K<sub>p</sub> because the motor is fast: the integral time T<sub>i</sub> = K<sub>p</sub>/K<sub>i</sub> = 75 ms "
  "is of the order of the motor time constant and the loop delay, as recommended by the classic tuning rules.")
table([["Form", "Kp", "Ki", "Kd", "Notes"],
       ["Used (error in RPM, output in PWM, I += Ki·e·dt)", "0.3", "4", "0", "Ki in 1/s"],
       ["Discrete per sample (I += Ki·e, dt = 0.02 s)", "0.3", "0.08", "0", "Typical Arduino form"],
       ["Physical units (error in rad/s, output in V)", "0.125 V·s/rad", "1.66 V/rad", "0",
        "1 PWM = 0.0435 V, 1 rad/s = 9.55 RPM"],
       ["Standard form Kp (1 + 1/(Ti s) + Td s)", "0.3", "Ti = 0.075 s", "Td = 0", "Ti close to τ"]],
      [6.0, 2.3, 2.3, 1.3, 4.3], "Equivalent forms of the wheel PID used.")
P("4.3 Parameter selection", H2)
P("The first test on the real robot, with the wheels lifted and K<sub>p</sub> = 1.0, oscillated with errors "
  "of 138 to 202 RPM RMS and 23 to 37 % of saturation, because the 40 ms actuation delay plus the filter lag "
  "had not been modeled. The delay was added to a simulated robot (first-order motors, dead zone, "
  "omnidirectional kinematics) and the gains were tuned on it with a parameter sweep on the square trajectory: "
  "K<sub>p</sub> = 0.3 to 0.4 with K<sub>i</sub> = 2 to 4 gave the lowest wheel error, K<sub>i</sub> = 8 with "
  "K<sub>p</sub> = 0.5 became unstable, and K<sub>d</sub> did not improve the response because it differentiates "
  "a filtered and delayed measurement. The feedforward look-ahead reduced the simulated wheel error from 22 to "
  f"7 RPM RMS. Figure {nfig[0] + 1} shows the retuned controllers on the real robot with the wheels lifted: the P controller "
  "keeps a steady-state error (the floor feedforward pushes too much in the air) and the PID removes it.")
figure(fig_lifted(), "Retuned wheel controllers on the real robot, wheels lifted (straight-line references).", w=16)

P("5. Reference generation", H1)
P("The references are predefined velocity profiles. Each segment (a side of the square, the straight line or "
  "the circle) is a trapezoid of speed whose area is exactly the requested distance. The ramps are S-shaped "
  "(1 − cos), which limits the jerk: on the real robot the linear ramps made the wheels slip when starting each "
  "side (Section 6.5). In the omnidirectional mode the references are world velocities (V<sub>x</sub>, "
  "V<sub>y</sub>); at every step they are rotated to the robot frame with the gyroscope heading, and a PI on the "
  "heading (K<sub>p</sub> = 4, K<sub>i</sub> = 2, output limited to 2 rad/s) keeps the orientation at its "
  "initial value (0°). Without it, the small differences between the wheels make the robot rotate slowly "
  "during the run.")
bullets([
    "<b>Straight line</b>: V<sub>x</sub> = v during distance / v (0.8 m).",
    "<b>Circle</b>: speed v tangent to a circle of radius R = 0.35 m (ω of the path = v/R), heading kept at 0.",
    "<b>Square</b>: four sides of 0.6 m in the directions 0°, 90°, 180° and 270°, with a short pause at each "
    "corner and no turns.",
])
figure(fig_references(), "Speed profile of a side (linear vs S-curve ramps) and world-velocity references of "
       "the omnidirectional square.", w=16)
table([["Profile", "Speed [m/s]", "Peak acceleration [m/s²]", "Use"]] +
      [[p["name"], f"{p['v']:.2f}", f"{p['acc']:.1f}", u] for p, u in
       zip(config.PROFILES, ["Floors with more slip", "", "Experiments of this report", "Smooth floor"])],
      [4.6, 2.6, 4.2, 4.6], "Speed profiles available in the interface.")

P("6. Experiments with the physical robot", H1)
P("All experiments were made on the real robot on a smooth floor, after the floor calibration, with the "
  "experiment speed of 0.75 m/s (profile 3). Positions are the odometry estimate (encoders with the "
  f"gyroscope heading). The three required movements are shown first with the PID (Sections 6.1 to 6.3), then "
  "the comparison with the P controller and the other tests. Photographs and videos of the experiments are "
  "submitted separately with the assignment.")
for sec, rid, name, s, c, desc in (
        ("6.1 Straight line", R_LINE, "line", sl, cl, "straight line of 0.8 m"),
        ("6.2 Circle", R_CIRCLE, "circle", sc, cc, "circle of radius 0.35 m (one turn)"),
        ("6.3 Square", R_SQUARE, "square", ss, csq, "square of 0.6 m")):
    P(sec, H2)
    _, _, p1, p2 = fig_experiment(rid, name)
    figure(p1, f"Experiment {sec[:3]}: {desc} at 0.75 m/s with the PID (run {rid[-15:]}). Wheel speeds, "
           "estimated trajectory and world velocities.", w=16)
    figure(p2, f"Orientation and angular velocity of the same run (reference: θ = 0).", w=16)
    errs = [s[f"wheel{i}_err_rms_rpm"] for i in (1, 2, 3)]
    if name == "line":
        P(f"The robot travels the 0.8 m in {s['duration_s'] - config.REST_AFTER_S:.1f} s. Wheels 2 and 3 follow "
          "their references (opposite signs, as given by the kinematics) and wheel 1 stays near zero, with small "
          "corrections from the heading control. The wheel errors are "
          f"{errs[0]:.0f}, {errs[1]:.0f} and {errs[2]:.0f} RPM RMS and come from the ramps; on the plateau the "
          f"measured speed matches the reference. The final position is {s['final_pos_err_m'] * 100:.1f} cm from the "
          f"ideal end point and the heading stays within {max_heading(c):.0f}° (the largest deviation appears "
          "when accelerating).")
    elif name == "circle":
        P("In the circle the three wheel references change continuously (sinusoids with a 120° phase shift), "
          "because the direction of motion rotates while the robot keeps its orientation. The controllers follow "
          f"them with {errs[0]:.0f}, {errs[1]:.0f} and {errs[2]:.0f} RPM RMS of error; the error is mostly a "
          "small phase lag. The estimated path stays on the circle (cross-track RMS "
          f"{s['path_dev_rms_m'] * 100:.1f} cm), the robot closes the circle {s['final_pos_err_m'] * 100:.1f} cm "
          f"from the start point and the heading stays within {max_heading(c):.0f}°.")
    else:
        P("The square is executed only with translations: on the ±X sides wheels 2 and 3 work and wheel 1 is "
          "nearly still, on the ±Y sides wheel 1 drives at full speed and wheels 2 and 3 at half speed in the same "
          "direction. The corners are sharp because the robot does not need to turn. The wheel errors are "
          f"{errs[0]:.0f}, {errs[1]:.0f} and {errs[2]:.0f} RPM RMS, the path deviation "
          f"{s['path_dev_rms_m'] * 100:.1f} cm RMS and the final error {s['final_pos_err_m'] * 100:.1f} cm; the "
          f"heading stays within {max_heading(c):.0f}°.")

P("6.4 P controller vs PID (additional challenge)", H2)
sp_, spid, cp_, cpid = stats(R_SQ_P), stats(R_SQ_PID), stats(R_CI_P), stats(R_CI_PID)
table([["Trajectory (0.75 m/s)", "Controller", "Runs", "Wheel error RMS [RPM]", "Path deviation RMS [cm]",
        "Final error [cm]", "Max heading [deg]"],
       ["Square", "P (Kp 0.4)", len(R_SQ_P), rng(sp_["we"], 0), rng(sp_["dev"], 1, 100), rng(sp_["fin"], 1, 100),
        rng(sp_["th"], 0)],
       ["Square", "PID (Kp 0.3, Ki 4)", len(R_SQ_PID), rng(spid["we"], 0), rng(spid["dev"], 1, 100),
        rng(spid["fin"], 1, 100), rng(spid["th"], 0)],
       ["Circle", "P (Kp 0.4)", len(R_CI_P), rng(cp_["we"], 0), rng(cp_["dev"], 1, 100), rng(cp_["fin"], 1, 100),
        rng(cp_["th"], 0)],
       ["Circle", "PID (Kp 0.3, Ki 4)", len(R_CI_PID), rng(cpid["we"], 0), rng(cpid["dev"], 1, 100),
        rng(cpid["fin"], 1, 100), rng(cpid["th"], 0)]],
      [2.6, 3.0, 1.3, 2.4, 2.5, 2.0, 2.2], "P vs PID on the real robot, same references and settings "
      "(both with feedforward).")
figure(fig_p_vs_pid(), "P vs PID: estimated square and speed of wheel 1 on the second side.", w=16)
P("With both controllers the wheels follow the references closely, because the feedforward provides most of "
  "the PWM. The difference is the steady-state error: the P controller can only correct the error that remains "
  "(a residual error of the feedforward model always remains), so each wheel runs a few RPM slower or faster "
  "than requested and the robot drifts, which accumulates along the sides. The integral of the PID removes this "
  f"error: the path deviation of the square drops from {rng(sp_['dev'], 1, 100)} cm to "
  f"{rng(spid['dev'], 1, 100)} cm and the final error from {rng(sp_['fin'], 1, 100)} cm to "
  f"{rng(spid['fin'], 1, 100)} cm.")

P("6.5 Feedforward, speed and wheel slip", H2)
s_ff, s_nff = stats([R_FF]), stats(R_NOFF)
table([["PID (omni square, 0.65 m/s)", "Wheel error RMS [RPM]", "Path deviation RMS [cm]", "Final error [cm]"],
       ["With feedforward", rng(s_ff["we"], 0), rng(s_ff["dev"], 1, 100), rng(s_ff["fin"], 1, 100)],
       ["Without feedforward", rng(s_nff["we"], 0), rng(s_nff["dev"], 1, 100), rng(s_nff["fin"], 1, 100)]],
      [5.5, 3.5, 3.5, 3.0], "Effect of the feedforward (same gains).")
figure(fig_ff(), "Wheel 2 with and without feedforward: without it the PID must build all the PWM from the error "
       "and the wheel lags about 140 ms behind the reference.", w=15)
P("The floor is smooth and the wheels slip when the robot accelerates strongly. The slip appears as a sudden "
  "rotation of the robot measured by the gyroscope that the wheel encoders do not see. Over all PID runs, the "
  "maximum heading deviation grew with the peak acceleration of the references: 5 to 17° with 1.5 to 2.0 m/s² "
  "and linear ramps, 28 to 44° with 2.2 to 2.5 m/s². The S-curve ramps reduced it to 16 to 24° at 2.2 m/s², and "
  f"with 1.8 m/s² the square stays within 6 to 17°. Figure {nfig[0] + 1} shows a circle in which the heading control, "
  "saturated at its 2 rad/s limit, made the wheels spin: the encoders indicate −200° of rotation while the robot "
  "only turned +60°. Reducing the heading correction while slip is detected (and freezing its integral) removed "
  "this behavior (circles within 6 to 11°). A traction control that slows the references down while the gyroscope "
  "and the encoders disagree was also tested; on this floor it gave no measurable improvement (square 7 to 17° "
  "with it, 6 to 10° without), so it is kept as an option.")
figure(fig_slip(), "Wheel slip: heading from the gyroscope vs heading from the encoders.", w=15)
P("At 0.95 m/s the wheel references reach 500 RPM and the motors saturated 30 % of the time, so the fastest "
  "profile was limited to 0.85 m/s.")
P("6.6 Rotating vs omnidirectional execution and encoder fault", H2)
figure(fig_diff_vs_omni(), "Square executed like a differential robot (turning 90° in place at each corner) "
       "before and after the rotation calibration, and with the omnidirectional execution.", w=16)
P("Executed like a differential robot, every turn in place rotated about 9 % more than predicted by the "
  "wheels, and the square did not close (36 cm of final error). The rotation factor k<sub>rot</sub> reduced "
  "the error to under 1 cm in the best runs, but the result still depended on the slip of each turn (7 to 30 cm "
  "in other runs). The omnidirectional execution avoids the turns, is faster and much more repeatable.")
P(f"During one run the speed measured on wheel 3 became negative while its PWM was positive (Figure {nfig[0] + 1}). The "
  "controller increased the PWM to correct a speed that seemed to go the wrong way, which is a positive "
  "feedback: the wheel saturated and the robot left the path. The most probable cause is an intermittent "
  "connection of the direction channel of that encoder. A protection was added that aborts the run if a wheel "
  "turns against its PWM for 0.15 s; replayed on all the saved runs it would only have stopped this one.")
figure(fig_fault(), "Encoder fault on wheel 3.", w=15)

P("7. Conclusions and limitations", H1)
P("A velocity control system for the three wheels of the omnidirectional robot was implemented in Python, with "
  "the Arduino acting only as the sensor and actuator interface. The wheel references are computed from the "
  "desired velocities with the omnidirectional kinematics, the speeds are measured with the encoders through a "
  "Kalman filter, and P and PID controllers with a calibrated feedforward regulate the wheels. The robot "
  "executed the straight line, the circle and the square from the velocity references with final errors of the "
  f"order of 1 cm and the heading within about 10 to 17°. The PID reduced the path deviation of the square about "
  "nine times with respect to the P controller, because it removes the steady-state error. The main parameters "
  "were the 40 ms actuation delay, which limits the gains, the feedforward, which provides most of the PWM, and "
  "the acceleration of the references, which determines how much the wheels slip.")
bullets([
    "The positions are the odometry estimate: if the wheels slip without the gyroscope noticing a rotation "
    "(pure translation slip), the estimate does not see it.",
    "The wheel diameter was not measured; the distance factor k<sub>lin</sub> = 1.06 calibrated with a tape "
    "absorbs it.",
    "The chassis slips on smooth floors when accelerating; the acceleration of the profiles is the practical "
    "limit, and the floor calibration must be repeated on every floor.",
    "No position or heading tracking controller is used for the path (as required at this stage), so the errors "
    "of the wheels accumulate along the trajectory.",
])

P("References", H1)
P("[1] L. H. Ríos G. and M. Bueno L., \"Modelo Matemático para un Robot Móvil,\" <i>Scientia et Technica</i>, "
  "vol. XIV, no. 38, pp. 13–18, Universidad Tecnológica de Pereira, 2008.")
P("[2] Source code and run data: github.com/Ferchito1974/Robot_Autonomo (folder tarea_velocidad_u3).")


def on_page(c, doc):
    if doc.page == 1:
        return
    c.saveState()
    c.setFont("TR", 10)
    c.drawCentredString(letter[0] / 2, 1.3 * cm, str(doc.page))
    c.restoreState()


doc = SimpleDocTemplate(OUT, pagesize=letter, leftMargin=2.7 * cm, rightMargin=2.7 * cm, topMargin=2.5 * cm,
                        bottomMargin=2.3 * cm, title=TITLE)
doc.build(story, onFirstPage=on_page, onLaterPages=on_page)
print("PDF:", OUT, "| figures:", nfig[0], "| tables:", ntab[0])
