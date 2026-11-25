"""
config.py - Constants of AMR Assessment 1 Unit 3: wheel velocity control (P / PID)
and trajectory generation (line, circle, square) with the 3-wheel FutBot Omni.

The Arduino (firmware FutBot_fw_v5, already flashed) only reads the sensors and
applies raw PWM ({PWM,p1,p2,p3}); the wheel controllers run here in Python.
"""
import math

# ── Serial link ──────────────────────────────────────────────────────────
PORT = "/dev/ttyACM0"
BAUD = 115200
FIRMWARE = "FutBot_fw_v5"

# ── Geometry (firmware robot_config.h) ───────────────────────────────────
WHEEL_DIAMETER_M = 0.038          # firmware value (PENDING physical check, 38 vs 48 mm)
WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0
COUNTS_PER_REV = 120              # 12 PPR x gearbox 10, rising edge of channel A
L_M = 0.095                       # center to wheel
WHEEL_ANGLES_DEG = (270.0, 30.0, 150.0)   # wheel 1 rear, 2 front-right, 3 front-left
WHEEL_ANGLES_RAD = tuple(math.radians(a) for a in WHEEL_ANGLES_DEG)
# Robot frame (as firmware and odometry assignment): +vy forward, +vx right, +w CCW.
#   v_i = -sin(a_i) vx + cos(a_i) vy + L w      (tangential wheel speed, firmware sign)

# ── Motors (identified 23-nov-2026) ──────────────────────────────────────
MOTOR_K_RPM_PER_PWM = 2.35        # wheels lifted
MOTOR_TAU_S = 0.030
PWM_MAX = 255
# Feedforward: PWM = offset + RPM / K. Floor dead zone measured 23-nov for wheels 3 (12.0)
# and 2 (21.5); wheel 1 was braked then, so it uses the mean (ASSUMED, not measured).
FF_OFFSET_PWM = (16.7, 21.5, 12.0)
FF_K_RPM_PER_PWM = (2.35, 2.49, 2.20)

# ── Control ──────────────────────────────────────────────────────────────
DT = 0.02                         # 50 Hz: one control step per telemetry line
GAINS = {                         # PWM per RPM (kp), PWM per RPM*s (ki), PWM per RPM/s (kd)
    # real robot 25-nov (lifted): kp 1.0 oscillated (applied PWM lags ~40 ms + Kalman lag);
    # retuned on the virtual robot with that delay
    "P":   {"kp": 0.4, "ki": 0.0, "kd": 0.0, "ff": True},
    "PID": {"kp": 0.3, "ki": 4.0, "kd": 0.0, "ff": True},     # sim sweep: kd brings no gain, ki 8 + kp 0.5 unstable
}
I_MAX_PWM = 120.0                 # anti-windup clamp of the integral term (room for rough floors)
START_PWM = 35.0                  # start-up kick: wheel stopped with a reference -> at least this PWM
START_RPM = 3.0                   # "stopped" threshold for the kick

# ── Trajectories (defaults, editable in the interface) ──────────────────
TRAJ = {
    "line":   {"v": 0.50, "distance": 0.8},       # fits a 1.2 x 1.2 m space
    "circle": {"v": 0.50, "radius": 0.35, "turns": 1.0},
    "square": {"v": 0.50, "side": 0.60, "w_turn": 4.0},
    "spin":   {"w": 3.0, "turns": 2.0},           # rotation in place (odometry calibration of k_rot)
}
# Omni mode: the robot keeps its initial heading (0 deg, gyroscope) and translates in the world frame
TRAJ_OMNI = {
    "line":   {"v": 0.50, "distance": 0.8},
    "circle": {"v": 0.50, "radius": 0.35, "turns": 1.0},
    "square": {"v": 0.50, "side": 0.60},
}
# Speed profiles (omni). Profile 1 is the base one (floors with more slip); the others are faster.
PROFILES = [
    # acc = PEAK acceleration of the S-curve ramps. Real 25-nov: heading kicks of 5-17 deg with
    # 1.5-2.0 m/s^2 and 28-44 deg with 2.2-2.5 m/s^2 (wheel slip when accelerating).
    {"name": "1 Base (piso que derrapa)", "v": 0.50, "acc": 1.2},
    {"name": "2 Medio",                   "v": 0.65, "acc": 1.5},
    {"name": "3 Rápido",                  "v": 0.75, "acc": 1.8},
    {"name": "4 Muy rápido",              "v": 0.85, "acc": 2.0},     # 0.95 saturated 30 % on the real robot
]
K_HEAD = 4.0                      # heading hold (omni mode): w = K_HEAD e + KI_HEAD int(e), e = 0 - theta
KI_HEAD = 2.0                     # real 25-nov: P 2.0 let the heading swing +-25 deg at 0.8 m/s;
                                  # sim with the 40 ms delay: unstable from K_HEAD ~7 (margin x1.6)
W_HOLD_MAX = 2.0                  # rad/s, limit of the heading hold (3.0 made the wheels slip more)

# Traction control (omni): slip = |w_wheels - w_gyro|. Real 25-nov: runs without problems stay below
# ~1 rad/s; runs where the heading kicked 20-60 deg reached 1.8-5 rad/s.
TRACTION = 0.0                    # 1 on, 0 off (real 25-nov 16:01: no measurable gain at 1.8 m/s^2; optional)
SLIP_ON = 1.0                     # rad/s, slip above this slows the reference down
TC_GAIN = 0.6                     # scale reduction per rad/s of slip above SLIP_ON
TC_SCALE_MIN = 0.4                # the reference never goes slower than 40 %
TC_DOWN, TC_UP = 0.7, 0.5         # 1/s; v*dscale/dt must stay small or the change itself makes the wheels slip (sim: 5.0 made it worse)

ACC_MAX = 1.5                     # m/s^2, ramp of the linear velocity reference
ALPHA_MAX = 12.0                  # rad/s^2, ramp of the angular velocity reference
REST_BEFORE_S = 1.5               # robot still: gyro bias estimation
REST_AFTER_S = 1.0                # keep logging after the reference ends

# ── Floor calibration (rotation in place: the three wheels get the same speed) ─
CAL_RPM = (80.0, 160.0)           # two speed levels, each held CCW and CW
CAL_HOLD_S = 1.6                  # hold time per level; the PWM is averaged over the last half

# ── Settings editable in the interface (settings.json overrides these defaults) ─
# Kinematic correction factors (as in the odometry assignment):
#   real rotation = K_ROT x rotation predicted by the wheel speeds (wheel slip on the floor)
#   real distance = K_LIN x distance predicted by the wheel speeds (wheel diameter not verified)
# 25-nov floor runs: gyro / encoder rotation = 1.07 to 1.13 (median ~1.09); odometry task: 1.082.
K_ROT = 1.08
K_LIN = 1.0                       # PENDING: measure a straight run with a tape

FF_LOOKAHEAD_S = 0.10             # feedforward uses the reference this far ahead (delay compensation;
                                  # sim: wheel error 22 -> 7 RPM RMS on the omni square)

S_CURVE = 1.0                     # 1: S-shaped ramps (finite jerk), 0: linear ramps

EDITABLE = {"traction": TRACTION, "slip_on": SLIP_ON, "s_curve": S_CURVE, "heading_source": "gyro", "k_head": K_HEAD, "ki_head": KI_HEAD, "ff_lookahead": FF_LOOKAHEAD_S, "k_rot": K_ROT, "k_lin": K_LIN, "acc_max": ACC_MAX, "alpha_max": ALPHA_MAX, "start_pwm": START_PWM, "i_max": I_MAX_PWM,
            "rest_before_s": REST_BEFORE_S, "ff_offset": list(FF_OFFSET_PWM), "ff_k": list(FF_K_RPM_PER_PWM)}

# ── Safety ───────────────────────────────────────────────────────────────
TELEMETRY_TIMEOUT_S = 0.30
REVERSE_PWM, REVERSE_RPM, REVERSE_S = 100, 100, 0.15   # wheel turning against its PWM -> abort (encoder fault)
SAT_ABORT_S = 1.0                 # one wheel saturated this long -> abort (runaway)
MAX_COUNTS_PER_SAMPLE = 60
MAX_RUN_TIME_S = 60.0
MIN_V, MAX_V = 0.05, 1.00          # 1.0 m/s = 435 RPM in wheels 2/3 (max ~550 on the floor)
IMU_MAX_AGE_MS = 150
GYRO_Z_SIGN = -1.0                # IMU mounted upside down (verified 18-nov-2026)

# ── Kalman filter per wheel (same tuning as the odometry assignment) ────
KF_ACCEL_STD = 2.0
KF_MEAS_STD = (2 * math.pi / COUNTS_PER_REV) / math.sqrt(12)

# ── Web interface ────────────────────────────────────────────────────────
WEB_HOST = "0.0.0.0"
WEB_PORT = 5002                   # 5000 odometry, 5001 PID project
