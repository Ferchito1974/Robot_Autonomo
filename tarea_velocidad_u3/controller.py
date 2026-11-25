"""
controller.py - Wheel speed controller (runs in Python at 50 Hz, one per wheel).

    e   = rpm_ref - rpm_measured                 (measured = Kalman filter of the encoder)
    u   = FF(rpm_ref) + Kp e + I - Kd d(rpm)/dt   [PWM], saturated at +/-255
    I  += Ki e dt  (only while not saturated: anti-windup), |I| <= I_MAX
    FF  = sign(ref) (offset + |ref| / K)          optional feedforward (static motor model)

P controller: Ki = Kd = 0.  PID (additional challenge): Ki > 0 (Kd optional).
Start-up kick: a stopped wheel with a reference gets at least START_PWM (static friction).
"""
import config


class WheelController:
    def __init__(self, kp, ki=0.0, kd=0.0, ff=True, offset=0.0, k_rpm=config.MOTOR_K_RPM_PER_PWM,
                 i_max=config.I_MAX_PWM, pwm_max=config.PWM_MAX, start_pwm=config.START_PWM):
        self.kp, self.ki, self.kd, self.ff_on, self.start_pwm = kp, ki, kd, ff, start_pwm
        self.offset, self.k_rpm, self.i_max, self.pwm_max = offset, k_rpm, i_max, pwm_max
        self.integral, self.prev_meas = 0.0, None
        self.terms = (0.0, 0.0, 0.0, 0.0)

    def ff(self, ref):
        if not self.ff_on or abs(ref) < 1.0:
            return 0.0
        f = self.offset + abs(ref) / self.k_rpm
        return f if ref > 0 else -f

    def step(self, ref, meas, dt, ref_ff=None):
        """ref_ff: reference used by the feedforward (the reference a few tens of ms ahead, to
        compensate the actuation delay and the motor lag); defaults to ref."""
        e = ref - meas
        d = 0.0 if self.prev_meas is None or dt <= 0 else -self.kd * (meas - self.prev_meas) / dt
        self.prev_meas = meas
        f = self.ff(ref if ref_ff is None else ref_ff)
        i_new = max(-self.i_max, min(self.i_max, self.integral + self.ki * e * dt))
        u = f + self.kp * e + i_new + d
        if abs(u) <= self.pwm_max or (u > 0) != (e > 0):      # anti-windup: freeze I when saturated
            self.integral = i_new
        u = f + self.kp * e + self.integral + d
        # start-up kick (static friction, e.g. rough floor): stopped wheel with a reference
        if abs(ref) > 5 and abs(meas) < config.START_RPM and 0 < abs(u) < self.start_pwm and (u > 0) == (ref > 0):
            u = self.start_pwm if ref > 0 else -self.start_pwm
        self.terms = (f, self.kp * e, self.integral, d)
        return max(-self.pwm_max, min(self.pwm_max, u))

    def reset(self):
        self.integral, self.prev_meas = 0.0, None


def make(kind, kp, ki, kd, ff, st=None):
    """Three wheel controllers with the per-wheel feedforward calibration (st: settings dict)."""
    st = st or config.EDITABLE
    if kind == "P":
        ki = kd = 0.0
    return [WheelController(kp, ki, kd, ff, st["ff_offset"][i], st["ff_k"][i], i_max=st["i_max"],
                            start_pwm=st["start_pwm"]) for i in range(3)]
