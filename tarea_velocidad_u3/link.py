"""
link.py - Serial link with the Arduino (firmware FutBot_fw_v5). Copied from Proyecto_Robot_Control.
Copied from tarea_odometria/robot_link.py and extended for v5: wheel RPM
command, runtime PID gains / limits, PID debug lines "D,...".

The Arduino only acquires sensors and drives motors. This module:
  * opens the port (the Mega resets on connect) and waits for "Ready",
  * reads lines in a background thread,
  * parses telemetry lines "T,..." into Sample objects,
  * sends commands (velocity, raw PWM, mode, calibration, stop).
"""
import collections
import threading
import time
from dataclasses import dataclass

import serial

import config


@dataclass
class Sample:
    """One telemetry line, converted to physical units."""
    t_ms: int            # Arduino millis()
    enc: tuple           # cumulative encoder counts (M1, M2, M3)
    pwm: tuple           # signed PWM applied (wheel convention)
    gyro: tuple          # deg/s, sensor frame
    acc: tuple           # m/s^2, sensor frame (with gravity)
    yaw_deg: float       # fused yaw (BNO085 rotation vector)
    gyro_age_ms: int
    acc_age_ms: int
    yaw_age_ms: int
    mode: int            # 0 closed loop, 1 open loop, 2 raw PWM
    rx_time: float       # Pi time.time() when received


def parse_telemetry(line):
    """Return a Sample or None if the line is not a valid telemetry line."""
    p = line.strip().split(",")
    if len(p) != 19 or p[0] != "T":
        return None
    try:
        v = [int(x) for x in p[1:]]
    except ValueError:
        return None
    return Sample(
        t_ms=v[0], enc=(v[1], v[2], v[3]), pwm=(v[4], v[5], v[6]),
        gyro=(v[7] / 100.0, v[8] / 100.0, v[9] / 100.0),
        acc=(v[10] / 100.0, v[11] / 100.0, v[12] / 100.0),
        yaw_deg=v[13] / 100.0, gyro_age_ms=v[14], acc_age_ms=v[15],
        yaw_age_ms=v[16], mode=v[17], rx_time=time.time())


def parse_debug(line):
    """D,ms,sp1,sp2,sp3,rpm1,rpm2,rpm3,ff2,p2,i2,d2,ff3,p3,i3,d3 (x10 integers)."""
    p = line.strip().split(",")
    if len(p) != 16:
        return None
    try:
        v = [int(x) / 10.0 for x in p[2:]]
    except ValueError:
        return None
    return {"t_ms": int(p[1]), "sp": v[0:3], "rpm": v[3:6],
            "terms2": dict(zip(("ff", "p", "i", "d"), v[6:10])),
            "terms3": dict(zip(("ff", "p", "i", "d"), v[10:14]))}


class RobotLink:
    def __init__(self, port=config.PORT, baud=config.BAUD):
        self.port, self.baud = port, baud
        self.ser = None
        self._tx_lock = threading.Lock()
        self._running = False
        self.log = collections.deque(maxlen=200)      # non-telemetry lines
        self.last_sample = None
        self.last_rx = 0.0
        self.bad_lines = 0                            # consecutive bad "T," lines
        self._subscribers = []                        # callables(sample)
        self._sub_lock = threading.Lock()
        self.firmware = "?"
        self.last_debug = None                        # last "D,..." line (dict)
        self.config_line = ""                          # last "[CFG] ..." line

    # ── connection ────────────────────────────────────────────────────
    def connect(self, wait_ready_s=6.0):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        t0 = time.time()
        ready = False
        while time.time() - t0 < wait_ready_s:             # Mega resets on open
            line = self.ser.readline().decode("utf-8", "replace").strip()
            if line:
                self.log.append(line)
                if "Firmware:" in line:
                    self.firmware = line.split("Firmware:")[1].strip()
                if line.startswith("Ready") and self.firmware != "?":
                    ready = True                  # only after the firmware banner was read
                    break
        if not ready:
            self.ser.close()
            raise RuntimeError("Arduino did not send 'Ready' (check firmware/port)")
        if self.firmware != config.FIRMWARE:
            # SAFETY: other firmware may parse "{...}" char by char ('1' = rotate 45 deg)
            # or not know the v5 commands. Send nothing at all.
            self.ser.close()
            self.ser = None
            raise RuntimeError(f"firmware '{self.firmware}' no es {config.FIRMWARE}: no se envia nada")
        self._running = True
        threading.Thread(target=self._reader, daemon=True).start()

    def close(self):
        try:
            self.stop()
            self.send("{TEL,0}")
        finally:
            self._running = False
            time.sleep(0.1)
            if self.ser:
                self.ser.close()

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open and self._running

    # ── reading ───────────────────────────────────────────────────────
    def subscribe(self, fn):
        with self._sub_lock:
            self._subscribers.append(fn)

    def unsubscribe(self, fn):
        with self._sub_lock:
            if fn in self._subscribers:
                self._subscribers.remove(fn)

    def _reader(self):
        while self._running:
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError) as e:
                self.log.append(f"SERIAL ERROR: {e}")
                self._running = False
                break
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").strip()
            if line.startswith("T,"):
                s = parse_telemetry(line)
                if s is None:
                    self.bad_lines += 1
                    continue
                self.bad_lines = 0
                self.last_sample, self.last_rx = s, s.rx_time
                with self._sub_lock:
                    subs = list(self._subscribers)
                for fn in subs:
                    fn(s)
            elif line.startswith("D,"):
                d = parse_debug(line)
                if d:
                    self.last_debug = d
            elif line:
                if line.startswith("[CFG]"):
                    self.config_line = line
                self.log.append(line)

    # ── writing ───────────────────────────────────────────────────────
    def send(self, text):
        if not self.ser:
            return
        with self._tx_lock:
            self.ser.write((text + "\n").encode())

    def velocity(self, vx_mms, vy_mms, omega_rads):
        """Robot-frame velocity: +vy forward, +vx right, +omega CCW."""
        self.send(f"<{vx_mms:.1f},{vy_mms:.1f},{omega_rads:.3f}>")

    def raw_pwm(self, p1, p2, p3):
        self.send(f"{{PWM,{int(p1)},{int(p2)},{int(p3)}}}")

    def open_loop(self, on=True):
        self.send("{OL,1}" if on else "{OL,0}")

    def set_calibration(self, cal):
        """cal: list of 3 dicts with 'offset' and 'gain' (PWM per RPM)."""
        args = ",".join(f"{c['offset']:.2f},{c['gain']:.4f}" for c in cal)
        self.send(f"{{CAL,{args}}}")

    def telemetry(self, on=True, period_ms=20):
        self.send(f"{{TEL,1,{period_ms}}}" if on else "{TEL,0}")

    # ── v5 commands ──────────────────────────────────────────────
    def wheel_rpm(self, r1, r2, r3):
        self.send(f"{{W,{r1:.1f},{r2:.1f},{r3:.1f}}}")

    def gains(self, kp, ki, kd, imax):
        self.send(f"{{GAIN,{kp:.5f},{ki:.5f},{kd:.5f},{imax:.1f}}}")

    def limits(self, pwm_max, deadband):
        self.send(f"{{LIM,{int(pwm_max)},{int(deadband)}}}")

    def feedforward(self, on):
        self.send("{FF,1}" if on else "{FF,0}")

    def debug(self, on=True, period_ms=100):
        self.send(f"{{DBG,1,{period_ms}}}" if on else "{DBG,0}")

    def request_config(self):
        self.send("{CFG}")

    def stop(self):
        """Stop motors. Sent twice: it is the safety command."""
        self.send("s")
        self.send("s")
