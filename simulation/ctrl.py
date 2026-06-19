import numpy as np
from dataclasses import dataclass


@dataclass
class Gains:
    kp_roll:  float = 3.5;  ki_roll:  float = 0.04; kd_roll:  float = 0.15
    kp_pitch: float = 3.5;  ki_pitch: float = 0.04; kd_pitch: float = 0.15
    kp_yaw:   float = 2.0;  ki_yaw:   float = 0.02; kd_yaw:   float = 0.08
    kp_x:     float = 0.25; ki_x:     float = 0.005; kd_x:    float = 0.15
    kp_y:     float = 0.25; ki_y:     float = 0.005; kd_y:    float = 0.15
    kp_z:     float = 0.40; ki_z:     float = 0.02;  kd_z:    float = 0.20


GAINS_KEYS = [
    "kp_roll","ki_roll","kd_roll",
    "kp_pitch","ki_pitch","kd_pitch",
    "kp_yaw","ki_yaw","kd_yaw",
    "kp_x","ki_x","kd_x",
    "kp_y","ki_y","kd_y",
    "kp_z","ki_z","kd_z",
]


class PID:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.reset()

    def set_gains(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd

    def reset(self):
        self._integral = 0.0
        self._prev_meas = 0.0
        self._first = True

    def compute(self, error, measurement, dt):
        if dt <= 0:
            return np.clip(self.kp * error + self._integral, self.out_min, self.out_max)
        p = self.kp * error
        self._integral = np.clip(self._integral + self.ki * error * dt, self.out_min, self.out_max)
        d = 0.0
        if not self._first:
            d = -self.kd * (measurement - self._prev_meas) / dt
        self._prev_meas = measurement
        self._first = False
        return np.clip(p + self._integral + d, self.out_min, self.out_max)


MAX_TILT = 0.35
HOVER_THROTTLE = 0.50
POSITION_HZ = 25.0


class PositionPID:
    def __init__(self):
        self._pid_x = PID(0, 0, 0, -MAX_TILT, MAX_TILT)
        self._pid_y = PID(0, 0, 0, -MAX_TILT, MAX_TILT)
        self._pid_z = PID(0, 0, 0, -0.4, 0.4)
        self._desired_yaw = 0.0

    def setup(self, kp_x, ki_x, kd_x, kp_y, ki_y, kd_y, kp_z, ki_z, kd_z):
        self._pid_x.set_gains(kp_x, ki_x, kd_x)
        self._pid_y.set_gains(kp_y, ki_y, kd_y)
        self._pid_z.set_gains(kp_z, ki_z, kd_z)

    def update(self, tx, ty, tz, px, py, pz, dt):
        cv_x = self._pid_x.compute(tx - px, px, dt)
        cv_y = self._pid_y.compute(ty - py, py, dt)
        cv_z = self._pid_z.compute(tz - pz, pz, dt)
        return cv_x, cv_y, cv_z, self._desired_yaw


class AttitudeController:
    def __init__(self):
        self._pid_roll = PID(0, 0, 0, -0.5, 0.5)
        self._pid_pitch = PID(0, 0, 0, -0.5, 0.5)

    def setup(self, kp_r, ki_r, kd_r, kp_p, ki_p, kd_p):
        self._pid_roll.set_gains(kp_r, ki_r, kd_r)
        self._pid_pitch.set_gains(kp_p, ki_p, kd_p)

    def update(self, imu_roll, imu_pitch, cv_x, cv_y, dt):
        desired_pitch = cv_x
        desired_roll = -cv_y
        roll_cmd = self._pid_roll.compute(desired_roll - imu_roll, imu_roll, dt)
        pitch_cmd = self._pid_pitch.compute(desired_pitch - imu_pitch, imu_pitch, dt)
        return roll_cmd, pitch_cmd


class YawController:
    def __init__(self):
        self._pid_yaw = PID(0, 0, 0, -0.5, 0.5)

    def setup(self, kp, ki, kd):
        self._pid_yaw.set_gains(kp, ki, kd)

    def update(self, imu_yaw, desired_yaw, dt):
        err = desired_yaw - imu_yaw
        err = (err + np.pi) % (2 * np.pi) - np.pi
        return self._pid_yaw.compute(err, imu_yaw, dt)


class Controller:
    def __init__(self, gains=None):
        self._pos_pid = PositionPID()
        self._att_ctrl = AttitudeController()
        self._yaw_ctrl = YawController()
        self._cv_x = self._cv_y = self._cv_yaw = 0.0
        self._base_throttle = HOVER_THROTTLE
        self._last_pos_t = 0.0
        if gains:
            self.apply_gains(gains)

    def apply_gains(self, g):
        self._pos_pid.setup(g.kp_x, g.ki_x, g.kd_x,
                            g.kp_y, g.ki_y, g.kd_y,
                            g.kp_z, g.ki_z, g.kd_z)
        self._att_ctrl.setup(g.kp_roll, g.ki_roll, g.kd_roll,
                             g.kp_pitch, g.ki_pitch, g.kd_pitch)
        self._yaw_ctrl.setup(g.kp_yaw, g.ki_yaw, g.kd_yaw)

    def update(self, t, dt, roll_sim, pitch_sim, yaw_sim,
               pos_x, pos_y, pos_z, tx, ty, tz):
        if abs(roll_sim) > 0.785 or abs(pitch_sim) > 0.785:
            return np.zeros(4)

        imu_roll = roll_sim
        imu_pitch = -pitch_sim
        imu_yaw = -yaw_sim

        dt_pos = t - self._last_pos_t
        if dt_pos >= 1.0 / POSITION_HZ:
            self._last_pos_t = t
            cv_x, cv_y, cv_z, cv_yaw = self._pos_pid.update(
                tx, ty, tz, pos_x, pos_y, pos_z, dt_pos)
            self._cv_x = cv_x
            self._cv_y = cv_y
            self._cv_yaw = cv_yaw
            self._base_throttle = np.clip(HOVER_THROTTLE + cv_z, 0.0, 1.0)

        roll_cmd, pitch_cmd = self._att_ctrl.update(
            imu_roll, imu_pitch, self._cv_x, self._cv_y, dt)
        yaw_cmd = self._yaw_ctrl.update(imu_yaw, self._cv_yaw, dt)

        return np.clip([
            self._base_throttle + roll_cmd - pitch_cmd + yaw_cmd,
            self._base_throttle - roll_cmd - pitch_cmd - yaw_cmd,
            self._base_throttle + roll_cmd + pitch_cmd - yaw_cmd,
            self._base_throttle - roll_cmd + pitch_cmd + yaw_cmd,
        ], 0, 1)


def load_gains(path):
    g = Gains()
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, _, val = line.partition("=")
                key = key.strip().lower()
                if key in GAINS_KEYS:
                    setattr(g, key, float(val.strip()))
    except FileNotFoundError:
        pass
    return g


def save_gains(g, path):
    with open(path, "w") as f:
        f.write("# Drone_Main gains\n")
        for key in GAINS_KEYS:
            f.write(f"{key.upper()}={getattr(g, key):.4f}\n")
