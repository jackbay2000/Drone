"""
Faithful Python port of Drone_Main/IMU.cpp and Drone_Main/Position.cpp --
NOT a statistical approximation of their effects. Runs the same complementary
filter, axis remap, median-of-3 spike filter, in-flight yaw-bias learning,
tilt correction, and optical-flow integration (including the yaw-frame
rotation fix applied to Position.cpp) as the real firmware, driven by
synthetic raw sensor readings derived from the true physics state.

Every constant below is copied from the C++ source it mirrors. If
Drone_Main's IMU.cpp/Position.cpp change, this file needs the same edit --
there is no build-time link between them, so drift is silent.

Raw-sensor synthesis inverts the firmware's own raw->physical conversions
(scale factors, axis remap, flow/tilt geometry) so that, once fed through the
ported algorithms below, the *true* physical quantities are recovered up to
injected noise/dropout -- exactly what would happen on real hardware if the
sensors were noiseless. Noise/dropout magnitudes are engineering estimates,
not measured device characteristics.
"""

import math
import numpy as np

G = 9.81

# MPU6050 default full-scale factors used by IMU.cpp
ACCEL_LSB_PER_G  = 16384.0   # IMU.cpp: ax/16384.0
GYRO_LSB_PER_DPS = 131.0     # IMU.cpp: gx/131.0

# PMW3901 lens focal length (effective pixels) -- Position.cpp FOCAL_LEN
# Bench-calibrated 2026-07-19, see Drone_Main/Position.cpp for derivation.
FOCAL_LEN = 618.0


# ============================================================================
# Raw sensor synthesis
# ============================================================================

def synth_raw_imu(f_body, omega_body_rad, rng,
                  accel_noise_std_g=0.02, gyro_noise_std_dps=0.05):
    """
    f_body:         true specific force in the flow-sensor-aligned body frame
                    (+x fwd, +y left, +z up) [m/s^2] -- PhysicsEngine.true_sensor_frame().
    omega_body_rad: true body rates in the same frame [rad/s].

    Returns raw (ax, ay, az, gx, gy, gz) as a real MPU6050 -- mounted 90 deg
    rotated from the flow sensor, per IMU.cpp's own axis-remap comment --
    would report, so IMU.cpp's unmodified remap logic recovers the
    flow-frame values.
    """
    afx, afy, afz = f_body / G
    gfx, gfy, gfz = np.degrees(omega_body_rad)

    # Invert IMU.cpp's remap:
    #   a_x(flow) = FLOW_X_SIGN * (ay_raw/16384)        [FLOW_X_SIGN = +1]
    #   a_y(flow) = FLOW_Y_SIGN * (ax_raw/16384)         [FLOW_Y_SIGN = -1]
    #   a_z(flow) =               az_raw/16384
    ay_g, ax_g, az_g = afx, -afy, afz

    #   _gRate[0](flow x) = FLOW_X_SIGN * (gy_raw/131 - bias)
    #   _gRate[1](flow y) = FLOW_Y_SIGN * (gx_raw/131 - bias)
    #   _gRate[2](flow z) =               gz_raw/131 - bias
    gy_dps, gx_dps, gz_dps = gfx, -gfy, gfz

    rn = rng.normal
    ax = (ax_g + rn(0.0, accel_noise_std_g)) * ACCEL_LSB_PER_G
    ay = (ay_g + rn(0.0, accel_noise_std_g)) * ACCEL_LSB_PER_G
    az = (az_g + rn(0.0, accel_noise_std_g)) * ACCEL_LSB_PER_G
    gx = (gx_dps + rn(0.0, gyro_noise_std_dps)) * GYRO_LSB_PER_DPS
    gy = (gy_dps + rn(0.0, gyro_noise_std_dps)) * GYRO_LSB_PER_DPS
    gz = (gz_dps + rn(0.0, gyro_noise_std_dps)) * GYRO_LSB_PER_DPS
    return ax, ay, az, gx, gy, gz


class FlowCountSynth:
    """
    Stateful raw-count synthesizer for the PMW3901 -- inverts Position.cpp's
    forward conversion (disp = dx_f*z/FOCAL_LEN, dx_f = dx - gz*FOCAL_LEN*dt)
    so the ported estimator recovers true translation.

    Real motion-count registers accumulate sub-pixel motion continuously
    between reads and report only the integer part, retaining any fractional
    remainder for the next read. At 250 Hz and typical slow-flight speeds,
    true displacement per tick is well under 1 count -- an independent
    round-per-tick (rather than a carried remainder) would silently discard
    essentially all real motion instead of it showing up a tick or two later,
    which is not how the actual sensor register behaves.
    """
    def __init__(self):
        self._dx_rem = 0.0
        self._dy_rem = 0.0

    def reset(self):
        self._dx_rem = 0.0
        self._dy_rem = 0.0

    def sample(self, v_body_xy, gz_true_rad_s, gx_true_rad_s, z_true, dt, rng,
              count_noise_std=1.0, dropout_prob=0.0):
        if z_true <= 0.0 or rng.random() < dropout_prob:
            return 0, 0   # lost-lock frame -- FLOW_GATE rejects this downstream, same as real hardware

        disp_x_true = v_body_xy[0] * dt
        disp_y_true = v_body_xy[1] * dt
        dx_f = disp_x_true * FOCAL_LEN / z_true + gz_true_rad_s * FOCAL_LEN * dt
        dy_f = disp_y_true * FOCAL_LEN / z_true + gx_true_rad_s * FOCAL_LEN * dt

        self._dx_rem += dx_f
        self._dy_rem += dy_f
        dx_int = math.trunc(self._dx_rem)
        dy_int = math.trunc(self._dy_rem)
        self._dx_rem -= dx_int
        self._dy_rem -= dy_int

        dx_int += int(round(rng.normal(0.0, count_noise_std)))
        dy_int += int(round(rng.normal(0.0, count_noise_std)))

        # Emit in "raw sensor" convention (swapped + inverted relative to
        # flow-frame forward/left) -- see the matching correction in
        # PositionEstimator.update(), which mirrors the real fix bench-
        # verified 2026-07-18 in Position.cpp. These two swaps are exact
        # inverses of each other, so this has zero net effect on simulated
        # behavior; it exists purely so both halves keep mirroring the
        # actual (now-corrected) firmware convention.
        return -dy_int, -dx_int


def synth_raw_range_mm(z_true, roll_true, pitch_true, rng,
                       noise_std_m=0.01, dropout_prob=0.0):
    """Inverts Position.cpp's tilt correction (h = d*cos(roll)*cos(pitch)),
    using the TRUE tilt -- the raw sensor measures the true slant distance
    regardless of what the estimator later believes the tilt to be."""
    if rng.random() < dropout_prob:
        return 0   # rejected reading -- Rangefinder.cpp's `0 < d < 4000` check rejects this
    denom = math.cos(roll_true) * math.cos(pitch_true)
    if denom <= 1e-3:
        return 0
    d_m = z_true / denom + rng.normal(0.0, noise_std_m)
    return int(round(d_m * 1000.0))


# ============================================================================
# ComplementaryFilter -- exact port of IMU.cpp
# ============================================================================

class ComplementaryFilter:
    FILTER_TAU     = 0.75
    RAD_PER_DEG    = 0.01745329251
    FLOW_X_SIGN    = 1.0
    FLOW_Y_SIGN    = -1.0
    YAW_BIAS_LEARN = 0.0001

    def __init__(self):
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._roll_offset = 0.0
        self._pitch_offset = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._g_rate = [0.0, 0.0, 0.0]
        self._accel_roll_buf = [0.0, 0.0, 0.0]
        self._accel_pitch_buf = [0.0, 0.0, 0.0]
        self._accel_buf_idx = 0

    @staticmethod
    def _median3(a, b, c):
        if a > b: a, b = b, a
        if b > c: b, c = c, b
        if a > b: a, b = b, a
        return b

    def calibrate(self, raw_samples):
        """raw_samples: iterable of (ax,ay,az,gx,gy,gz) raw readings taken
        while stationary and level -- mirrors IMU::setup()'s N=1000 average."""
        sa = [0.0, 0.0, 0.0]
        sg = [0.0, 0.0, 0.0]
        n = 0
        for ax, ay, az, gx, gy, gz in raw_samples:
            sa[0] += ax / ACCEL_LSB_PER_G; sa[1] += ay / ACCEL_LSB_PER_G; sa[2] += az / ACCEL_LSB_PER_G
            sg[0] += gx / GYRO_LSB_PER_DPS; sg[1] += gy / GYRO_LSB_PER_DPS; sg[2] += gz / GYRO_LSB_PER_DPS
            n += 1
        sa = [v / n for v in sa]
        sg = [v / n for v in sg]

        self._gyro_bias[0] = sg[1]   # flow GX bias = IMU GY bias
        self._gyro_bias[1] = sg[0]   # flow GY bias = IMU GX bias
        self._gyro_bias[2] = sg[2]

        ax_avg = self.FLOW_X_SIGN * sa[1]
        ay_avg = self.FLOW_Y_SIGN * sa[0]
        az_avg = sa[2]
        self._roll_offset  = math.atan2(ay_avg, az_avg)
        self._pitch_offset = math.atan2(-ax_avg, math.sqrt(ay_avg**2 + az_avg**2))
        self._roll = self._pitch = self._yaw = 0.0

    def update(self, ax, ay, az, gx, gy, gz, dt):
        if dt <= 0.0 or dt > 0.1:
            return

        ax_g, ay_g, az_g = ax / ACCEL_LSB_PER_G, ay / ACCEL_LSB_PER_G, az / ACCEL_LSB_PER_G
        gx_dps, gy_dps, gz_dps = gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS

        a_x = self.FLOW_X_SIGN * ay_g
        a_y = self.FLOW_Y_SIGN * ax_g
        a_z = az_g

        self._g_rate[0] = self.FLOW_X_SIGN * (gy_dps - self._gyro_bias[0]) * self.RAD_PER_DEG
        self._g_rate[1] = self.FLOW_Y_SIGN * (gx_dps - self._gyro_bias[1]) * self.RAD_PER_DEG
        self._g_rate[2] =                    (gz_dps - self._gyro_bias[2]) * self.RAD_PER_DEG

        alpha = self.FILTER_TAU / (self.FILTER_TAU + dt)

        idx = self._accel_buf_idx
        self._accel_roll_buf[idx]  = math.atan2(a_y, a_z)
        self._accel_pitch_buf[idx] = math.atan2(-a_x, math.sqrt(a_y*a_y + a_z*a_z))
        self._accel_buf_idx = (idx + 1) % 3

        accel_roll  = self._median3(*self._accel_roll_buf)  - self._roll_offset
        accel_pitch = self._median3(*self._accel_pitch_buf) - self._pitch_offset

        self._roll  = alpha * (self._roll  + self._g_rate[0] * dt) + (1.0 - alpha) * accel_roll
        self._pitch = alpha * (self._pitch + self._g_rate[1] * dt) + (1.0 - alpha) * accel_pitch

        if abs(self._g_rate[2]) < 0.005:
            self._gyro_bias[2] += (gz_dps - self._gyro_bias[2]) * self.YAW_BIAS_LEARN

        self._yaw += self._g_rate[2] * dt

    def get_roll(self):  return self._roll
    def get_pitch(self): return self._pitch
    def get_yaw(self):   return self._yaw
    def get_gx(self):    return self._g_rate[0]
    def get_gz(self):    return self._g_rate[2]


# ============================================================================
# PositionEstimator -- exact port of Position.cpp, including the yaw-frame
# rotation fix (see project memory: without it, Position.cpp accumulated
# body-frame flow displacement as if it were world frame).
# ============================================================================

class PositionEstimator:
    ALT_ALPHA = 0.85
    ALT_MIN   = 0.05
    ALT_MAX   = 3.50
    FLOW_GATE = 1
    # Mirrors Drone_Main/Position.cpp's ALT_STALE_TIMEOUT -- see that file
    # for why this exists (diagnosed 2026-07-19: no recovery path once the
    # rangefinder stops producing valid readings for a sustained stretch).
    ALT_STALE_TIMEOUT = 0.5

    def __init__(self):
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._time_since_valid_range = 0.0

    def reset(self):
        self._x = 0.0
        self._y = 0.0
        self._time_since_valid_range = 0.0
        # Z intentionally not reset -- mirrors Position::reset()

    def is_stale(self) -> bool:
        return self._time_since_valid_range > self.ALT_STALE_TIMEOUT

    def update(self, dx, dy, range_ready, range_mm, est_roll, est_pitch,
              est_gx, est_gz, est_yaw, dt):
        if dt <= 0.0 or dt > 0.1:
            return

        self._time_since_valid_range += dt

        # -- Altitude (rangefinder), only when a fresh sample is ready --
        if range_ready:
            d_m = range_mm * 0.001
            if 0.0 < d_m < 4.0:   # Rangefinder.cpp: 0 < d < 4000mm
                h = d_m * math.cos(est_roll) * math.cos(est_pitch)
                if self.ALT_MIN <= h <= self.ALT_MAX:
                    self._z = self.ALT_ALPHA * self._z + (1.0 - self.ALT_ALPHA) * h
                    self._time_since_valid_range = 0.0

        if self._z < self.ALT_MIN:
            return

        # -- X/Y from optical flow --
        # Bench-verified 2026-07-18: the PMW3901's raw axes are swapped and
        # inverted relative to body forward/left. Mirrors the same fix in
        # Drone_Main/Position.cpp (dx, dy here are the "raw" sensor values,
        # matching FlowCountSynth's synthesis convention above).
        dx, dy = -dy, -dx

        if abs(dx) <= self.FLOW_GATE and abs(dy) <= self.FLOW_GATE:
            return

        dx_f = dx - est_gz * FOCAL_LEN * dt
        dy_f = dy - est_gx * FOCAL_LEN * dt

        disp_x = dx_f * self._z / FOCAL_LEN
        disp_y = dy_f * self._z / FOCAL_LEN

        cos_y, sin_y = math.cos(est_yaw), math.sin(est_yaw)
        disp_x_world = cos_y * disp_x - sin_y * disp_y
        disp_y_world = sin_y * disp_x + cos_y * disp_y

        self._x += disp_x_world
        self._y += disp_y_world

    def get_x(self): return self._x
    def get_y(self): return self._y
    def get_z(self): return self._z
