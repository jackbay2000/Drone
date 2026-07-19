"""
Reproduces the real hardware's control/sensor cadence AND runs the real
estimator algorithms inside the physics sim.

Without this, the sim was calling Controller::update() every physics
integration step (500 Hz at the default dt, or 250 Hz during tuning) and
feeding it the true, instantaneous, noiseless simulated state. Real hardware
differs in three ways that matter for gain tuning:

  1. Controller cadence -- Drone_Main.ino::loop() gates the whole loop body
     (imu.update(), position.update(), the attitude/yaw loop, and the 25 Hz
     position loop nested inside Controller::update()) to 250 Hz via
     LOOP_PERIOD_US=4000.

  2. Altitude refresh -- Rangefinder.cpp sets the VL53L1X timing budget to
     20 ms ("up to 50 Hz") and only refreshes when dataReady(); roll/pitch/
     yaw/x/y are recomputed every 250 Hz tick same as real hardware.

  3. Sensor estimation -- sim/estimator.py is a faithful port of
     Drone_Main/IMU.cpp's complementary filter and Drone_Main/Position.cpp's
     optical-flow/rangefinder fusion (not a statistical approximation of
     their effects). This module synthesizes raw sensor readings from the
     true physics state (sim/physics_engine.py::true_sensor_frame) and runs
     them through those ported algorithms every control tick, exactly
     mirroring Drone_Main.ino::loop() calling imu.update() and
     position.update() right before controller.update().

Motor commands are zero-order-held between control ticks, matching real
ESC PWM: the value written to writeMicroseconds() stays on the line
unchanged until the next loop iteration rewrites it.
"""

import numpy as np

from sim.estimator import ComplementaryFilter, PositionEstimator, FlowCountSynth, synth_raw_imu, synth_raw_range_mm

G = 9.81

CONTROL_HZ     = 250.0  # Drone_Main.ino: LOOP_PERIOD_US = 4000
RANGEFINDER_HZ = 50.0   # Rangefinder.cpp: setTimingBudget(20) -> up to 50 Hz

# Raw-sensor noise/dropout. Applied at the RAW reading level (before the
# ported estimator algorithms run), so calibration, the median-of-3 spike
# filter, axis remap, in-flight yaw-bias learning, and the FLOW_GATE/
# altitude-window rejection all see and react to it exactly as real firmware
# would.
#
# Grounded in published datasheet specs where available (2026-07-18):
#   - Gyro: MPU6050 datasheet total RMS noise spec is 0.05 deg/s (rate noise
#     spectral density 0.005 deg/s/sqrt(Hz)); an independent bench measurement
#     at 500 Hz found 0.0331 deg/s (Pololu forum). 0.05 deg/s used directly.
#   - Accel: MPU6050 datasheet noise density is ~400 ug/sqrt(Hz). RMS at our
#     ~250 Hz sample rate: 400e-6 * sqrt(250) ~= 0.0063 g. Real-world (mounting,
#     temperature) typically exceeds the datasheet figure, so this is a floor,
#     not a ceiling -- the separate vibration terms below add the rest.
#   - Rangefinder: VL53L1X datasheet typical accuracy is ~+-5mm under good
#     conditions; used a touch looser (7mm) for realistic indoor conditions.
#   - Flow: no public PMW3901 noise-count spec found, and this sensor's
#     count magnitude is inherently tied to Position.cpp's own uncalibrated
#     FOCAL_LEN constant anyway (see project memory "estimator port") --
#     left as an engineering estimate pending real bench calibration.
DEFAULT_ACCEL_NOISE_STD_G   = 0.006  # MPU6050 datasheet: ~400 ug/sqrt(Hz) @ ~250Hz
DEFAULT_GYRO_NOISE_STD_DPS  = 0.05   # MPU6050 datasheet: total RMS noise spec
DEFAULT_FLOW_NOISE_STD      = 1.0    # raw counts -- engineering estimate, no public spec (see above)
DEFAULT_FLOW_DROPOUT_PROB   = 0.03   # per control tick (lost optical-flow lock) -- engineering estimate
DEFAULT_RANGE_NOISE_STD_M   = 0.007  # VL53L1X datasheet: ~+-5mm typical accuracy, +margin
DEFAULT_RANGE_DROPOUT_PROB  = 0.05   # per scheduled 50 Hz sample (rejected reading) -- engineering estimate

# Prop/motor vibration coupling into the IMU -- NOT modeled at all before
# this. Real vibration is periodic (motor RPM and its harmonics, prop
# blade-pass frequency) and usually well above the 250 Hz sample rate, so it
# aliases into the sampled signal rather than showing up as a clean tone --
# properly simulating that aliasing needs real per-motor RPM and a
# vibration spectrum neither of which is measured here. This models only
# the practically-important consequence: more commanded thrust -> more
# accelerometer/gyro noise, approximated as extra white noise whose std
# scales linearly with average commanded throttle (0..1) on top of the
# baseline sensor noise above. Coarse by design; replace with real
# accelerometer-logged vibration data if/when available.
DEFAULT_VIBRATION_ACCEL_STD_G   = 0.03   # additional accel noise std at full throttle
DEFAULT_VIBRATION_GYRO_STD_DPS  = 0.15   # additional gyro noise std at full throttle


class ControlLoopTiming:
    def __init__(self, engine, control_hz: float = CONTROL_HZ, rangefinder_hz: float = RANGEFINDER_HZ,
                 sensor_noise: bool = True, rng: np.random.Generator = None,
                 accel_noise_std_g: float = DEFAULT_ACCEL_NOISE_STD_G,
                 gyro_noise_std_dps: float = DEFAULT_GYRO_NOISE_STD_DPS,
                 flow_noise_std: float = DEFAULT_FLOW_NOISE_STD,
                 flow_dropout_prob: float = DEFAULT_FLOW_DROPOUT_PROB,
                 range_noise_std_m: float = DEFAULT_RANGE_NOISE_STD_M,
                 range_dropout_prob: float = DEFAULT_RANGE_DROPOUT_PROB,
                 vibration_accel_std_g: float = DEFAULT_VIBRATION_ACCEL_STD_G,
                 vibration_gyro_std_dps: float = DEFAULT_VIBRATION_GYRO_STD_DPS):
        self.engine = engine
        self.control_period = 1.0 / control_hz
        self.range_period    = 1.0 / rangefinder_hz
        self._rng = rng if rng is not None else np.random.default_rng()

        self.sensor_noise = sensor_noise
        self._accel_noise_std_g  = accel_noise_std_g  if sensor_noise else 0.0
        self._gyro_noise_std_dps = gyro_noise_std_dps if sensor_noise else 0.0
        self._flow_noise_std     = flow_noise_std     if sensor_noise else 0.0
        self._flow_dropout_prob  = flow_dropout_prob  if sensor_noise else 0.0
        self._range_noise_std_m  = range_noise_std_m  if sensor_noise else 0.0
        self._range_dropout_prob = range_dropout_prob if sensor_noise else 0.0
        self._vibration_accel_std_g  = vibration_accel_std_g  if sensor_noise else 0.0
        self._vibration_gyro_std_dps = vibration_gyro_std_dps if sensor_noise else 0.0

        self._t_since_control = 0.0
        self._t_since_range   = 0.0
        self._last_cmd        = np.zeros(4)
        self._filter  = ComplementaryFilter()
        self._pos_est = PositionEstimator()
        self._flow    = FlowCountSynth()

    def reset(self, state0: np.ndarray):
        """state0: the 12-element true state to calibrate/seed from (typically engine.reset())."""
        self._t_since_control = 0.0
        self._t_since_range   = 0.0
        self._last_cmd        = np.zeros(4)
        self._filter  = ComplementaryFilter()
        self._pos_est = PositionEstimator()
        self._pos_est.reset()
        self._flow    = FlowCountSynth()

        # Calibration mirrors IMU::setup()'s ~5s / 1000-sample average while
        # stationary and level -- on real hardware this happens on the
        # ground, SUPPORTED against gravity, reading +1g on Z. This is
        # deliberately NOT engine.true_sensor_frame(state0): state0 is
        # airborne with the motors freshly zeroed (engine.reset() sets
        # every motor's lag state to 0 thrust), which is physically
        # free-fall (specific force = 0), not "resting on the ground" --
        # using that would calibrate the roll/pitch offsets against a
        # degenerate atan2(0,0) reference and corrupt every estimate
        # downstream.
        f_body_level  = np.array([0.0, 0.0, G])
        omega_body_zero = np.array([0.0, 0.0, 0.0])
        samples = [
            synth_raw_imu(f_body_level, omega_body_zero, self._rng,
                          accel_noise_std_g=self._accel_noise_std_g,
                          gyro_noise_std_dps=self._gyro_noise_std_dps)
            for _ in range(1000)
        ]
        self._filter.calibrate(samples)

    def step(self, controller, true_state: np.ndarray, dt: float) -> np.ndarray:
        """
        Advance the sensor/control clocks by one physics dt. At most once
        every control_period, synthesizes raw IMU/flow/rangefinder readings
        from the true state and runs them through the ported estimator
        (sim/estimator.py), then calls the real controller on the resulting
        sensed state -- mirroring Drone_Main.ino::loop() calling
        imu.update(); position.update(...); controller.update(...) every
        250 Hz tick. Returns the motor command to apply for this physics
        step (held between control ticks).
        """
        self._t_since_control += dt
        self._t_since_range   += dt

        if self._t_since_control >= self.control_period:
            control_dt = self._t_since_control
            self._t_since_control -= self.control_period

            range_ready = False
            if self._t_since_range >= self.range_period:
                range_ready = True
                self._t_since_range -= self.range_period

            # Vibration scales with commanded thrust; use the previous tick's
            # command (self._last_cmd) as the proxy for current motor speed
            # since motor lag means actual RPM tracks the command, not the
            # instant it changes.
            avg_throttle = float(np.mean(self._last_cmd))
            accel_std = self._accel_noise_std_g + self._vibration_accel_std_g * avg_throttle
            gyro_std  = self._gyro_noise_std_dps + self._vibration_gyro_std_dps * avg_throttle

            f_body, omega_body, R = self.engine.true_sensor_frame(true_state)
            ax, ay, az, gx, gy, gz = synth_raw_imu(
                f_body, omega_body, self._rng,
                accel_noise_std_g=accel_std,
                gyro_noise_std_dps=gyro_std)

            v_body = R.T @ true_state[3:6]
            gz_true, gx_true = omega_body[2], omega_body[0]
            z_true, roll_true, pitch_true = true_state[2], true_state[6], true_state[7]

            dx, dy = self._flow.sample(
                v_body[0:2], gz_true, gx_true, z_true, control_dt, self._rng,
                count_noise_std=self._flow_noise_std, dropout_prob=self._flow_dropout_prob)

            range_mm = 0
            if range_ready:
                range_mm = synth_raw_range_mm(
                    z_true, roll_true, pitch_true, self._rng,
                    noise_std_m=self._range_noise_std_m, dropout_prob=self._range_dropout_prob)

            self._filter.update(ax, ay, az, gx, gy, gz, control_dt)
            self._pos_est.update(
                dx, dy, range_ready, range_mm,
                self._filter.get_roll(), self._filter.get_pitch(),
                self._filter.get_gx(), self._filter.get_gz(), self._filter.get_yaw(),
                control_dt)

            sensed_state = true_state.copy()
            sensed_state[0] = self._pos_est.get_x()
            sensed_state[1] = self._pos_est.get_y()
            sensed_state[2] = self._pos_est.get_z()
            sensed_state[6] = self._filter.get_roll()
            sensed_state[7] = self._filter.get_pitch()
            sensed_state[8] = self._filter.get_yaw()

            self._last_cmd = controller.update(sensed_state, control_dt)

        return self._last_cmd
