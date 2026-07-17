"""
Controller entry point.

Primary path: load the compiled Drone_Main controller DLL via drone_bridge.
If the DLL is unavailable (no compiler, build failed) fall back to a pure-
Python translation of the same C++ code so the simulation still runs.

Both paths expose the same DroneController class and CascadePID alias.
"""

# ---------------------------------------------------------------------------
# Try the compiled bridge first
# ---------------------------------------------------------------------------
# NOTE: importing drone_bridge does NOT itself attempt a compile -- the build
# only happens lazily, on first DroneController(...) construction. So we have
# to force that build here, at import time, or a missing compiler would only
# surface later as a hard crash instead of a graceful fallback.
_USING_DLL = False
try:
    from controller import drone_bridge
    drone_bridge.build(verbose=True)
    from controller.drone_bridge import DroneController
    _USING_DLL = True
    print("[controller] Using compiled Drone_Main controller (DLL).")
except Exception as _bridge_err:
    print(f"[controller] DLL bridge unavailable ({_bridge_err})\n"
          "             Falling back to Python translation of Drone_Main.")

# ---------------------------------------------------------------------------
# Pure-Python fallback — faithful translation of Drone_Main's C++ control loop
# ---------------------------------------------------------------------------
if not _USING_DLL:
    import numpy as np
    from controller.base_controller import BaseController
    from sim.physics_engine import PhysicsEngine

    _HOVER_THROTTLE  = 0.5
    _POSITION_HZ     = 25.0
    _MAX_TILT        = 0.1396  # rad (~8 deg) — PositionPID::MAX_TILT
    _WAYPOINT_RADIUS = 0.15    # m   — PositionPID::WAYPOINT_RADIUS

    class _PID:
        """
        Translates Drone_Main PID.cpp exactly.
        Derivative is taken on the measurement (not the error) to avoid
        output spikes when the setpoint changes suddenly.
        Integral is clamped to [out_min, out_max] (anti-windup).
        """
        def __init__(self, kp=0.0, ki=0.0, kd=0.0,
                     out_min=-1.0, out_max=1.0):
            self.kp = kp; self.ki = ki; self.kd = kd
            self._out_min   = out_min
            self._out_max   = out_max
            self._integral  = 0.0
            self._prev_meas = 0.0
            self._first_run = True

        def compute(self, error: float, measurement: float, dt: float) -> float:
            if dt <= 0:
                return float(np.clip(self.kp * error + self._integral,
                                     self._out_min, self._out_max))
            p = self.kp * error
            self._integral = float(np.clip(
                self._integral + self.ki * error * dt,
                self._out_min, self._out_max))
            d = (0.0 if self._first_run
                 else -self.kd * (measurement - self._prev_meas) / dt)
            self._prev_meas = measurement
            self._first_run = False
            return float(np.clip(p + self._integral + d,
                                 self._out_min, self._out_max))

        def reset(self):
            self._integral  = 0.0
            self._first_run = True

    class DroneController(BaseController):
        """
        Python translation of Drone_Main's Controller class.

        Outer loop  (25 Hz):  position error → tilt commands + throttle delta
        Inner loop  (full):   desired tilt + angle → normalized roll/pitch
        Yaw loop    (full):   heading error → normalized yaw moment
        Motor mix   (X-cfg):  throttle ± roll ± pitch ± yaw → [0,1] per motor

        Waypoint sequencing mirrors Drone_Main.ino::loop(): the controller
        holds position at the last waypoint indefinitely unless that
        waypoint has z == 0, in which case arrival cuts the motors (landed).
        """

        def __init__(self, engine: PhysicsEngine, gains: dict = None):
            self._engine = engine
            g = gains or {}

            # Position (outer) PIDs
            self._pid_x = _PID(g.get('kp_x', 0.25), g.get('ki_x', 0.005),
                               g.get('kd_x', 0.15), -_MAX_TILT, _MAX_TILT)
            self._pid_y = _PID(g.get('kp_y', 0.25), g.get('ki_y', 0.005),
                               g.get('kd_y', 0.15), -_MAX_TILT, _MAX_TILT)
            self._pid_z = _PID(g.get('kp_z', 0.40), g.get('ki_z', 0.02),
                               g.get('kd_z', 0.20), -0.4, 0.4)

            # Attitude (inner) PIDs
            self._pid_roll  = _PID(g.get('kp_roll',  3.5), g.get('ki_roll',  0.04),
                                   g.get('kd_roll',  0.15), -0.5, 0.5)
            self._pid_pitch = _PID(g.get('kp_pitch', 3.5), g.get('ki_pitch', 0.04),
                                   g.get('kd_pitch', 0.15), -0.5, 0.5)

            # Yaw PID
            self._pid_yaw = _PID(g.get('kp_yaw', 2.0), g.get('ki_yaw', 0.02),
                                 g.get('kd_yaw', 0.08), -0.5, 0.5)

            # Commands held between position-loop ticks
            self._cv_x          = 0.0
            self._cv_y          = 0.0
            self._cv_yaw        = 0.0
            self._base_throttle = _HOVER_THROTTLE

            # Waypoints — sequencing/landing/heading lives here, mirroring
            # Drone_Main.ino::loop() + Waypoints.h::_startLeg (Controller
            # itself only flies TO a target; it doesn't know about a mission).
            self._waypoints: list[tuple[float, float, float, bool]] = []
            self._current_wp    = 0
            self._landed        = False
            self._leg_yaw       = 0.0   # heading target for the current leg

            # Position-loop timing accumulator
            self._pos_accum     = 0.0

        # ------------------------------------------------------------------
        def add_waypoint(self, x: float, y: float, z: float, keep_heading: bool = True):
            self._waypoints.append((float(x), float(y), float(z), bool(keep_heading)))
            if len(self._waypoints) == 1:
                self._start_leg(0)   # first waypoint starts leg 0

        def clear_waypoints(self):
            self._waypoints.clear()
            self._current_wp = 0
            self._landed     = False
            self._leg_yaw    = 0.0
            self._pid_x.reset(); self._pid_y.reset(); self._pid_z.reset()

        @property
        def mission_complete(self) -> bool:
            return self._landed   # mirrors Drone_Main.ino: "complete" == landed

        @property
        def current_waypoint_index(self) -> int:
            return self._current_wp

        # ------------------------------------------------------------------
        def _start_leg(self, wp_index: int):
            """Mirrors Drone_Main.ino::_startLeg(): bearing from the previous
            waypoint (or takeoff origin) toward the leg's target, computed
            once per leg so the drone has the whole leg to turn."""
            wx, wy, _, keep_heading = self._waypoints[wp_index]
            if keep_heading:
                self._leg_yaw = 0.0
                return
            fx, fy = (0.0, 0.0) if wp_index == 0 else self._waypoints[wp_index - 1][:2]
            dx, dy = wx - fx, wy - fy
            if dx*dx + dy*dy > 1e-6:
                self._leg_yaw = float(np.arctan2(dy, dx))
            # else: degenerate leg (e.g. pure altitude change) -- keep previous heading

        # ------------------------------------------------------------------
        def update(self, state: np.ndarray, dt: float) -> np.ndarray:
            phi, theta, psi = state[6], state[7], state[8]
            x,   y,   z    = state[0], state[1], state[2]

            if self._landed:
                return np.zeros(4)

            # Waypoint sequencing + landing, mirrors Drone_Main.ino::loop():
            # check arrival at the CURRENT target, advance/land, using
            # whatever target results for this tick's control loop.
            if self._waypoints:
                wp = self._waypoints[self._current_wp]
                ex, ey, ez = wp[0] - x, wp[1] - y, wp[2] - z
                arrived = (ex*ex + ey*ey + ez*ez) ** 0.5 < _WAYPOINT_RADIUS

                if arrived and wp[2] == 0.0:
                    self._landed = True
                    return np.zeros(4)

                if arrived and self._current_wp < len(self._waypoints) - 1:
                    self._current_wp += 1
                    self._start_leg(self._current_wp)

            self._cv_yaw = self._leg_yaw

            # Safety cut (mirrors controller.cpp: 45° = 0.785 rad)
            if abs(phi) > 0.785 or abs(theta) > 0.785:
                return np.zeros(4)

            # Outer position loop (25 Hz)
            self._pos_accum += dt
            if self._pos_accum >= 1.0 / _POSITION_HZ and self._waypoints:
                tx, ty, tz, _ = self._waypoints[self._current_wp]
                self._run_position_loop(tx, ty, tz, x, y, z, self._pos_accum)
                self._pos_accum = 0.0

            # Inner attitude loop (AttitudeController::update)
            desired_pitch = self._cv_x
            desired_roll  = -self._cv_y    # positive left → negative roll

            roll_err  = desired_roll  - phi
            pitch_err = desired_pitch - theta

            att_roll  = self._pid_roll .compute(roll_err,  phi,   dt)
            att_pitch = self._pid_pitch.compute(pitch_err, theta, dt)

            # Yaw loop (YawController::update)
            yaw_err = _wrap(self._cv_yaw - psi)
            yaw_cmd = self._pid_yaw.compute(yaw_err, psi, dt)

            # Motor mix — mirrors controller.cpp _writeMotors()
            # Sim motor order: [FL-CCW, FR-CW, RR-CCW, RL-CW]
            # Drone_Main:       M1=FL,   M2=FR,  M4=RR,   M3=RL
            t, r, p, yw = self._base_throttle, att_roll, att_pitch, yaw_cmd
            return np.clip([
                t + r - p + yw,   # M1 FL-CCW
                t - r - p - yw,   # M2 FR-CW
                t - r + p + yw,   # M4 RR-CCW
                t + r + p - yw,   # M3 RL-CW
            ], 0.0, 1.0)

        def reset(self):
            for pid in (self._pid_x, self._pid_y, self._pid_z,
                        self._pid_roll, self._pid_pitch, self._pid_yaw):
                pid.reset()
            self._cv_x          = 0.0
            self._cv_y          = 0.0
            self._cv_yaw        = 0.0
            self._base_throttle = _HOVER_THROTTLE
            self._current_wp    = 0
            self._landed        = False
            self._leg_yaw       = 0.0
            self._pos_accum     = 0.0
            if self._waypoints:
                self._start_leg(0)   # recompute leg 0's heading target

        # ------------------------------------------------------------------
        def _run_position_loop(self, tx, ty, tz, x, y, z, dt_pos):
            """Mirrors PositionPID::update() — pure target-tracking, no
            waypoint awareness (that lives in update(), like Drone_Main.ino)."""
            ex, ey, ez = tx - x, ty - y, tz - z
            self._cv_x          = self._pid_x.compute(ex, x, dt_pos)
            self._cv_y          = self._pid_y.compute(ey, y, dt_pos)
            cv_z                = self._pid_z.compute(ez, z, dt_pos)
            self._base_throttle = float(np.clip(
                _HOVER_THROTTLE + cv_z, 0.0, 1.0))

    def _wrap(a: float) -> float:
        return (a + np.pi) % (2 * np.pi) - np.pi


# Backward-compatibility alias
CascadePID = DroneController
