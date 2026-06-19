"""
6-DOF rigid body dynamics for the drone.

State vector (17 elements):
  [0:3]  position       (m, world frame, Z-up)
  [3:6]  velocity       (m/s, world frame)
  [6:10] quaternion     (w, x, y, z) — body to world rotation
  [10:13] angular_vel   (rad/s, body frame)
  [13:17] motor_forces  (N, first-order lag on commanded force, all 4 motors)

Body frame: X = right, Y = forward, Z = up
World frame: Z = up, X/Y aligned with initial body heading

Motor torque contributions (body frame):
  τ_y (roll,  about Y forward): (r/√2) * (+F1 - F2 + F3 - F4)
  τ_x (pitch, about X right)  : (r/√2) * (+F1 + F2 - F3 - F4)
  τ_z (yaw,   about Z up CCW) : K_TORQUE * (-F1 + F2 + F3 - F4)

Sign note: the controller's pitch convention is nose-DOWN positive, and its yaw
is CW-positive.  Those sign flips are handled in ctrl.py, not here.
"""

import numpy as np

G = 9.81   # m/s²

# Drag: simple linear body-frame drag (opposing velocity in world frame)
LINEAR_DRAG_KG_PER_S = 0.3    # N per (m/s), each axis


class DronePhysics:
    def __init__(self, mass_kg, I_kg_m2, K_thrust, K_torque, arm_length_m, motor_tau):
        """
        mass_kg      : total mass
        I_kg_m2      : 3×3 inertia tensor about CoM (body frame)
        K_thrust     : thrust coefficient (N per unit [0,1] throttle) per motor
        K_torque     : yaw drag-torque coefficient (N·m per unit throttle) per motor
        arm_length_m : centre-to-motor distance (m) for roll/pitch torques
        motor_tau    : first-order motor time constant (s)
        """
        self.mass    = mass_kg
        self.I       = I_kg_m2
        self.I_inv   = np.linalg.inv(I_kg_m2)
        self.Kt      = K_thrust
        self.Kq      = K_torque
        self.arm     = arm_length_m
        self.tau     = motor_tau
        self._r      = arm_length_m / np.sqrt(2)   # per-axis arm component

        self.reset()

    # ── State access ──────────────────────────────────────────────────────────

    def reset(self):
        self._state = np.zeros(17)
        self._state[6] = 1.0    # quaternion w=1 (identity = level hover orientation)
        self._u_cmd    = np.zeros(4)   # commanded throttle per motor [0,1]

    @property
    def pos(self):   return self._state[0:3].copy()
    @property
    def vel(self):   return self._state[3:6].copy()
    @property
    def quat(self):  return self._state[6:10].copy()   # (w,x,y,z)
    @property
    def omega(self): return self._state[10:13].copy()  # body frame rad/s
    @property
    def motor_forces(self): return self._state[13:17].copy()

    def get_pos_frame(self):
        """
        Return (px, py, pz) in the firmware's position frame:
          pos_X = forward  (= world +Y when yaw=0)
          pos_Y = leftward (= world -X when yaw=0)
          pos_Z = altitude (= world +Z)
        This matches the coordinate system used by PositionPID and the waypoints.
        """
        p = self._state[0:3]
        return p[1], -p[0], p[2]   # (world_Y, -world_X, world_Z)

    def set_motor_commands(self, u: np.ndarray):
        """u: (4,) array, throttle per motor in [0,1]."""
        self._u_cmd = np.clip(u, 0.0, 1.0)

    # ── Euler angles extracted from quaternion ────────────────────────────────

    def get_roll_pitch_yaw(self):
        """
        Returns (roll, pitch, yaw) in body frame convention:
          roll  φ : rotation about Y (forward),  positive = right bank  [rad]
          pitch θ : rotation about X (right),    positive = nose UP      [rad]
          yaw   ψ : rotation about Z (up),       positive = CCW          [rad]

        These are passed through sign corrections in ctrl.py to match the
        real IMU convention.
        """
        q = self._state[6:10]
        w, x, y, z = q / np.linalg.norm(q)

        # Rotation matrix (body to world): R[:,k] = world-frame unit vector of body-k axis
        R = np.array([
            [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
            [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
            [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
        ])

        # Body Z-axis in world frame (R[:,2])
        bz = R[:, 2]   # should be [0,0,1] when level

        # Roll: rotation about body-Y axis (forward, R[:,1])
        # = angle between world-Z and body-Z projected onto body-XZ plane
        roll  =  np.arctan2(bz[0], bz[2])   # right tilt positive

        # Pitch: rotation about body-X axis (right, R[:,0])
        pitch = -np.arctan2(bz[1], bz[2])   # nose-up positive

        # Yaw: heading of body-Y (forward) projected onto world-XY plane.
        # arctan2(fwd[0], fwd[1]) is CW-positive from +Y, so negate for CCW-positive.
        fwd = R[:, 1]                         # body forward in world frame
        yaw = -np.arctan2(fwd[0], fwd[1])    # CCW from world-Y positive

        return roll, pitch, yaw

    # ── Numerical integration ─────────────────────────────────────────────────

    def step(self, dt: float):
        """Advance state by dt seconds (RK4)."""
        s = self._state
        k1 = self._derivatives(s)
        k2 = self._derivatives(s + 0.5*dt*k1)
        k3 = self._derivatives(s + 0.5*dt*k2)
        k4 = self._derivatives(s +     dt*k3)
        self._state = s + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)
        # Re-normalise quaternion to prevent drift
        q = self._state[6:10]
        self._state[6:10] = q / np.linalg.norm(q)

    def _derivatives(self, s: np.ndarray) -> np.ndarray:
        pos  = s[0:3]
        vel  = s[3:6]
        q    = s[6:10] / np.linalg.norm(s[6:10])
        w    = s[10:13]
        F_m  = s[13:17]   # actual motor forces (N, all 4, first-order lag)

        # Commanded motor forces (N)
        F_cmd = self.Kt * self._u_cmd   # all 4 motors

        # Rotation matrix
        qw, qx, qy, qz = q
        R = np.array([
            [1-2*(qy*qy+qz*qz),   2*(qx*qy-qw*qz),   2*(qx*qz+qw*qy)],
            [  2*(qx*qy+qw*qz), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qw*qx)],
            [  2*(qx*qz-qw*qy),   2*(qy*qz+qw*qx), 1-2*(qx*qx+qy*qy)],
        ])

        # Total thrust (along body Z)
        F_thrust = np.sum(F_m)
        thrust_world = R @ np.array([0.0, 0.0, F_thrust])

        # Gravity + drag
        grav  = np.array([0.0, 0.0, -self.mass * G])
        drag  = -LINEAR_DRAG_KG_PER_S * vel

        # Linear dynamics
        d_pos = vel
        d_vel = (thrust_world + grav + drag) / self.mass

        # Body-frame torques
        r = self._r
        F1, F2, F3, F4 = F_m   # all lagged
        tau_x = r * ( F1 + F2 - F3 - F4)   # pitch (nose-up positive)
        tau_y = r * ( F1 - F2 + F3 - F4)   # roll  (right-bank positive)
        tau_z = self.Kq * (-F1 + F2 + F3 - F4)  # yaw (CCW positive)
        tau = np.array([tau_x, tau_y, tau_z])

        # Angular dynamics (Euler's equations)
        Iw     = self.I @ w
        d_omega = self.I_inv @ (tau - np.cross(w, Iw))

        # Quaternion kinematics:  dq/dt = 0.5 * q ⊗ [0, ω]
        wx, wy, wz = w
        Omega = np.array([
            [ 0,  -wx, -wy, -wz],
            [wx,    0,  wz, -wy],
            [wy,  -wz,   0,  wx],
            [wz,   wy, -wx,   0],
        ])
        d_q = 0.5 * Omega @ q

        # Motor force lag for all 4 motors
        d_Fm = (F_cmd - F_m) / self.tau

        ds = np.empty(17)
        ds[0:3]  = d_pos
        ds[3:6]  = d_vel
        ds[6:10] = d_q
        ds[10:13]= d_omega
        ds[13:17]= d_Fm
        return ds
