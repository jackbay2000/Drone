"""
6-DOF rigid body quadcopter dynamics.

State vector (12 elements):
  [0:3]  position     x, y, z         [m]       world frame
  [3:6]  velocity     vx, vy, vz      [m/s]     world frame
  [6:9]  euler angles phi, theta, psi [rad]      roll, pitch, yaw (ZYX)
  [9:12] body rates   p, q, r         [rad/s]    body frame

Body frame: +x=forward, +y=left, +z=up (right-handed).
Positive roll  (phi)   = left side up  (right-hand about +x)
Positive pitch (theta) = nose down     (right-hand about +y)
Positive yaw   (psi)   = nose left     (right-hand about +z)
"""

import numpy as np
from sim.vehicle import VehicleParams
from sim.motor_model import MotorModel

G = 9.81  # m/s^2


class PhysicsEngine:
    def __init__(self, vehicle: VehicleParams, motor: MotorModel):
        self.vehicle = vehicle
        self.motor = motor
        self.I_inv = np.linalg.inv(vehicle.inertia)
        self.A_mix = vehicle.mixer_matrix(motor.torque_ratio)
        self.A_mix_inv = np.linalg.pinv(self.A_mix)

        # One MotorModel instance per motor (independent lag states)
        self._motors = [MotorModel() for _ in range(4)]
        for m in self._motors:
            m.a = motor.a
            m.b = motor.b
            m.torque_ratio = motor.torque_ratio
            m.time_constant = motor.time_constant

    # ------------------------------------------------------------------
    def reset(self, state: np.ndarray = None) -> np.ndarray:
        """Returns initial state. Defaults to hover at z=1m."""
        for m in self._motors:
            m._actual_throttle = 0.0
        if state is not None:
            return state.copy()
        s = np.zeros(12)
        s[2] = 1.0  # start 1 m up
        return s

    # ------------------------------------------------------------------
    def motor_commands_from_wrench(self, F: float, tau: np.ndarray) -> np.ndarray:
        """
        Given desired total thrust F [N] and body torques tau [N·m],
        return per-motor throttle commands in [0,1].
        """
        desired = np.array([F, tau[0], tau[1], tau[2]])
        thrusts = self.A_mix_inv @ desired
        thrusts = np.clip(thrusts, 0.0, self.motor.max_thrust_N)
        return np.array([self.motor.throttle_from_thrust(t) for t in thrusts])

    # ------------------------------------------------------------------
    def step(self, state: np.ndarray, motor_commands: np.ndarray, dt: float) -> np.ndarray:
        """RK4 integration step. Returns new state."""
        # Advance motor lag states
        actual = np.array([self._motors[i].step(motor_commands[i], dt) for i in range(4)])

        def derivatives(s):
            return self._deriv(s, actual)

        k1 = derivatives(state)
        k2 = derivatives(state + 0.5 * dt * k1)
        k3 = derivatives(state + 0.5 * dt * k2)
        k4 = derivatives(state + dt * k3)
        new_state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        # Keep yaw in (-pi, pi]
        new_state[8] = _wrap_angle(new_state[8])
        return new_state

    # ------------------------------------------------------------------
    def _deriv(self, s: np.ndarray, throttles: np.ndarray) -> np.ndarray:
        x, y, z       = s[0:3]
        vx, vy, vz    = s[3:6]
        phi, theta, psi = s[6:9]
        p, q, r       = s[9:12]

        # Per-motor thrust and reaction torques
        thrusts = np.array([self._motors[i].thrust(throttles[i]) for i in range(4)])
        F_total = float(np.sum(thrusts))

        # Body-frame torques
        pos  = self.vehicle.motor_positions
        signs = self.vehicle.motor_yaw_signs
        tau_x = float(np.sum(pos[:, 1] * thrusts))
        tau_y = float(np.sum(-pos[:, 0] * thrusts))
        tau_z = float(np.sum(signs * self.motor.torque_ratio * thrusts))

        # Rotation matrix: body → world (ZYX Euler)
        R = _rotation_matrix(phi, theta, psi)

        # Forces in world frame
        F_body = np.array([0.0, 0.0, F_total])
        drag_body = -self.vehicle.drag_linear * np.array([vx, vy, vz])  # approx in world
        gravity_world = np.array([0.0, 0.0, -G * self.vehicle.mass_kg])
        F_world = R @ F_body + gravity_world + drag_body

        a_world = F_world / self.vehicle.mass_kg

        # Euler angle rates from body rates
        dphi, dtheta, dpsi = _euler_rates(phi, theta, p, q, r)

        # Angular acceleration (Euler's equations)
        omega = np.array([p, q, r])
        tau = np.array([tau_x, tau_y, tau_z])
        tau_drag = -self.vehicle.drag_rotational * omega
        omega_dot = self.I_inv @ (tau + tau_drag - np.cross(omega, self.vehicle.inertia @ omega))

        return np.array([
            vx, vy, vz,              # d/dt position
            a_world[0], a_world[1], a_world[2],  # d/dt velocity
            dphi, dtheta, dpsi,      # d/dt euler angles
            omega_dot[0], omega_dot[1], omega_dot[2],  # d/dt body rates
        ])


# ------------------------------------------------------------------
def _rotation_matrix(phi, theta, psi):
    """ZYX Euler rotation matrix (body to world)."""
    cp, sp = np.cos(phi),   np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cy, sy = np.cos(psi),   np.sin(psi)
    return np.array([
        [cy*ct,  cy*st*sp - sy*cp,  cy*st*cp + sy*sp],
        [sy*ct,  sy*st*sp + cy*cp,  sy*st*cp - cy*sp],
        [-st,    ct*sp,              ct*cp            ],
    ])


def _euler_rates(phi, theta, p, q, r):
    """Map body angular rates to Euler angle rates (ZYX convention)."""
    cp, sp = np.cos(phi), np.sin(phi)
    ct, tt = np.cos(theta), np.tan(theta)
    # Singularity at theta = ±90°; acceptable for typical flight
    dphi   = p + (q*sp + r*cp) * tt
    dtheta = q*cp - r*sp
    dpsi   = (q*sp + r*cp) / (ct + 1e-9)
    return dphi, dtheta, dpsi


def _wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi
