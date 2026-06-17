import numpy as np
import json
import os

_CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', 'config', 'motor_config.json')


class MotorModel:
    """
    Single motor thrust/torque model with first-order lag.

    Thrust curve derived from Emax RS2205S 2300KV + Gemfan 5045 prop
    tested on 4S, scaled to 3S (factor = (11.1/14.8)^2 = 0.5625).

    T(u) = a*u^2 + b*u  [Newtons],  u in [0, 1]
    Q(u) = torque_ratio * T(u)       [N·m]
    """

    def __init__(self, config_path=None):
        path = config_path or _CONFIG_PATH
        with open(path) as f:
            cfg = json.load(f)

        tc = cfg['thrust_curve']
        self.a = tc['a']
        self.b = tc['b']
        self.torque_ratio = cfg['torque_ratio']
        self.prop_radius   = cfg['prop_diameter_m'] / 2.0
        self.time_constant = cfg['time_constant_s']

        self._actual_throttle = 0.0

    # ------------------------------------------------------------------
    def thrust(self, throttle: float) -> float:
        u = np.clip(throttle, 0.0, 1.0)
        return self.a * u**2 + self.b * u

    def torque(self, throttle: float) -> float:
        return self.torque_ratio * self.thrust(throttle)

    def throttle_from_thrust(self, T_newtons: float) -> float:
        """Invert the thrust curve; clamps to [0, 1]."""
        T = max(T_newtons, 0.0)
        disc = self.b**2 + 4.0 * self.a * T
        u = (-self.b + np.sqrt(disc)) / (2.0 * self.a)
        return float(np.clip(u, 0.0, 1.0))

    def step(self, command: float, dt: float) -> float:
        """First-order lag: returns actual throttle after one timestep."""
        alpha = dt / (self.time_constant + dt)
        self._actual_throttle += alpha * (np.clip(command, 0.0, 1.0) - self._actual_throttle)
        return self._actual_throttle

    @property
    def max_thrust_N(self) -> float:
        return self.thrust(1.0)
