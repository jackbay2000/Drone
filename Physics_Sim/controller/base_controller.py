"""
Abstract base class for quadcopter controllers.
"""

from abc import ABC, abstractmethod
import numpy as np


class BaseController(ABC):
    @abstractmethod
    def update(self, state: np.ndarray, dt: float) -> np.ndarray:
        """
        Args:
            state: 12-element state vector [x,y,z, vx,vy,vz, phi,theta,psi, p,q,r]
            dt:    timestep [s]
        Returns:
            motor_commands: shape (4,), values in [0, 1]
                Motor order: [front-left, front-right, rear-right, rear-left]
        """

    @abstractmethod
    def reset(self):
        """Reset integrators / internal state."""
