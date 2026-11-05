"""Interface every robot backend implements (MuJoCo sim now, servos later).

The control loop only talks to this, so the same policy drives sim and hardware.
Joint angles are radians in the policy's joint order (kinematics.ACTUATOR_ORDER).
Body frame: x forward, y left, z up.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class RobotState:
    time: float  # s
    joint_positions: np.ndarray  # (12,) rad
    joint_velocities: np.ndarray  # (12,) rad/s
    orientation: np.ndarray  # (4,) quaternion w, x, y, z: body -> world
    gyro: np.ndarray  # (3,) rad/s, body frame
    accelerometer: np.ndarray  # (3,) m/s^2, body frame; reads +9.81 z when level and still


class RobotBackend(ABC):
    @abstractmethod
    def reset(self, joint_positions):
        """Put the robot in a known pose, standing still."""

    @abstractmethod
    def read_state(self) -> RobotState:
        """Latest sensor readings."""

    @abstractmethod
    def send_joint_targets(self, joint_positions):
        """Command the 12 joint angles."""

    @abstractmethod
    def advance(self, dt):
        """Let dt seconds pass: the sim steps physics, hardware waits for real time."""

    def is_running(self):
        """False once the robot should stop (viewer closed, e-stop, ...)."""
        return True

    def close(self):
        """Release windows, ports, etc."""
