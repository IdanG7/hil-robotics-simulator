"""Forward and inverse kinematics for 2-DOF arm."""

import numpy as np
from typing import Tuple


class ArmKinematics:
    """2-DOF planar arm kinematics.

    Link 1 (L1): Shoulder to elbow
    Link 2 (L2): Elbow to end-effector
    """

    def __init__(self, L1: float = 0.1, L2: float = 0.08):
        """
        Initialize with link lengths.

        Args:
            L1: Shoulder to elbow length (meters)
            L2: Elbow to end-effector length (meters)
        """
        self.L1 = L1
        self.L2 = L2

    def forward(self, theta1: float, theta2: float) -> Tuple[float, float]:
        """
        Compute forward kinematics.

        Args:
            theta1: Shoulder angle (radians)
            theta2: Elbow angle (radians)

        Returns:
            (x, y) end-effector position in meters
        """
        x = self.L1 * np.cos(theta1) + self.L2 * np.cos(theta1 + theta2)
        y = self.L1 * np.sin(theta1) + self.L2 * np.sin(theta1 + theta2)
        return (x, y)

    def inverse(self, x: float, y: float) -> Tuple[float, float]:
        """
        Compute inverse kinematics (elbow-up solution).

        Uses law of cosines to solve for joint angles.

        Args:
            x: Target X position (meters)
            y: Target Y position (meters)

        Returns:
            (theta1, theta2) joint angles in radians

        Raises:
            ValueError: If target unreachable
        """
        # Distance to target
        d_squared = x**2 + y**2
        d = np.sqrt(d_squared)

        # Check reachability
        if d > (self.L1 + self.L2) or d < abs(self.L1 - self.L2):
            raise ValueError(f"Target ({x:.3f}, {y:.3f}) unreachable "
                           f"(distance {d:.3f}m, max {self.L1+self.L2:.3f}m)")

        # Elbow angle using law of cosines
        cos_theta2 = (d_squared - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)

        # Clamp to [-1, 1] to handle numerical errors
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

        # Elbow-up solution (positive angle)
        theta2 = np.arccos(cos_theta2)

        # Shoulder angle
        alpha = np.arctan2(y, x)
        beta = np.arctan2(self.L2 * np.sin(theta2),
                         self.L1 + self.L2 * np.cos(theta2))
        theta1 = alpha - beta

        return (theta1, theta2)
