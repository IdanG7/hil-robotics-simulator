"""Trajectory generation for robotic arm motion planning."""

import numpy as np
from typing import List, Tuple
from scipy.interpolate import CubicSpline
from middleware.kinematics import ArmKinematics


class TrajectoryPlanner:
    """Generate smooth trajectories for 2-DOF arm."""

    def __init__(self, kinematics: ArmKinematics = None):
        self.kinematics = kinematics or ArmKinematics()

    def linear(self, waypoints: List[Tuple[float, float]],
               num_points: int = 100) -> List[Tuple[float, float]]:
        """Generate linear interpolation through waypoints."""
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints")

        segments = len(waypoints) - 1
        points_per_segment = (num_points - 1) // segments

        trajectory = []
        for i in range(segments):
            start = np.array(waypoints[i])
            end = np.array(waypoints[i + 1])

            if i == segments - 1:
                n = num_points - len(trajectory)
            else:
                n = points_per_segment + 1

            for j in range(n):
                if i == segments - 1 or j < n - 1:
                    t = j / (n - 1) if n > 1 else 0
                    point = start + t * (end - start)
                    trajectory.append(tuple(point))

        return trajectory

    def cubic_spline(self, waypoints: List[Tuple[float, float]],
                     num_points: int = 100) -> List[Tuple[float, float]]:
        """Generate cubic spline interpolation through waypoints."""
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints")

        waypoints = np.array(waypoints)
        t = np.zeros(len(waypoints))
        for i in range(1, len(waypoints)):
            t[i] = t[i-1] + np.linalg.norm(waypoints[i] - waypoints[i-1])

        if t[-1] > 0:
            t = t / t[-1]

        cs_x = CubicSpline(t, waypoints[:, 0], bc_type='natural')
        cs_y = CubicSpline(t, waypoints[:, 1], bc_type='natural')

        t_new = np.linspace(0, 1, num_points)
        trajectory = [(float(cs_x(ti)), float(cs_y(ti))) for ti in t_new]

        return trajectory

    def figure8(self, center: Tuple[float, float] = (0.12, 0.0),
                width: float = 0.04, height: float = 0.03,
                num_points: int = 100) -> List[Tuple[float, float]]:
        """Generate figure-8 (lemniscate) trajectory."""
        trajectory = []

        for i in range(num_points):
            t = 2 * np.pi * i / num_points

            denom = 1 + np.sin(t)**2
            x = center[0] + (width / 2) * np.cos(t) / denom
            y = center[1] + (height) * np.sin(t) * np.cos(t) / denom

            trajectory.append((x, y))

        return trajectory

    def to_joint_space(self, cartesian_trajectory: List[Tuple[float, float]]
                       ) -> List[Tuple[float, float]]:
        """Convert Cartesian trajectory to joint-space angles."""
        joint_trajectory = []

        for i, (x, y) in enumerate(cartesian_trajectory):
            try:
                theta1, theta2 = self.kinematics.inverse(x, y)
                joint_trajectory.append((theta1, theta2))
            except ValueError as e:
                raise ValueError(f"Point {i} ({x:.3f}, {y:.3f}) unreachable: {e}")

        return joint_trajectory
