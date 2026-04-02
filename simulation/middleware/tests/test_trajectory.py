"""Tests for trajectory generation."""

import pytest
import numpy as np
from middleware.trajectory import TrajectoryPlanner


class TestLinearInterpolation:
    def test_linear_single_segment(self):
        planner = TrajectoryPlanner()
        waypoints = [(0.0, 0.0), (1.0, 1.0)]
        trajectory = planner.linear(waypoints, num_points=11)
        assert len(trajectory) == 11
        assert trajectory[0] == pytest.approx((0.0, 0.0), abs=1e-6)
        assert trajectory[5] == pytest.approx((0.5, 0.5), abs=1e-6)
        assert trajectory[10] == pytest.approx((1.0, 1.0), abs=1e-6)

    def test_linear_multiple_segments(self):
        planner = TrajectoryPlanner()
        waypoints = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
        trajectory = planner.linear(waypoints, num_points=21)
        assert len(trajectory) == 21
        assert trajectory[0] == pytest.approx((0.0, 0.0), abs=1e-6)
        assert trajectory[10] == pytest.approx((1.0, 0.0), abs=1e-6)
        assert trajectory[20] == pytest.approx((1.0, 1.0), abs=1e-6)


class TestCubicSpline:
    def test_cubic_smooth_velocity(self):
        planner = TrajectoryPlanner()
        waypoints = [(0.0, 0.0), (0.5, 0.5), (1.0, 0.0)]
        trajectory = planner.cubic_spline(waypoints, num_points=101)
        assert len(trajectory) == 101
        assert trajectory[0] == pytest.approx((0.0, 0.0), abs=1e-6)
        assert trajectory[100] == pytest.approx((1.0, 0.0), abs=1e-6)

        velocities = []
        for i in range(1, len(trajectory)):
            dx = trajectory[i][0] - trajectory[i-1][0]
            dy = trajectory[i][1] - trajectory[i-1][1]
            velocities.append((dx, dy))

        avg_speed = np.mean([np.sqrt(v[0]**2 + v[1]**2) for v in velocities])
        for i in range(1, len(velocities)):
            dv = np.sqrt((velocities[i][0] - velocities[i-1][0])**2 +
                        (velocities[i][1] - velocities[i-1][1])**2)
            assert dv < avg_speed * 0.5

    def test_cubic_passes_through_waypoints(self):
        planner = TrajectoryPlanner()
        waypoints = [(0.0, 0.0), (0.1, 0.05), (0.15, 0.0)]
        trajectory = planner.cubic_spline(waypoints, num_points=101)
        for wp in waypoints:
            distances = [np.sqrt((p[0]-wp[0])**2 + (p[1]-wp[1])**2) for p in trajectory]
            assert min(distances) < 0.01


class TestFigure8:
    def test_figure8_shape(self):
        planner = TrajectoryPlanner()
        trajectory = planner.figure8(center=(0.12, 0.0), width=0.04, height=0.03, num_points=100)
        assert len(trajectory) == 100
        xs = [p[0] for p in trajectory]
        ys = [p[1] for p in trajectory]
        assert max(xs) - min(xs) >= 0.035
        # Lemniscate y-range is approximately 2/3 of height parameter due to sin*cos factor
        assert max(ys) - min(ys) >= 0.02
        assert abs(np.mean(xs) - 0.12) < 0.01
        assert abs(np.mean(ys) - 0.0) < 0.01

    def test_figure8_closed_loop(self):
        planner = TrajectoryPlanner()
        trajectory = planner.figure8(center=(0.12, 0.0), width=0.04, height=0.03, num_points=100)
        dist = np.sqrt((trajectory[0][0] - trajectory[-1][0])**2 +
                      (trajectory[0][1] - trajectory[-1][1])**2)
        assert dist < 0.005


class TestCartesianToJoint:
    def test_to_joint_space_valid(self):
        planner = TrajectoryPlanner()
        cartesian = [(0.15, 0.0), (0.14, 0.02), (0.13, 0.04)]
        joint_trajectory = planner.to_joint_space(cartesian)
        assert len(joint_trajectory) == 3
        for angles in joint_trajectory:
            assert len(angles) == 2
            assert -np.pi/2 <= angles[0] <= np.pi/2
            assert -np.pi/2 <= angles[1] <= np.pi/2

    def test_to_joint_space_roundtrip(self):
        planner = TrajectoryPlanner()
        cartesian = [(0.15, 0.0), (0.12, 0.05)]
        joint_trajectory = planner.to_joint_space(cartesian)
        for i, (x_orig, y_orig) in enumerate(cartesian):
            theta1, theta2 = joint_trajectory[i]
            x_check, y_check = planner.kinematics.forward(theta1, theta2)
            assert abs(x_check - x_orig) < 0.001
            assert abs(y_check - y_orig) < 0.001
