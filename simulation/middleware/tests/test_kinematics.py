"""Tests for arm kinematics."""

import pytest
import numpy as np
from middleware.kinematics import ArmKinematics


class TestForwardKinematics:
    """Test forward kinematics computation."""

    @pytest.fixture
    def arm(self):
        """Standard arm with L1=0.1m, L2=0.08m."""
        return ArmKinematics(L1=0.1, L2=0.08)

    def test_forward_zero_angles(self, arm):
        """At zero angles, end effector should be at (L1+L2, 0)."""
        x, y = arm.forward(0.0, 0.0)
        assert abs(x - 0.18) < 0.001  # 0.1 + 0.08
        assert abs(y - 0.0) < 0.001

    def test_forward_45_degrees(self, arm):
        """Test at 45 degrees shoulder, 0 elbow."""
        theta1 = np.pi / 4  # 45 degrees
        x, y = arm.forward(theta1, 0.0)
        # Expected: x = (L1+L2)*cos(45°), y = (L1+L2)*sin(45°)
        expected_x = 0.18 * np.cos(theta1)
        expected_y = 0.18 * np.sin(theta1)
        assert abs(x - expected_x) < 0.001
        assert abs(y - expected_y) < 0.001

    def test_forward_with_elbow_bend(self, arm):
        """Test with both joints at angles."""
        theta1 = np.pi / 6   # 30 degrees
        theta2 = -np.pi / 4  # -45 degrees
        x, y = arm.forward(theta1, theta2)
        # Expected from trig:
        # x = L1*cos(θ1) + L2*cos(θ1+θ2)
        # y = L1*sin(θ1) + L2*sin(θ1+θ2)
        L1, L2 = 0.1, 0.08
        expected_x = L1*np.cos(theta1) + L2*np.cos(theta1 + theta2)
        expected_y = L1*np.sin(theta1) + L2*np.sin(theta1 + theta2)
        assert abs(x - expected_x) < 0.001
        assert abs(y - expected_y) < 0.001


class TestInverseKinematics:
    """Test inverse kinematics computation."""

    @pytest.fixture
    def arm(self):
        return ArmKinematics(L1=0.1, L2=0.08)

    def test_inverse_forward_consistency(self, arm):
        """FK(IK(x,y)) should equal (x,y)."""
        # Start with known angles
        theta1_orig = 0.5
        theta2_orig = -0.3

        # Compute FK
        x, y = arm.forward(theta1_orig, theta2_orig)

        # Compute IK
        theta1, theta2 = arm.inverse(x, y)

        # Verify FK matches
        x_check, y_check = arm.forward(theta1, theta2)
        assert abs(x_check - x) < 0.001
        assert abs(y_check - y) < 0.001

    def test_inverse_unreachable_target(self, arm):
        """Should raise ValueError for unreachable targets."""
        # Target too far (beyond L1+L2)
        with pytest.raises(ValueError, match="unreachable"):
            arm.inverse(0.5, 0.5)  # Distance > 0.18m

    def test_inverse_at_origin(self, arm):
        """Target at origin should be unreachable (requires negative lengths)."""
        with pytest.raises(ValueError):
            arm.inverse(0.0, 0.0)
