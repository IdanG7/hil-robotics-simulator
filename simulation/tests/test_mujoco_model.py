"""Tests for MuJoCo arm model."""

import pytest
import os
import xml.etree.ElementTree as ET

# Try to import mujoco, skip tests if not available
try:
    import mujoco
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False


class TestArmModelBasic:
    """Basic tests that don't require MuJoCo."""

    @pytest.fixture
    def model_path(self):
        """Path to arm.xml model."""
        base = os.path.dirname(os.path.dirname(__file__))
        return os.path.join(base, 'mujoco_model', 'arm.xml')

    def test_model_file_exists(self, model_path):
        """Model file should exist."""
        assert os.path.exists(model_path), f"Model file not found at {model_path}"

    def test_model_is_valid_xml(self, model_path):
        """Model should be valid XML."""
        tree = ET.parse(model_path)
        root = tree.getroot()
        assert root.tag == 'mujoco'

    def test_model_has_required_elements(self, model_path):
        """Model should have worldbody, actuator, and sensor sections."""
        tree = ET.parse(model_path)
        root = tree.getroot()

        worldbody = root.find('worldbody')
        assert worldbody is not None, "Missing worldbody element"

        actuator = root.find('actuator')
        assert actuator is not None, "Missing actuator element"

        sensor = root.find('sensor')
        assert sensor is not None, "Missing sensor element"


@pytest.mark.skipif(not MUJOCO_AVAILABLE, reason="MuJoCo not installed")
class TestArmModel:
    """Test MuJoCo MJCF model validity."""

    @pytest.fixture
    def model_path(self):
        """Path to arm.xml model."""
        base = os.path.dirname(os.path.dirname(__file__))
        return os.path.join(base, 'mujoco_model', 'arm.xml')

    def test_model_loads(self, model_path):
        """Model should load without errors."""
        model = mujoco.MjModel.from_xml_path(model_path)
        assert model is not None

    def test_model_has_two_joints(self, model_path):
        """Model should have shoulder and elbow joints."""
        model = mujoco.MjModel.from_xml_path(model_path)
        assert model.njnt == 2  # 2 joints
        assert model.nq == 2    # 2 joint positions

    def test_model_has_actuators(self, model_path):
        """Model should have 2 actuators."""
        model = mujoco.MjModel.from_xml_path(model_path)
        assert model.nu == 2  # 2 actuators

    def test_model_joint_ranges(self, model_path):
        """Joints should have correct range limits."""
        model = mujoco.MjModel.from_xml_path(model_path)
        # Joint ranges: [-π/2, π/2]
        for i in range(2):
            jnt_range = model.jnt_range[i]
            assert abs(jnt_range[0] - (-1.57)) < 0.01
            assert abs(jnt_range[1] - 1.57) < 0.01
