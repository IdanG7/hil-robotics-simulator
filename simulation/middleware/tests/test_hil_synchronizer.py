"""Tests for hil_synchronizer module."""

import pytest
from unittest.mock import Mock, patch, MagicMock
from middleware.hil_synchronizer import HILSynchronizer


class TestHILSynchronizerInit:
    """Test HILSynchronizer initialization."""

    @patch('mujoco.MjModel.from_xml_path')
    @patch('middleware.hil_synchronizer.SerialManager')
    def test_init_loads_model(self, mock_serial_class, mock_model_load):
        """Should load MuJoCo model and initialize serial manager."""
        mock_model = Mock()
        mock_model_load.return_value = mock_model
        mock_serial = Mock()
        mock_serial_class.return_value = mock_serial

        hil = HILSynchronizer(port='COM3', model_path='arm.xml', update_rate=25.0)

        assert hil.port == 'COM3'
        assert hil.update_rate == 25.0
        mock_model_load.assert_called_once_with('arm.xml')
