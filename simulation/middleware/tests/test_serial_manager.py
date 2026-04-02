"""Tests for serial_manager module."""

import pytest
import serial
from unittest.mock import Mock, patch, MagicMock
from middleware.serial_manager import SerialManager
from middleware.protocol import CommandType, TelemetryType


class TestSerialManagerInit:
    """Test SerialManager initialization."""

    def test_init_stores_port_and_baud(self):
        """SerialManager should store port and baud rate."""
        manager = SerialManager(port='COM3', baud=115200)
        assert manager.port == 'COM3'
        assert manager.baud == 115200
        assert manager.timeout == 0.1  # Default timeout

    def test_init_not_connected_initially(self):
        """SerialManager should not be connected on init."""
        manager = SerialManager(port='COM3')
        assert not manager.is_connected()


class TestSerialManagerConnection:
    """Test SerialManager connection handling."""

    @patch('serial.Serial')
    def test_connect_success(self, mock_serial_class):
        """Should connect to serial port successfully."""
        mock_serial = Mock()
        mock_serial_class.return_value = mock_serial

        manager = SerialManager(port='COM3', baud=115200)
        result = manager.connect()

        assert result is True
        assert manager.is_connected()
        mock_serial_class.assert_called_once_with('COM3', 115200, timeout=0.1)

    @patch('serial.Serial')
    def test_connect_failure(self, mock_serial_class):
        """Should handle connection failure gracefully."""
        mock_serial_class.side_effect = serial.SerialException("Port not found")

        manager = SerialManager(port='COM99')
        result = manager.connect()

        assert result is False
        assert not manager.is_connected()

    @patch('serial.Serial')
    def test_disconnect(self, mock_serial_class):
        """Should disconnect and close serial port."""
        mock_serial = Mock()
        mock_serial_class.return_value = mock_serial

        manager = SerialManager(port='COM3')
        manager.connect()
        manager.disconnect()

        assert not manager.is_connected()
        mock_serial.close.assert_called_once()


class TestSerialManagerSendReceive:
    """Test SerialManager send and receive."""

    @patch('serial.Serial')
    def test_send_command(self, mock_serial_class):
        """Should encode and send command packet."""
        mock_serial = Mock()
        mock_serial_class.return_value = mock_serial

        manager = SerialManager(port='COM3')
        manager.connect()

        # Send SET_JOINT_ANGLES command
        result = manager.send_command(CommandType.SET_JOINT_ANGLES,
                                      b'\x00\x00\x00\x00\x00\x00\x00\x00')

        assert result is True
        mock_serial.write.assert_called_once()

    @patch('serial.Serial')
    def test_send_command_not_connected(self, mock_serial_class):
        """Should fail if not connected."""
        manager = SerialManager(port='COM3')
        result = manager.send_command(CommandType.SET_JOINT_ANGLES, b'')

        assert result is False

    @patch('serial.Serial')
    def test_receive_telemetry(self, mock_serial_class):
        """Should receive and decode telemetry packet."""
        mock_serial = Mock()
        # Mock ACK packet: [0xAA][0xF1][0x00][CRC]
        # CRC for [0xF1, 0x00] is 0x01
        # The receive_telemetry reads one byte at a time
        ack_packet = bytes([0xAA, 0xF1, 0x00, 0x01])

        # Track bytes read so in_waiting decreases
        bytes_read = [0]
        def mock_read(n):
            if bytes_read[0] < len(ack_packet):
                result = bytes([ack_packet[bytes_read[0]]])
                bytes_read[0] += 1
                return result
            return b''

        def mock_in_waiting():
            return max(0, len(ack_packet) - bytes_read[0])

        mock_serial.read = mock_read
        type(mock_serial).in_waiting = property(lambda self: mock_in_waiting())
        mock_serial_class.return_value = mock_serial

        manager = SerialManager(port='COM3')
        manager.connect()

        telemetry = manager.receive_telemetry(timeout=0.1)

        assert telemetry is not None
        assert telemetry['type'] == TelemetryType.ACK
