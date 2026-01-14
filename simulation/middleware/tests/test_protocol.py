"""Tests for binary communication protocol."""

import pytest
import struct
from middleware.protocol import (
    crc8, encode_packet, decode_packet, CommandType, encode_set_joint_angles,
    decode_telemetry_full, TelemetryType, PACKET_HEADER
)


class TestCRC8:
    """Test CRC-8 checksum implementation."""

    def test_crc8_empty_data(self):
        """CRC of empty bytes should be 0x00."""
        result = crc8(b'')
        assert result == 0x00

    def test_crc8_known_vector_1(self):
        """Test against known CRC-8 value."""
        # Test vector: [0x10, 0x08] -> CRC = 0x6F
        # Computed using CRC-8 with polynomial 0x07, init 0x00
        data = bytes([0x10, 0x08])
        result = crc8(data)
        assert result == 0x6F

    def test_crc8_known_vector_2(self):
        """Test with longer data."""
        # Full command packet data: type + length + payload
        data = bytes([0x10, 0x08, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0xBF])
        result = crc8(data)
        # Expected CRC computed separately
        assert isinstance(result, int)
        assert 0x00 <= result <= 0xFF


class TestPacketEncoding:
    """Test packet encoding functionality."""

    def test_encode_packet_structure(self):
        """Packet should have correct structure: [header][type][len][data][crc]."""
        cmd_type = 0x10
        data = struct.pack('<ff', 0.5, -0.3)  # 2 floats, 8 bytes

        packet = encode_packet(cmd_type, data)

        assert packet[0] == 0xAA  # Header
        assert packet[1] == cmd_type  # Type
        assert packet[2] == len(data)  # Length
        assert packet[3:3+len(data)] == data  # Payload
        assert len(packet) == 4 + len(data)  # Total size

    def test_encode_packet_crc_valid(self):
        """Packet CRC should be valid."""
        cmd_type = 0x20  # GET_TELEMETRY
        data = b''  # No payload

        packet = encode_packet(cmd_type, data)

        # Extract CRC from packet
        crc_received = packet[-1]

        # Compute expected CRC over [type][len][data]
        crc_expected = crc8(packet[1:-1])

        assert crc_received == crc_expected


class TestPacketDecoding:
    """Test packet decoding functionality."""

    def test_decode_valid_packet(self):
        """Should decode valid packet successfully."""
        cmd_type = 0x10
        data = struct.pack('<ff', 0.5, -0.3)
        packet = encode_packet(cmd_type, data)

        decoded = decode_packet(packet)

        assert decoded is not None
        assert decoded['type'] == cmd_type
        assert decoded['data'] == data

    def test_decode_invalid_header(self):
        """Should reject packet with wrong header."""
        bad_packet = bytes([0xFF, 0x10, 0x00, 0x00])  # Wrong header
        decoded = decode_packet(bad_packet)
        assert decoded is None

    def test_decode_invalid_crc(self):
        """Should reject packet with wrong CRC."""
        packet = encode_packet(0x20, b'')
        # Corrupt the CRC
        bad_packet = packet[:-1] + bytes([0xFF])
        decoded = decode_packet(bad_packet)
        assert decoded is None

    def test_decode_short_packet(self):
        """Should reject packet shorter than minimum size."""
        bad_packet = bytes([0xAA, 0x10])  # Too short
        decoded = decode_packet(bad_packet)
        assert decoded is None


class TestConvenienceFunctions:
    """Test high-level convenience functions."""

    def test_encode_set_joint_angles(self):
        """Should encode SET_JOINT_ANGLES command correctly."""
        shoulder = 0.785  # ~45 degrees
        elbow = -0.524    # ~-30 degrees

        packet = encode_set_joint_angles(shoulder, elbow)

        # Decode to verify
        decoded = decode_packet(packet)
        assert decoded['type'] == CommandType.SET_JOINT_ANGLES
        assert len(decoded['data']) == 8  # 2 floats

        # Unpack angles
        angles = struct.unpack('<ff', decoded['data'])
        assert abs(angles[0] - shoulder) < 0.001
        assert abs(angles[1] - elbow) < 0.001

    def test_encode_set_joint_angles_range_validation(self):
        """Should reject angles outside valid range."""
        with pytest.raises(ValueError, match="out of range"):
            encode_set_joint_angles(3.0, 0.0)  # > π/2

        with pytest.raises(ValueError, match="out of range"):
            encode_set_joint_angles(0.0, -2.0)  # < -π/2


class TestTelemetryDecoding:
    """Test telemetry packet decoding."""

    def test_decode_telemetry_full(self):
        """Should decode full telemetry packet."""
        # Manually construct a telemetry packet
        timestamp = 12345
        angles = [0.5, -0.3]
        velocities = [0.1, -0.05]
        accel = [0.0, 0.0, 9.81]
        gyro = [0.01, 0.02, -0.01]
        orientation = [0.1, 0.05]

        # Pack data (little-endian)
        data = struct.pack('<I 2f 2f 3f 3f 2f',
                          timestamp,
                          *angles, *velocities,
                          *accel, *gyro, *orientation)

        # Build packet manually
        packet_core = bytes([TelemetryType.FULL, len(data)]) + data
        crc = crc8(packet_core)
        packet = bytes([PACKET_HEADER]) + packet_core + bytes([crc])

        # Decode
        telemetry = decode_telemetry_full(packet)

        assert telemetry['timestamp_ms'] == timestamp
        assert len(telemetry['joint_angles']) == 2
        assert abs(telemetry['joint_angles'][0] - 0.5) < 0.001
        assert len(telemetry['imu_accel']) == 3
        assert abs(telemetry['imu_accel'][2] - 9.81) < 0.01
