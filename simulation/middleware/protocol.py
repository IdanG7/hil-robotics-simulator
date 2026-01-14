"""Binary communication protocol for HIL simulator.

Implements packet encoding/decoding with CRC-8 error detection.
Mirrors firmware protocol exactly for consistency.
"""

import struct
import math
from enum import IntEnum
from typing import Optional, Dict, Any


class CommandType(IntEnum):
    """Command types sent from simulation to hardware."""
    SET_JOINT_ANGLES = 0x10
    SET_JOINT_ANGLE_SINGLE = 0x11
    GET_TELEMETRY = 0x20
    SYSTEM_RESET = 0x30
    CALIBRATE_IMU = 0x31
    SET_PID_GAINS = 0x40
    SET_MODE = 0x50


class TelemetryType(IntEnum):
    """Telemetry types sent from hardware to simulation."""
    FULL = 0x01
    ANGLES_ONLY = 0x02
    IMU_ONLY = 0x03
    ERROR = 0xF0
    ACK = 0xF1


PACKET_HEADER = 0xAA


def crc8(data: bytes) -> int:
    """
    Compute CRC-8 checksum using polynomial 0x07.

    Algorithm: CRC-8 with polynomial x^8 + x^2 + x + 1 (0x07)
    Initial value: 0x00

    Args:
        data: Input bytes to checksum

    Returns:
        CRC-8 value (0x00-0xFF)

    Example:
        >>> crc8(b'\\x10\\x08')
        111  # 0x6F
    """
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def encode_packet(cmd_type: int, data: bytes) -> bytes:
    """
    Encode command into binary packet.

    Packet structure: [HEADER][TYPE][LENGTH][DATA][CRC8]

    Args:
        cmd_type: Command type from CommandType enum
        data: Payload bytes (struct-packed)

    Returns:
        Complete packet with header, length, CRC

    Example:
        >>> data = struct.pack('<ff', 0.5, -0.3)
        >>> packet = encode_packet(0x10, data)
        >>> len(packet)
        12  # 1 header + 1 type + 1 len + 8 data + 1 crc
    """
    length = len(data)
    if length > 64:
        raise ValueError(f"Data length {length} exceeds maximum 64 bytes")

    # Build packet: [type][length][data]
    packet_core = bytes([cmd_type, length]) + data

    # Compute CRC over [type][length][data]
    crc = crc8(packet_core)

    # Final packet: [header][type][length][data][crc]
    packet = bytes([PACKET_HEADER]) + packet_core + bytes([crc])

    return packet


def decode_packet(packet: bytes) -> Optional[Dict[str, Any]]:
    """
    Decode binary packet.

    Args:
        packet: Complete packet bytes

    Returns:
        Dict with 'type', 'data' keys if valid, None if invalid

    Example:
        >>> packet = encode_packet(0x10, b'\\x00\\x01')
        >>> decoded = decode_packet(packet)
        >>> decoded['type']
        16  # 0x10
    """
    # Minimum packet: header + type + len + crc = 4 bytes
    if len(packet) < 4:
        return None

    # Validate header
    if packet[0] != PACKET_HEADER:
        return None

    pkt_type = packet[1]
    length = packet[2]

    # Validate packet size matches length field
    expected_size = 4 + length  # header + type + len + data + crc
    if len(packet) != expected_size:
        return None

    # Extract data
    if length > 0:
        data = packet[3:3+length]
    else:
        data = b''

    # Extract and validate CRC
    crc_received = packet[-1]
    crc_expected = crc8(packet[1:-1])  # [type][len][data]

    if crc_received != crc_expected:
        return None

    return {
        'type': pkt_type,
        'data': data,
    }


def encode_set_joint_angles(shoulder: float, elbow: float) -> bytes:
    """
    Encode SET_JOINT_ANGLES command.

    Args:
        shoulder: Shoulder joint angle in radians [-π/2, π/2]
        elbow: Elbow joint angle in radians [-π/2, π/2]

    Returns:
        Complete binary packet

    Raises:
        ValueError: If angles out of valid range

    Example:
        >>> packet = encode_set_joint_angles(0.785, -0.524)
        >>> len(packet)
        12  # Full packet with header/crc
    """
    # Validate ranges
    max_angle = math.pi / 2
    if not (-max_angle <= shoulder <= max_angle):
        raise ValueError(f"Shoulder angle {shoulder} out of range [-π/2, π/2]")
    if not (-max_angle <= elbow <= max_angle):
        raise ValueError(f"Elbow angle {elbow} out of range [-π/2, π/2]")

    # Pack as little-endian floats
    data = struct.pack('<ff', shoulder, elbow)

    return encode_packet(CommandType.SET_JOINT_ANGLES, data)


def decode_telemetry_full(packet: bytes) -> Dict[str, Any]:
    """
    Decode TELEMETRY_FULL packet.

    Expected data: 52 bytes
    - uint32 timestamp_ms (4 bytes)
    - 2x float joint_angles (8 bytes)
    - 2x float joint_velocities (8 bytes)
    - 3x float imu_accel (12 bytes)
    - 3x float imu_gyro (12 bytes)
    - 2x float imu_orientation (8 bytes)

    Args:
        packet: Complete telemetry packet

    Returns:
        Dict with keys: timestamp_ms, joint_angles, joint_velocities,
                       imu_accel, imu_gyro, imu_orientation

    Raises:
        ValueError: If packet invalid or wrong type
    """
    decoded = decode_packet(packet)
    if decoded is None:
        raise ValueError("Invalid packet (CRC or structure error)")

    if decoded['type'] != TelemetryType.FULL:
        raise ValueError(f"Expected TELEMETRY_FULL (0x01), got 0x{decoded['type']:02X}")

    data = decoded['data']
    if len(data) != 52:
        raise ValueError(f"Expected 52 bytes, got {len(data)}")

    # Unpack all fields (little-endian)
    values = struct.unpack('<I 2f 2f 3f 3f 2f', data)

    return {
        'timestamp_ms': values[0],
        'joint_angles': list(values[1:3]),
        'joint_velocities': list(values[3:5]),
        'imu_accel': list(values[5:8]),
        'imu_gyro': list(values[8:11]),
        'imu_orientation': list(values[11:13]),
    }
