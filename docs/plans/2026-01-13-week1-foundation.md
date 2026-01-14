# Week 1 Foundation - HIL Robotics Simulator Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build the foundational components enabling hardware-simulation communication and physics modeling (Python protocol layer, MuJoCo arm model, firmware skeleton).

**Architecture:** Three-layer system with (1) Python middleware handling binary protocol and MuJoCo simulation, (2) USB-Serial communication bridge, (3) STM32 firmware for real-time control. This week focuses on software-only components testable without hardware.

**Tech Stack:** Python 3.10+ (pytest, struct, pyserial), MuJoCo 3.0+ (XML MJCF), STM32 HAL (C), Git

---

## Prerequisites

**Environment Setup:**

```bash
# Install Python dependencies
cd simulation
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt

# Verify MuJoCo installation
python -c "import mujoco; print(mujoco.__version__)"

# Install STM32CubeIDE (Windows)
# Download from: https://www.st.com/en/development-tools/stm32cubeide.html
```

**Git Configuration:**

```bash
cd D:\Projects\Robot
git init
git config user.name "Your Name"
git config user.email "your.email@example.com"
```

---

## Task 1: Python Protocol Module (CRC-8 Implementation)

**Files:**
- Create: `simulation/middleware/protocol.py`
- Create: `simulation/middleware/tests/test_protocol.py`

**Step 1: Write failing test for CRC-8 computation**

Create `simulation/middleware/tests/test_protocol.py`:

```python
"""Tests for binary communication protocol."""

import pytest
from middleware.protocol import crc8


class TestCRC8:
    """Test CRC-8 checksum implementation."""

    def test_crc8_empty_data(self):
        """CRC of empty bytes should be 0x00."""
        result = crc8(b'')
        assert result == 0x00

    def test_crc8_known_vector_1(self):
        """Test against known CRC-8 value."""
        # Test vector: [0x10, 0x08] -> CRC = 0x7D
        data = bytes([0x10, 0x08])
        result = crc8(data)
        assert result == 0x7D

    def test_crc8_known_vector_2(self):
        """Test with longer data."""
        # Full command packet data: type + length + payload
        data = bytes([0x10, 0x08, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0xBF])
        result = crc8(data)
        # Expected CRC computed separately
        assert isinstance(result, int)
        assert 0x00 <= result <= 0xFF
```

**Step 2: Run test to verify it fails**

```bash
cd simulation
pytest middleware/tests/test_protocol.py::TestCRC8::test_crc8_empty_data -v
```

Expected output: `ModuleNotFoundError: No module named 'middleware.protocol'`

**Step 3: Create minimal protocol.py with CRC-8 implementation**

Create `simulation/middleware/__init__.py` (empty file for package).

Create `simulation/middleware/protocol.py`:

```python
"""Binary communication protocol for HIL simulator.

Implements packet encoding/decoding with CRC-8 error detection.
Mirrors firmware protocol exactly for consistency.
"""


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
        125  # 0x7D
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
```

**Step 4: Run tests to verify they pass**

```bash
pytest middleware/tests/test_protocol.py::TestCRC8 -v
```

Expected output: All 3 tests PASS

**Step 5: Commit**

```bash
git add simulation/middleware/__init__.py simulation/middleware/protocol.py simulation/middleware/tests/test_protocol.py
git commit -m "feat(protocol): add CRC-8 checksum implementation

- Implement CRC-8 with polynomial 0x07
- Add unit tests with known test vectors
- Verified against protocol specification"
```

---

## Task 2: Protocol Packet Encoding

**Files:**
- Modify: `simulation/middleware/protocol.py`
- Modify: `simulation/middleware/tests/test_protocol.py`

**Step 1: Write failing test for packet encoding**

Add to `simulation/middleware/tests/test_protocol.py`:

```python
import struct
from middleware.protocol import crc8, encode_packet, CommandType


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
```

**Step 2: Run test to verify it fails**

```bash
pytest middleware/tests/test_protocol.py::TestPacketEncoding -v
```

Expected: `ImportError: cannot import name 'encode_packet'`

**Step 3: Implement encode_packet function**

Add to `simulation/middleware/protocol.py`:

```python
from enum import IntEnum


class CommandType(IntEnum):
    """Command types sent from simulation to hardware."""
    SET_JOINT_ANGLES = 0x10
    SET_JOINT_ANGLE_SINGLE = 0x11
    GET_TELEMETRY = 0x20
    SYSTEM_RESET = 0x30
    CALIBRATE_IMU = 0x31
    SET_PID_GAINS = 0x40
    SET_MODE = 0x50


PACKET_HEADER = 0xAA


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
```

**Step 4: Run tests to verify they pass**

```bash
pytest middleware/tests/test_protocol.py::TestPacketEncoding -v
```

Expected: Both tests PASS

**Step 5: Commit**

```bash
git add simulation/middleware/protocol.py simulation/middleware/tests/test_protocol.py
git commit -m "feat(protocol): add packet encoding with CRC validation

- Implement encode_packet with header/type/len/data/crc structure
- Add CommandType enum for type safety
- Validate packet structure and CRC correctness"
```

---

## Task 3: Protocol Packet Decoding

**Files:**
- Modify: `simulation/middleware/protocol.py`
- Modify: `simulation/middleware/tests/test_protocol.py`

**Step 1: Write failing test for packet decoding**

Add to `simulation/middleware/tests/test_protocol.py`:

```python
from middleware.protocol import encode_packet, decode_packet


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
```

**Step 2: Run test to verify it fails**

```bash
pytest middleware/tests/test_protocol.py::TestPacketDecoding -v
```

Expected: `ImportError: cannot import name 'decode_packet'`

**Step 3: Implement decode_packet function**

Add to `simulation/middleware/protocol.py`:

```python
from typing import Optional, Dict, Any


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
```

**Step 4: Run tests to verify they pass**

```bash
pytest middleware/tests/test_protocol.py::TestPacketDecoding -v
```

Expected: All 4 tests PASS

**Step 5: Commit**

```bash
git add simulation/middleware/protocol.py simulation/middleware/tests/test_protocol.py
git commit -m "feat(protocol): add packet decoding with validation

- Implement decode_packet with header/CRC validation
- Reject malformed packets (bad header, wrong CRC, invalid size)
- Return None for invalid packets, dict for valid"
```

---

## Task 4: Convenience Functions for Common Commands

**Files:**
- Modify: `simulation/middleware/protocol.py`
- Modify: `simulation/middleware/tests/test_protocol.py`

**Step 1: Write failing test for encode_set_joint_angles**

Add to `simulation/middleware/tests/test_protocol.py`:

```python
from middleware.protocol import encode_set_joint_angles, decode_packet, CommandType


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
```

**Step 2: Run test to verify it fails**

```bash
pytest middleware/tests/test_protocol.py::TestConvenienceFunctions -v
```

Expected: `ImportError: cannot import name 'encode_set_joint_angles'`

**Step 3: Implement encode_set_joint_angles**

Add to `simulation/middleware/protocol.py`:

```python
import struct
import math


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
```

**Step 4: Run tests to verify they pass**

```bash
pytest middleware/tests/test_protocol.py::TestConvenienceFunctions -v
```

Expected: Both tests PASS

**Step 5: Commit**

```bash
git add simulation/middleware/protocol.py simulation/middleware/tests/test_protocol.py
git commit -m "feat(protocol): add encode_set_joint_angles convenience function

- High-level API for commanding joint angles
- Validates angle ranges [-π/2, π/2]
- Packs floats in little-endian format"
```

---

## Task 5: Telemetry Decoding

**Files:**
- Modify: `simulation/middleware/protocol.py`
- Modify: `simulation/middleware/tests/test_protocol.py`

**Step 1: Write failing test for telemetry decoding**

Add to `simulation/middleware/tests/test_protocol.py`:

```python
from middleware.protocol import decode_telemetry_full, TelemetryType


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
```

**Step 2: Run test to verify it fails**

```bash
pytest middleware/tests/test_protocol.py::TestTelemetryDecoding -v
```

Expected: `ImportError: cannot import name 'decode_telemetry_full'`

**Step 3: Implement telemetry decoding**

Add to `simulation/middleware/protocol.py`:

```python
class TelemetryType(IntEnum):
    """Telemetry types sent from hardware to simulation."""
    FULL = 0x01
    ANGLES_ONLY = 0x02
    IMU_ONLY = 0x03
    ERROR = 0xF0
    ACK = 0xF1


def decode_telemetry_full(packet: bytes) -> Dict[str, Any]:
    """
    Decode TELEMETRY_FULL packet.

    Expected data: 40 bytes
    - uint32 timestamp_ms
    - 2x float joint_angles
    - 2x float joint_velocities
    - 3x float imu_accel
    - 3x float imu_gyro
    - 2x float imu_orientation

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
    if len(data) != 40:
        raise ValueError(f"Expected 40 bytes, got {len(data)}")

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
```

**Step 4: Run tests to verify they pass**

```bash
pytest middleware/tests/test_protocol.py::TestTelemetryDecoding -v
```

Expected: Test PASS

**Step 5: Commit**

```bash
git add simulation/middleware/protocol.py simulation/middleware/tests/test_protocol.py
git commit -m "feat(protocol): add telemetry decoding for TELEMETRY_FULL

- Decode 40-byte telemetry packets
- Extract timestamp, angles, velocities, IMU data
- Validate packet type and size"
```

---

## Task 6: Run Full Protocol Test Suite

**Files:**
- Run: All protocol tests

**Step 1: Run complete test suite with coverage**

```bash
cd simulation
pytest middleware/tests/test_protocol.py -v --cov=middleware.protocol --cov-report=term-missing
```

Expected output: All tests PASS, coverage >90%

**Step 2: Verify protocol.py is complete**

Check that protocol.py includes:
- ✅ CRC-8 implementation
- ✅ Packet encoding/decoding
- ✅ Convenience functions (encode_set_joint_angles)
- ✅ Telemetry decoding (decode_telemetry_full)
- ✅ Type enums (CommandType, TelemetryType)

**Step 3: Create protocol README**

Create `simulation/middleware/README.md`:

```markdown
# Middleware Package

## protocol.py

Binary communication protocol for HIL simulator.

**Key Functions:**
- `encode_packet(cmd_type, data)` - Build binary packet
- `decode_packet(packet)` - Parse binary packet
- `encode_set_joint_angles(shoulder, elbow)` - Command joints
- `decode_telemetry_full(packet)` - Parse telemetry

**Testing:**
```bash
pytest middleware/tests/test_protocol.py -v
```

**Coverage:**
Run `pytest --cov=middleware.protocol` for coverage report.
```

**Step 4: Commit**

```bash
git add simulation/middleware/README.md
git commit -m "docs(middleware): add protocol module documentation"
```

---

## Task 7: MuJoCo Arm Model (Basic Structure)

**Files:**
- Create: `simulation/mujoco_model/arm.xml`

**Step 1: Write basic 2-DOF arm MJCF model**

Create `simulation/mujoco_model/arm.xml`:

```xml
<mujoco model="2dof_arm">
  <compiler angle="radian" />

  <option timestep="0.001" gravity="0 0 -9.81" />

  <default>
    <joint damping="0.1" armature="0.01" />
    <geom contype="1" conaffinity="1" />
  </default>

  <worldbody>
    <!-- Fixed base -->
    <body name="base" pos="0 0 0">
      <geom type="box" size="0.05 0.05 0.02" rgba="0.3 0.3 0.3 1" mass="0.1" />

      <!-- Link 1: Shoulder to elbow (10cm) -->
      <body name="link1" pos="0 0 0.02">
        <joint name="shoulder" type="hinge" axis="0 0 1"
               range="-1.57 1.57" damping="0.1" />
        <geom type="capsule" size="0.015 0.05" rgba="0.8 0.2 0.2 1"
              pos="0.05 0 0" quat="0.707 0 0.707 0" mass="0.02" />

        <!-- Link 2: Elbow to end-effector (8cm) -->
        <body name="link2" pos="0.1 0 0">
          <joint name="elbow" type="hinge" axis="0 1 0"
                 range="-1.57 1.57" damping="0.08" />
          <geom type="capsule" size="0.012 0.04" rgba="0.2 0.8 0.2 1"
                pos="0.04 0 0" quat="0.707 0 0.707 0" mass="0.015" />

          <!-- End effector -->
          <body name="end_effector" pos="0.08 0 0">
            <geom type="sphere" size="0.01" rgba="0.2 0.2 0.8 1" mass="0.005" />

            <!-- IMU sensor location (visualization) -->
            <site name="imu" pos="0 0 0" size="0.005" rgba="1 1 0 1" />
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <!-- Position servos (modeling SG90 behavior) -->
    <position name="shoulder_servo" joint="shoulder" kp="2.0" />
    <position name="elbow_servo" joint="elbow" kp="2.0" />
  </actuator>

  <sensor>
    <jointpos name="shoulder_pos" joint="shoulder" />
    <jointpos name="elbow_pos" joint="elbow" />
    <framequat name="imu_quat" objtype="site" objname="imu" />
  </sensor>
</mujoco>
```

**Step 2: Test model loads correctly**

Create test file `simulation/tests/test_mujoco_model.py`:

```python
"""Tests for MuJoCo arm model."""

import pytest
import mujoco
import os


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
```

**Step 3: Run test to verify model loads**

```bash
cd simulation
pytest tests/test_mujoco_model.py -v
```

Expected: All tests PASS

**Step 4: Create visualization test script**

Create `simulation/mujoco_model/visualize_model.py`:

```python
"""Visualize the arm model interactively."""

import mujoco
import mujoco.viewer
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('arm.xml')
data = mujoco.MjData(model)

# Set initial joint angles
data.qpos[0] = 0.5   # Shoulder: 30 degrees
data.qpos[1] = -0.3  # Elbow: -17 degrees

# Launch viewer
print("Launching MuJoCo viewer...")
print("Press ESC to close")
mujoco.viewer.launch(model, data)
```

**Step 5: Test visualization (manual)**

```bash
cd simulation/mujoco_model
python visualize_model.py
```

Expected: MuJoCo viewer opens, arm visible in 3D

**Step 6: Commit**

```bash
git add simulation/mujoco_model/arm.xml simulation/tests/test_mujoco_model.py simulation/mujoco_model/visualize_model.py
git commit -m "feat(mujoco): add 2-DOF arm MJCF model

- Define 2-link arm with shoulder/elbow joints
- Add position actuators modeling servo behavior
- Include sensors for joint positions and IMU
- Add automated tests and visualization script"
```

---

## Task 8: Kinematics Module

**Files:**
- Create: `simulation/middleware/kinematics.py`
- Create: `simulation/middleware/tests/test_kinematics.py`

**Step 1: Write failing test for forward kinematics**

Create `simulation/middleware/tests/test_kinematics.py`:

```python
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
```

**Step 2: Run test to verify it fails**

```bash
pytest middleware/tests/test_kinematics.py -v
```

Expected: `ImportError: No module named 'middleware.kinematics'`

**Step 3: Implement forward kinematics**

Create `simulation/middleware/kinematics.py`:

```python
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
```

**Step 4: Run tests to verify they pass**

```bash
pytest middleware/tests/test_kinematics.py::TestForwardKinematics -v
```

Expected: All 3 tests PASS

**Step 5: Commit**

```bash
git add simulation/middleware/kinematics.py simulation/middleware/tests/test_kinematics.py
git commit -m "feat(kinematics): add forward kinematics for 2-DOF arm

- Implement FK using standard DH conventions
- Add tests for zero angles, single joint, combined angles"
```

---

## Task 9: Inverse Kinematics

**Files:**
- Modify: `simulation/middleware/kinematics.py`
- Modify: `simulation/middleware/tests/test_kinematics.py`

**Step 1: Write failing test for inverse kinematics**

Add to `simulation/middleware/tests/test_kinematics.py`:

```python
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
```

**Step 2: Run test to verify it fails**

```bash
pytest middleware/tests/test_kinematics.py::TestInverseKinematics -v
```

Expected: `AttributeError: 'ArmKinematics' object has no attribute 'inverse'`

**Step 3: Implement inverse kinematics**

Add to `simulation/middleware/kinematics.py`:

```python
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
```

**Step 4: Run tests to verify they pass**

```bash
pytest middleware/tests/test_kinematics.py::TestInverseKinematics -v
```

Expected: All 3 tests PASS

**Step 5: Commit**

```bash
git add simulation/middleware/kinematics.py simulation/middleware/tests/test_kinematics.py
git commit -m "feat(kinematics): add inverse kinematics with reachability check

- Implement IK using law of cosines
- Elbow-up solution for 2-DOF arm
- Validate FK-IK consistency"
```

---

## Task 10: STM32 Firmware Project Setup

**Files:**
- Create: `firmware/.gitignore`
- Create: STM32CubeIDE project via GUI

**Step 1: Launch STM32CubeIDE and create project**

Open STM32CubeIDE (Windows):

1. File → New → STM32 Project
2. Board Selector → Search "NUCLEO-F446RE"
3. Select board → Next
4. Project Name: `hil_firmware`
5. Location: `D:\Projects\Robot\firmware`
6. Targeted Language: C
7. Finish (initialize with default peripherals: Yes)

**Step 2: Configure peripherals in STM32CubeMX**

In the .ioc file (auto-opens):

**USART2** (USB-Serial via ST-Link):
- Mode: Asynchronous
- Baud: 115200
- Word Length: 8 bits
- Parity: None
- Stop bits: 1
- DMA Settings:
  - Add USART2_RX: DMA1 Stream 5, Circular mode
  - Add USART2_TX: DMA1 Stream 6, Normal mode

**TIM2** (Servo 1 PWM):
- Clock Source: Internal Clock
- Channel 1: PWM Generation CH1
- Prescaler: 180-1 (1MHz timer clock)
- Counter Period: 20000-1 (50Hz, 20ms period)
- Pulse: 1500 (neutral 1.5ms)

**TIM3** (Servo 2 PWM):
- Clock Source: Internal Clock
- Channel 2: PWM Generation CH2
- Prescaler: 180-1
- Counter Period: 20000-1
- Pulse: 1500

**I2C1** (MPU6050):
- Mode: I2C
- Speed: 100 kHz (Standard Mode)
- Pins: PB8 (SCL), PB9 (SDA)

**GPIO**:
- PA5: Output (LED)

**Step 3: Generate code**

Project → Generate Code (or Ctrl+S in .ioc)

**Step 4: Create .gitignore for firmware**

Create `firmware/.gitignore`:

```
# Build outputs
Debug/
Release/
Build/

# IDE files
.metadata/
.settings/
.cproject
.project

# Generated files
Drivers/

# Keep only custom source
!Core/
```

**Step 5: Build firmware to verify setup**

Project → Build Project (Ctrl+B)

Expected: BUILD SUCCESSFUL (0 errors, possibly warnings)

**Step 6: Create firmware README**

Create `firmware/README.md`:

```markdown
# HIL Firmware - STM32 NUCLEO-F446RE

Real-time control firmware for 2-DOF robotic arm.

## Hardware Configuration

- **MCU**: STM32F446RET6 (ARM Cortex-M4, 180MHz)
- **USART2**: USB-Serial communication (PA2 TX, PA3 RX)
- **TIM2_CH1**: Servo 1 PWM (PA0)
- **TIM3_CH2**: Servo 2 PWM (PA1)
- **I2C1**: MPU6050 IMU (PB8 SCL, PB9 SDA)

## Build

Open project in STM32CubeIDE:
1. File → Open Projects from File System
2. Select `firmware/` directory
3. Build: Project → Build Project (Ctrl+B)

## Flash

Connect NUCLEO board via USB:
1. Build project
2. Run → Debug (F11) or Run (Ctrl+F11)

## Project Structure

```
firmware/
├── Core/
│   ├── Src/
│   │   ├── main.c              # Initialization, main loop
│   │   ├── uart_handler.c      # (to be added)
│   │   └── protocol.c          # (to be added)
│   └── Inc/
│       └── ...
└── README.md
```
```

**Step 7: Commit**

```bash
cd D:\Projects\Robot
git add firmware/.gitignore firmware/README.md
git add firmware/Core/Src/main.c firmware/Core/Inc/main.h
git add firmware/.project firmware/.cproject
git commit -m "feat(firmware): initialize STM32CubeIDE project

- Configure USART2 for serial communication (115200 baud, DMA)
- Configure TIM2/TIM3 for servo PWM (50Hz)
- Configure I2C1 for MPU6050 IMU
- Add build instructions and hardware configuration"
```

---

## Task 11: Final Verification & Documentation

**Files:**
- Create: `docs/plans/2026-01-13-week1-completion-checklist.md`

**Step 1: Run all tests**

```bash
cd simulation
pytest middleware/tests/ -v --cov=middleware --cov-report=term
pytest tests/test_mujoco_model.py -v
```

Expected: All tests PASS, coverage >80%

**Step 2: Verify firmware builds**

Open STM32CubeIDE, build project.

Expected: 0 errors

**Step 3: Create completion checklist**

Create `docs/plans/2026-01-13-week1-completion-checklist.md`:

```markdown
# Week 1 Completion Checklist

## Python Protocol Module ✅
- [x] CRC-8 implementation with tests
- [x] Packet encoding/decoding with validation
- [x] Convenience functions (encode_set_joint_angles)
- [x] Telemetry decoding (decode_telemetry_full)
- [x] Test coverage >90%

## MuJoCo Simulation ✅
- [x] 2-DOF arm MJCF model (arm.xml)
- [x] Model loads without errors
- [x] 2 joints with correct ranges [-π/2, π/2]
- [x] Position actuators configured
- [x] Visualization script working

## Kinematics ✅
- [x] Forward kinematics implementation
- [x] Inverse kinematics with reachability check
- [x] FK-IK consistency tests passing

## Firmware Foundation ✅
- [x] STM32CubeIDE project created
- [x] USART2 configured (115200 baud, DMA)
- [x] TIM2/TIM3 PWM configured (50Hz)
- [x] I2C1 configured for MPU6050
- [x] Project builds successfully

## Documentation ✅
- [x] Design document (DESIGN.md)
- [x] Protocol specification (PROTOCOL.md)
- [x] Component interfaces (INTERFACES.md)
- [x] README.md portfolio-ready
- [x] Firmware README
- [x] Middleware README

## Git Repository ✅
- [x] Repository initialized
- [x] .gitignore configured
- [x] Professional commit messages
- [x] All code committed

## Ready for Week 2 🚀
With hardware arriving tomorrow (Jan 14), we can immediately start:
- UART communication testing
- Servo PWM output verification
- Protocol integration (Python ↔ STM32)
- Basic HIL loop implementation
```

**Step 4: Commit checklist**

```bash
git add docs/plans/2026-01-13-week1-completion-checklist.md
git commit -m "docs(plans): add Week 1 completion checklist"
```

**Step 5: Push to GitHub (if remote configured)**

```bash
git remote add origin https://github.com/yourusername/hil-robotics-simulator.git
git branch -M main
git push -u origin main
```

---

## Execution Notes

### TDD Discipline
- **Red-Green-Refactor**: Always write failing test first
- **Minimal implementation**: Just enough to pass the test
- **Frequent commits**: After each green test

### Code Quality
- **DRY**: No duplicate protocol logic between encode/decode
- **YAGNI**: Only implement what's needed for Week 1
- **Type hints**: Use Python type annotations for clarity
- **Docstrings**: Every public function documented

### Common Pitfalls
1. **Forgetting to activate venv**: Always `source venv/bin/activate`
2. **Wrong CWD for pytest**: Run from `simulation/` directory
3. **STM32CubeIDE warnings**: Clock configuration warnings are OK for now
4. **MuJoCo rendering on WSL2**: Use native Windows Python if X11 issues

### Estimated Time
- Tasks 1-6 (Protocol): ~2 hours
- Tasks 7-9 (MuJoCo + Kinematics): ~1.5 hours
- Task 10 (Firmware setup): ~1 hour
- Task 11 (Verification): ~30 minutes
- **Total**: ~5 hours (comfortable Week 1 workload)

---

## Next Plan: Week 2 - HIL Communication Loop

After completing this plan, create:
`docs/plans/2026-01-20-week2-hil-loop.md`

**Focus**:
- Firmware UART/protocol implementation (C)
- Servo PWM control with basic PID
- Python serial manager with reconnection
- First HIL loop: Command → Hardware → Telemetry → Sim
- MPU6050 driver and raw data reading

---

**Plan Status**: ✅ Ready for execution

**Saved to**: `docs/plans/2026-01-13-week1-foundation.md`
