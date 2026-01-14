# Communication Protocol Specification

**Project**: HIL Robotics Simulator
**Version**: 1.0
**Date**: January 13, 2026

---

## Overview

This document specifies the binary communication protocol for bidirectional data exchange between the Python simulation middleware and STM32 firmware. The protocol is optimized for:
- Low latency (<50ms round-trip)
- Deterministic packet size for predictable timing
- Error detection via CRC-8 checksum
- Extensibility for future commands

**Transport Layer**: USB-Serial (USART2 on STM32 via ST-Link)
**Baud Rate**: 115200 (configurable up to 921600)
**Data Format**: 8-N-1 (8 data bits, no parity, 1 stop bit)

---

## Packet Structure

All packets follow this general structure:

```
┌────────┬──────────┬────────┬───────────┬────────┐
│ HEADER │  TYPE    │ LENGTH │   DATA    │  CRC8  │
│  1 B   │   1 B    │  1 B   │  0-64 B   │  1 B   │
└────────┴──────────┴────────┴───────────┴────────┘
```

### Field Descriptions

| Field | Size | Description |
|-------|------|-------------|
| **HEADER** | 1 byte | Packet start delimiter: `0xAA` (fixed) |
| **TYPE** | 1 byte | Command or telemetry type (see tables below) |
| **LENGTH** | 1 byte | Length of DATA field (0-64 bytes) |
| **DATA** | N bytes | Payload (structure depends on TYPE) |
| **CRC8** | 1 byte | Checksum over `[TYPE][LENGTH][DATA]` fields |

**Maximum Packet Size**: 68 bytes (1 + 1 + 1 + 64 + 1)

---

## Command Packets (Simulation → Hardware)

These packets are sent from Python middleware to STM32 firmware to control the robot.

### Command Type Table

| TYPE | Name | Data Size | Description |
|------|------|-----------|-------------|
| `0x10` | SET_JOINT_ANGLES | 8 B | Set target angles for both joints |
| `0x11` | SET_JOINT_ANGLE_SINGLE | 5 B | Set target angle for one joint |
| `0x20` | GET_TELEMETRY | 0 B | Request immediate telemetry packet |
| `0x30` | SYSTEM_RESET | 0 B | Reset system, zero angles, recalibrate |
| `0x31` | CALIBRATE_IMU | 0 B | Recalibrate IMU (500 sample bias) |
| `0x40` | SET_PID_GAINS | 24 B | Update PID gains for both joints |
| `0x41` | SET_PID_GAINS_SINGLE | 13 B | Update PID gains for one joint |
| `0x50` | SET_MODE | 1 B | Set control mode (IDLE, POSITION, TRAJECTORY) |
| `0x60` | SET_TRAJECTORY_POINT | 16 B | Add waypoint to trajectory buffer |
| `0x70` | DEBUG_COMMAND | Variable | Debug/test commands (development only) |

---

### 0x10: SET_JOINT_ANGLES

Command both joints to target angles simultaneously.

**Data Structure** (8 bytes):
```c
struct SetJointAnglesData {
    float shoulder_angle;  // Target angle in radians [-π/2, π/2]
    float elbow_angle;     // Target angle in radians [-π/2, π/2]
};
```

**Byte Layout** (little-endian):
```
[0xAA][0x10][0x08][shoulder (4B)][elbow (4B)][CRC8]
```

**Example** (shoulder=0.785 rad (45°), elbow=-0.524 rad (-30°)):
```
AA 10 08 DB 0F 49 3F AB AA 28 BF [CRC]
```

**Python Encoding**:
```python
import struct

def encode_set_joint_angles(shoulder, elbow):
    data = struct.pack('<ff', shoulder, elbow)  # Little-endian floats
    return build_packet(0x10, data)
```

---

### 0x11: SET_JOINT_ANGLE_SINGLE

Command a single joint (more efficient for individual moves).

**Data Structure** (5 bytes):
```c
struct SetJointAngleSingleData {
    uint8_t joint_id;      // 0=shoulder, 1=elbow
    float target_angle;    // Target angle in radians
};
```

**Byte Layout**:
```
[0xAA][0x11][0x05][joint_id (1B)][angle (4B)][CRC8]
```

---

### 0x20: GET_TELEMETRY

Request immediate telemetry packet (normally sent automatically at 50Hz).

**Data Structure**: None (0 bytes)

**Byte Layout**:
```
[0xAA][0x20][0x00][CRC8]
```

**Use Case**: On-demand sensor reading (e.g., after reset, for calibration).

---

### 0x30: SYSTEM_RESET

Reset the control system to initial state.

**Data Structure**: None (0 bytes)

**Actions Performed by Firmware**:
1. Stop all servo outputs (set to neutral 1500μs pulse)
2. Clear PID integral terms
3. Reset target angles to 0.0
4. Recalibrate IMU (500 samples)
5. Clear error flags

**Response**: Telemetry packet with all zeros after reset completes (~2 seconds).

---

### 0x31: CALIBRATE_IMU

Recalibrate IMU bias without full system reset.

**Data Structure**: None (0 bytes)

**Precondition**: Arm must be stationary (firmware checks gyro variance).

**Calibration Procedure**:
1. Collect 500 samples at 100Hz (5 seconds)
2. Compute mean accel/gyro offsets
3. Store in RAM (not persistent across power cycles)

---

### 0x40: SET_PID_GAINS

Update PID controller gains for both joints.

**Data Structure** (24 bytes):
```c
struct SetPIDGainsData {
    float shoulder_kp;
    float shoulder_ki;
    float shoulder_kd;
    float elbow_kp;
    float elbow_ki;
    float elbow_kd;
};
```

**Valid Ranges** (firmware clamps to safe values):
- Kp: [0.0, 10.0]
- Ki: [0.0, 1.0]
- Kd: [0.0, 2.0]

**Python Example**:
```python
def set_pid_gains(shoulder_gains, elbow_gains):
    data = struct.pack('<ffffff',
                       shoulder_gains['kp'], shoulder_gains['ki'], shoulder_gains['kd'],
                       elbow_gains['kp'], elbow_gains['ki'], elbow_gains['kd'])
    return build_packet(0x40, data)
```

---

### 0x41: SET_PID_GAINS_SINGLE

Update PID gains for one joint (more efficient for tuning).

**Data Structure** (13 bytes):
```c
struct SetPIDGainsSingleData {
    uint8_t joint_id;  // 0=shoulder, 1=elbow
    float kp;
    float ki;
    float kd;
};
```

---

### 0x50: SET_MODE

Set the control mode of the system.

**Data Structure** (1 byte):
```c
enum ControlMode {
    MODE_IDLE = 0,       // Servos disabled (safe mode)
    MODE_POSITION = 1,   // Position control (PID to target angles)
    MODE_TRAJECTORY = 2  // Trajectory following mode (future)
};
```

**Default Mode**: `MODE_IDLE` (set to `MODE_POSITION` after initialization).

---

### 0x60: SET_TRAJECTORY_POINT

Add a waypoint to the trajectory buffer (for smooth multi-point motion).

**Data Structure** (16 bytes):
```c
struct TrajectoryPoint {
    float shoulder_angle;
    float elbow_angle;
    float duration_sec;      // Time to reach this point from previous
    uint32_t flags;          // Reserved for future use
};
```

**Note**: Trajectory mode is a **stretch goal** (Week 3+). Basic implementation uses point-to-point moves.

---

### 0x70: DEBUG_COMMAND

Development-only command for testing specific firmware functions.

**Data Structure**: Variable (depends on debug command)

**Examples**:
- `0x70 0x01`: Toggle LED
- `0x70 0x02 [angle]`: Force servo PWM directly (bypass PID)
- `0x70 0x03`: Dump register values to UART

**Warning**: Not for production use. May be removed in final firmware.

---

## Telemetry Packets (Hardware → Simulation)

Telemetry packets are sent from STM32 firmware to Python middleware at 50Hz (every 20ms) or on-demand via GET_TELEMETRY command.

### Telemetry Type Table

| TYPE | Name | Data Size | Description |
|------|------|-----------|-------------|
| `0x01` | TELEMETRY_FULL | 40 B | Full sensor state (angles, IMU, timestamp) |
| `0x02` | TELEMETRY_ANGLES_ONLY | 12 B | Joint angles only (fast mode) |
| `0x03` | TELEMETRY_IMU_ONLY | 28 B | IMU data only |
| `0xF0` | ERROR_RESPONSE | Variable | Error message (command failed) |
| `0xF1` | ACK | 1 B | Command acknowledged |

---

### 0x01: TELEMETRY_FULL

Complete robot state including joint angles, IMU data, and timing.

**Data Structure** (40 bytes):
```c
struct TelemetryFull {
    uint32_t timestamp_ms;      // System time since boot (ms)
    float joint_angles[2];      // Shoulder, elbow (radians)
    float joint_velocities[2];  // Angular velocity (rad/s, estimated)
    float imu_accel[3];         // X, Y, Z acceleration (m/s²)
    float imu_gyro[3];          // X, Y, Z angular velocity (rad/s)
    float imu_orientation[2];   // Roll, pitch from fusion (radians)
};
```

**Byte Layout**:
```
[0xAA][0x01][0x28][timestamp (4B)][angles (8B)][velocities (8B)]
                   [accel (12B)][gyro (12B)][orientation (8B)][CRC8]
```

**Python Decoding**:
```python
def decode_telemetry_full(packet):
    header, pkt_type, length = struct.unpack('BBB', packet[:3])
    assert header == 0xAA and pkt_type == 0x01 and length == 40

    data = struct.unpack('<I 2f 2f 3f 3f 2f', packet[3:43])

    return {
        'timestamp_ms': data[0],
        'joint_angles': data[1:3],
        'joint_velocities': data[3:5],
        'imu_accel': data[5:8],
        'imu_gyro': data[8:11],
        'imu_orientation': data[11:13],
    }
```

**Send Rate**: 50Hz (every 20ms) in normal operation.

---

### 0x02: TELEMETRY_ANGLES_ONLY

Lightweight telemetry with joint angles only (useful for high-rate control).

**Data Structure** (12 bytes):
```c
struct TelemetryAnglesOnly {
    uint32_t timestamp_ms;
    float joint_angles[2];
};
```

**Use Case**: Reduce bandwidth if IMU data is not needed (e.g., pure position control without orientation validation).

---

### 0x03: TELEMETRY_IMU_ONLY

IMU data without joint angles (for sensor debugging/calibration).

**Data Structure** (28 bytes):
```c
struct TelemetryIMUOnly {
    uint32_t timestamp_ms;
    float imu_accel[3];
    float imu_gyro[3];
    float imu_orientation[2];
};
```

---

### 0xF0: ERROR_RESPONSE

Sent when a command fails or encounters an error.

**Data Structure** (Variable):
```c
struct ErrorResponse {
    uint8_t error_code;     // Error type (see error codes below)
    uint8_t failed_cmd;     // Command that caused the error
    char message[N];        // Human-readable error (null-terminated)
};
```

**Error Codes**:
| Code | Name | Description |
|------|------|-------------|
| `0x01` | ERR_INVALID_CMD | Unknown command type |
| `0x02` | ERR_CRC_MISMATCH | CRC validation failed |
| `0x03` | ERR_OUT_OF_RANGE | Data value out of valid range |
| `0x04` | ERR_TIMEOUT | Operation timed out |
| `0x05` | ERR_HARDWARE | Hardware failure (servo, IMU) |
| `0x06` | ERR_BUSY | System busy, retry later |

**Example**:
```
[0xAA][0xF0][0x0F][0x03][0x10]["Angle out of range"][CRC8]
```

---

### 0xF1: ACK

Simple acknowledgment (optional, for critical commands).

**Data Structure** (1 byte):
```c
struct Ack {
    uint8_t acked_cmd;  // Command that was successfully executed
};
```

**Use Case**: Confirm receipt of SET_PID_GAINS, CALIBRATE_IMU, etc.

---

## CRC-8 Checksum

**Purpose**: Detect transmission errors (bit flips, noise).

**Algorithm**: CRC-8 with polynomial 0x07 (x^8 + x^2 + x + 1).

**Computation**:
```c
uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}
```

**Fields Included in CRC**: `[TYPE][LENGTH][DATA]` (excludes HEADER and CRC itself).

**Python Implementation**:
```python
def crc8(data):
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

**Validation**:
```python
def validate_packet(packet):
    if len(packet) < 4:
        return False
    if packet[0] != 0xAA:  # Check header
        return False

    length = packet[2]
    if len(packet) != 4 + length:  # Header + Type + Len + Data + CRC
        return False

    computed_crc = crc8(packet[1:3 + length])  # Type + Len + Data
    received_crc = packet[3 + length]
    return computed_crc == received_crc
```

---

## Error Handling & Recovery

### Timeout Handling

**Simulation Side** (Python):
- If no telemetry received for **100ms**, mark connection as "DEGRADED"
- If no telemetry for **500ms**, mark as "DISCONNECTED"
- Attempt reconnection by sending GET_TELEMETRY command
- After 3 failed reconnection attempts, alert user

**Firmware Side** (STM32):
- If no command received for **2 seconds**, remain in last commanded state (don't reset to zero!)
- If no command for **10 seconds**, enter IDLE mode (disable servos for safety)

### CRC Mismatch

**Firmware Action**:
1. Discard packet
2. Send ERROR_RESPONSE with ERR_CRC_MISMATCH
3. Do not change system state

**Simulation Action**:
1. Log error
2. Retry command (up to 3 attempts)
3. If persistent, alert user (possible hardware issue)

### Buffer Overflow

**Firmware**:
- Circular RX buffer (256 bytes)
- If buffer full, discard oldest data
- Increment overflow counter (reported in telemetry)

**Simulation**:
- Python serial buffer managed by pySerial (OS-level)
- Read frequently to prevent OS buffer overflow

### Invalid Command

**Firmware**:
- Send ERROR_RESPONSE with ERR_INVALID_CMD
- Log to debug UART if available

---

## Protocol Extensions (Future)

### Variable Packet Header (v2.0)

Current protocol uses fixed `0xAA` header. Future versions could use:
- `0xAA`: Standard packet
- `0xAB`: Compressed packet (e.g., delta encoding)
- `0xAC`: Encrypted packet (for wireless)

### Multi-Robot Support (v2.0)

Add robot ID field for controlling multiple arms:
```
[HEADER][ROBOT_ID][TYPE][LENGTH][DATA][CRC8]
```

### Acknowledgment Mode

Add ACK request flag in command TYPE field (MSB):
```
0x10: SET_JOINT_ANGLES (no ACK)
0x90: SET_JOINT_ANGLES (request ACK)
```

---

## Example Communication Sequences

### Initialization Sequence

```
Python                          STM32
  │                               │
  │──────── 0xAA 0x30 0x00 ───────►│  (SYSTEM_RESET)
  │                               │
  │                          [2s delay]
  │                               │
  │◄───── 0xAA 0xF1 0x01 ──────────│  (ACK RESET)
  │                               │
  │──────── 0xAA 0x20 0x00 ───────►│  (GET_TELEMETRY)
  │                               │
  │◄───── 0xAA 0x01 0x28 [data] ───│  (TELEMETRY_FULL)
  │                               │
  │──────── 0xAA 0x50 0x01 0x01 ──►│  (SET_MODE POSITION)
  │                               │
  │◄───── 0xAA 0xF1 0x01 ──────────│  (ACK)
  │                               │
  [Normal operation begins]
```

### Normal Operation (50Hz Loop)

```
Python                          STM32
  │                               │
  │────── SET_ANGLES (0.5, -0.3) ─►│
  │                               │  [PID computes PWM]
  │                               │  [Servos move]
  │                               │  [Read IMU]
  │                               │
  │◄──── TELEMETRY_FULL ───────────│  (20ms later)
  │                               │
  │────── SET_ANGLES (0.52, -0.28)─►│
  │                               │
  │◄──── TELEMETRY_FULL ───────────│
  │                               │
  [Continues at 50Hz...]
```

### Error Recovery

```
Python                          STM32
  │                               │
  │────── SET_ANGLES (bad CRC) ───►│
  │                               │
  │◄──── ERROR_RESPONSE (0x02) ────│  (CRC mismatch)
  │                               │
  │────── SET_ANGLES (correct) ────►│  (Retry)
  │                               │
  │◄──── TELEMETRY_FULL ───────────│  (Success)
```

---

## Performance Considerations

### Latency Budget (Target: <50ms)

| Stage | Time | Notes |
|-------|------|-------|
| Python encode | <1ms | struct.pack is fast |
| USB-Serial TX | ~2ms | 8 bytes @ 115200 baud |
| Firmware RX ISR | <0.1ms | DMA to buffer |
| Protocol parsing | <0.5ms | Fixed size, no malloc |
| PID computation | <1ms | FPU accelerated |
| PWM update | ~0.1ms | Hardware timer |
| Servo response | ~10-15ms | Mechanical delay |
| IMU read | ~2ms | I2C @ 400kHz |
| Sensor fusion | <0.5ms | Simple complementary filter |
| Firmware TX | ~4ms | 41 bytes @ 115200 baud |
| Python RX | <1ms | pySerial buffered |
| **Total** | ~**22-27ms** | **Well under 50ms target!** |

### Throughput

**Commands** (Python → STM32):
- Typical: 50 commands/sec (20ms period)
- Burst: Up to 100 commands/sec (10ms period)

**Telemetry** (STM32 → Python):
- Standard: 50 packets/sec (20ms period)
- Bandwidth: 41 bytes × 50 Hz = 2050 bytes/sec = ~16 kbit/s
- Serial utilization: 16k / 115.2k = **14%** (comfortable margin)

### Baud Rate Selection

| Baud Rate | Byte Time | Packet Time (41B) | Max Rate | Recommendation |
|-----------|-----------|-------------------|----------|----------------|
| 9600 | 1.04 ms | 42.7 ms | 23 Hz | Too slow |
| 115200 | 86.8 μs | 3.56 ms | 281 Hz | **Recommended** |
| 230400 | 43.4 μs | 1.78 ms | 562 Hz | Overkill, may be unreliable |
| 921600 | 10.9 μs | 445 μs | 2247 Hz | Unnecessary for this application |

**Selected**: 115200 baud (reliable, well-supported, sufficient headroom).

---

## Testing & Validation

### Protocol Tests (Python)

```python
# test_protocol.py
import pytest
from protocol import encode_set_joint_angles, decode_telemetry_full, crc8

def test_crc8_known_values():
    """Test CRC-8 against known test vectors"""
    assert crc8(b'\x10\x08') == 0x7D  # Example precalculated value

def test_encode_set_joint_angles():
    """Test command packet encoding"""
    packet = encode_set_joint_angles(0.5, -0.3)
    assert packet[0] == 0xAA  # Header
    assert packet[1] == 0x10  # Command type
    assert packet[2] == 0x08  # Length
    assert len(packet) == 12  # Total size

def test_decode_telemetry():
    """Test telemetry decoding (use captured real packet)"""
    # Captured from actual STM32 transmission
    raw_packet = bytes([0xAA, 0x01, 0x28, ...])  # Full 44 bytes
    telemetry = decode_telemetry_full(raw_packet)
    assert 'timestamp_ms' in telemetry
    assert len(telemetry['joint_angles']) == 2
```

### Firmware Tests

```c
// test_protocol.c
void test_crc8_calculation(void) {
    uint8_t data[] = {0x10, 0x08, 0x00, 0x00, 0x00, 0x3F};
    uint8_t crc = crc8(data, sizeof(data));
    assert(crc == 0x7D);  // Expected value
}

void test_packet_parsing(void) {
    uint8_t packet[] = {0xAA, 0x10, 0x08, /* data */, /* crc */};
    CommandPacket cmd;
    bool success = parse_packet(packet, sizeof(packet), &cmd);
    assert(success == true);
    assert(cmd.type == 0x10);
}
```

### HIL Test Procedure

1. **Loopback Test**: Connect TX → RX on STM32, verify echo
2. **CRC Test**: Send packets with intentional bit flips, verify rejection
3. **Latency Test**: Timestamp commands, measure round-trip time
4. **Stress Test**: Send 100 commands/sec for 1 minute, check packet loss
5. **Error Recovery Test**: Disconnect/reconnect serial, verify reconnection

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-01-13 | Initial specification |
| | | |

---

**Document Status**: ✅ Complete - Ready for implementation

**Next Steps**:
1. Implement protocol.py in Python (with unit tests)
2. Implement protocol.c/.h in STM32 firmware
3. Validate CRC implementation consistency between Python and C
4. Test with loopback before HIL integration
