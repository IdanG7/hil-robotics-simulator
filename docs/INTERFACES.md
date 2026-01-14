# Component Interfaces & Data Structures

**Project**: HIL Robotics Simulator
**Version**: 1.0
**Date**: January 13, 2026

---

## Overview

This document defines the interfaces between major system components, data structures, and API contracts. It serves as a reference for implementation and ensures consistency across firmware (C/C++) and simulation (Python).

---

## Table of Contents

1. [Firmware-Simulation Interface](#firmware-simulation-interface)
2. [Firmware Internal Interfaces](#firmware-internal-interfaces)
3. [Python Module Interfaces](#python-module-interfaces)
4. [Data Structures](#data-structures)
5. [Configuration](#configuration)

---

## Firmware-Simulation Interface

### Serial Communication Contract

**Direction**: Bidirectional (USB-Serial via USART2)
**Baud Rate**: 115200
**Format**: 8-N-1

#### Firmware → Simulation (Telemetry)

**Rate**: 50Hz (every 20ms)
**Packet Type**: `0x01` (TELEMETRY_FULL)

```python
# Python representation
class TelemetryData:
    timestamp_ms: int           # uint32_t, system time
    joint_angles: List[float]   # 2x float32, [shoulder, elbow] in radians
    joint_velocities: List[float]  # 2x float32, rad/s (estimated)
    imu_accel: List[float]      # 3x float32, [x, y, z] in m/s²
    imu_gyro: List[float]       # 3x float32, [x, y, z] in rad/s
    imu_orientation: List[float]   # 2x float32, [roll, pitch] in radians
```

```c
// C representation (firmware)
typedef struct {
    uint32_t timestamp_ms;
    float joint_angles[2];
    float joint_velocities[2];
    float imu_accel[3];
    float imu_gyro[3];
    float imu_orientation[2];
} TelemetryFull_t;
```

#### Simulation → Firmware (Commands)

**Primary Command**: `0x10` (SET_JOINT_ANGLES)

```python
# Python API
def send_joint_angles(shoulder: float, elbow: float) -> None:
    """
    Command target joint angles.

    Args:
        shoulder: Shoulder joint angle in radians [-π/2, π/2]
        elbow: Elbow joint angle in radians [-π/2, π/2]

    Raises:
        ValueError: If angles out of valid range
        SerialException: If communication error
    """
```

```c
// C representation (firmware)
typedef struct {
    float shoulder_angle;  // radians
    float elbow_angle;     // radians
} SetJointAnglesCmd_t;
```

---

## Firmware Internal Interfaces

### Module: UART Handler

**File**: `uart_handler.h` / `uart_handler.c`

**Purpose**: Low-level UART communication with DMA and interrupt handling.

#### Public API

```c
#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include "stdint.h"
#include "stdbool.h"

// Initialize UART with DMA
void UART_Init(uint32_t baudrate);

// Non-blocking transmit
bool UART_Transmit(const uint8_t *data, uint16_t length);

// Check if RX data available
uint16_t UART_GetRxAvailable(void);

// Read from RX buffer (non-blocking)
uint16_t UART_Read(uint8_t *buffer, uint16_t max_length);

// Clear RX buffer
void UART_FlushRx(void);

#endif
```

**Callbacks** (defined by user):
```c
// Called when RX DMA buffer half-full
void UART_RxHalfCallback(void);

// Called when RX DMA buffer full
void UART_RxCompleteCallback(void);
```

---

### Module: Protocol Handler

**File**: `protocol.h` / `protocol.c`

**Purpose**: Packet framing, parsing, and CRC validation.

#### Public API

```c
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "stdint.h"
#include "stdbool.h"

#define PROTOCOL_MAX_PAYLOAD 64
#define PROTOCOL_HEADER 0xAA

// Command types (Simulation → Hardware)
typedef enum {
    CMD_SET_JOINT_ANGLES = 0x10,
    CMD_SET_JOINT_ANGLE_SINGLE = 0x11,
    CMD_GET_TELEMETRY = 0x20,
    CMD_SYSTEM_RESET = 0x30,
    CMD_CALIBRATE_IMU = 0x31,
    CMD_SET_PID_GAINS = 0x40,
    CMD_SET_PID_GAINS_SINGLE = 0x41,
    CMD_SET_MODE = 0x50,
} CommandType_t;

// Telemetry types (Hardware → Simulation)
typedef enum {
    TELEM_FULL = 0x01,
    TELEM_ANGLES_ONLY = 0x02,
    TELEM_IMU_ONLY = 0x03,
    TELEM_ERROR = 0xF0,
    TELEM_ACK = 0xF1,
} TelemetryType_t;

// Generic packet structure
typedef struct {
    uint8_t header;
    uint8_t type;
    uint8_t length;
    uint8_t data[PROTOCOL_MAX_PAYLOAD];
    uint8_t crc;
} Packet_t;

// Parse incoming packet from byte stream
// Returns true if complete valid packet found
bool Protocol_ParsePacket(const uint8_t *buffer, uint16_t length, Packet_t *packet);

// Encode command into packet
uint16_t Protocol_EncodePacket(uint8_t type, const uint8_t *data, uint8_t data_len, uint8_t *output);

// Compute CRC-8
uint8_t Protocol_CRC8(const uint8_t *data, uint16_t length);

// Validate packet CRC
bool Protocol_ValidateCRC(const Packet_t *packet);

#endif
```

---

### Module: Servo Control

**File**: `servo_control.h` / `servo_control.c`

**Purpose**: PWM generation and PID position control for servos.

#### Public API

```c
#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "stdint.h"
#include "stdbool.h"

typedef enum {
    SERVO_SHOULDER = 0,
    SERVO_ELBOW = 1,
    SERVO_COUNT = 2
} ServoID_t;

// PID gains structure
typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PIDGains_t;

// Initialize servo PWM (TIM2/TIM3)
void Servo_Init(void);

// Set target angle for servo (radians)
void Servo_SetTargetAngle(ServoID_t servo, float angle);

// Get current angle (radians)
float Servo_GetCurrentAngle(ServoID_t servo);

// Update PID controller (call at 50Hz)
void Servo_UpdateControl(void);

// Set PID gains
void Servo_SetPIDGains(ServoID_t servo, const PIDGains_t *gains);

// Get PID gains
void Servo_GetPIDGains(ServoID_t servo, PIDGains_t *gains);

// Enable/disable servo output
void Servo_Enable(ServoID_t servo, bool enable);

// Emergency stop (set to neutral position)
void Servo_EmergencyStop(void);

#endif
```

**Internal Constants**:
```c
// servo_control.c
#define SERVO_PWM_FREQ_HZ 50       // 20ms period
#define SERVO_MIN_PULSE_US 1000    // 0° position
#define SERVO_MAX_PULSE_US 2000    // 180° position
#define SERVO_NEUTRAL_PULSE_US 1500  // 90° position

#define SERVO_MIN_ANGLE_RAD (-M_PI/2)
#define SERVO_MAX_ANGLE_RAD (M_PI/2)

#define PID_UPDATE_FREQ_HZ 50
#define PID_DT_SEC (1.0f / PID_UPDATE_FREQ_HZ)
```

---

### Module: IMU Driver

**File**: `imu_driver.h` / `imu_driver.c`

**Purpose**: MPU6050 I2C communication and data acquisition.

#### Public API

```c
#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "stdint.h"
#include "stdbool.h"

// IMU data structure (raw)
typedef struct {
    int16_t accel_x;    // Raw accelerometer X
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;       // Temperature (not used)
    int16_t gyro_x;     // Raw gyroscope X
    int16_t gyro_y;
    int16_t gyro_z;
} IMU_RawData_t;

// IMU data structure (scaled)
typedef struct {
    float accel[3];     // m/s² [x, y, z]
    float gyro[3];      // rad/s [x, y, z]
} IMU_ScaledData_t;

// Initialize MPU6050 (I2C1)
bool IMU_Init(void);

// Read raw data from IMU
bool IMU_ReadRaw(IMU_RawData_t *data);

// Read and scale data
bool IMU_ReadScaled(IMU_ScaledData_t *data);

// Calibrate IMU (compute bias offsets)
// Requires arm to be stationary
bool IMU_Calibrate(uint16_t num_samples);

// Get calibration status
bool IMU_IsCalibrated(void);

#endif
```

**I2C Configuration**:
```c
// imu_driver.c
#define MPU6050_I2C_ADDR 0x68
#define MPU6050_I2C_TIMEOUT_MS 100

#define MPU6050_ACCEL_SCALE_2G  16384.0f  // LSB/g
#define MPU6050_GYRO_SCALE_250  131.0f    // LSB/(deg/s)

#define GRAVITY_MS2 9.81f
#define DEG_TO_RAD (M_PI / 180.0f)
```

---

### Module: Sensor Fusion

**File**: `sensor_fusion.h` / `sensor_fusion.c`

**Purpose**: Complementary filter for orientation estimation.

#### Public API

```c
#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "imu_driver.h"

// Orientation output
typedef struct {
    float roll;     // radians
    float pitch;    // radians
} Orientation_t;

// Initialize sensor fusion
void SensorFusion_Init(void);

// Update filter with new IMU data
void SensorFusion_Update(const IMU_ScaledData_t *imu_data, float dt);

// Get current orientation estimate
void SensorFusion_GetOrientation(Orientation_t *orientation);

// Reset orientation to zero
void SensorFusion_Reset(void);

#endif
```

**Algorithm Parameters**:
```c
// sensor_fusion.c
#define COMP_FILTER_ALPHA 0.98f  // Gyro weight (0.98 = trust gyro 98%)
```

---

## Python Module Interfaces

### Module: serial_manager.py

**Purpose**: Serial port handling with reconnection logic.

#### Public API

```python
from typing import Optional
import serial

class SerialManager:
    """Manages USB-Serial communication with STM32."""

    def __init__(self, port: str = 'COM3', baud: int = 115200, timeout: float = 0.1):
        """
        Initialize serial connection.

        Args:
            port: Serial port name (e.g., 'COM3', '/dev/ttyUSB0')
            baud: Baud rate (default: 115200)
            timeout: Read timeout in seconds (default: 0.1)

        Raises:
            serial.SerialException: If port cannot be opened
        """

    def send_packet(self, packet: bytes) -> None:
        """
        Send binary packet to STM32.

        Args:
            packet: Complete packet bytes (header, type, data, CRC)

        Raises:
            serial.SerialException: If write fails
        """

    def read_packet(self) -> Optional[bytes]:
        """
        Read one complete packet from serial buffer (non-blocking).

        Returns:
            Complete packet bytes if available, None otherwise
        """

    def flush(self) -> None:
        """Clear RX/TX buffers."""

    def close(self) -> None:
        """Close serial port."""

    @property
    def is_connected(self) -> bool:
        """Check if serial port is open and responsive."""
```

---

### Module: protocol.py

**Purpose**: Binary protocol encoding/decoding (mirrors firmware).

#### Public API

```python
from typing import List, Dict, Any, Optional
from enum import IntEnum

class CommandType(IntEnum):
    """Command types (Python → STM32)"""
    SET_JOINT_ANGLES = 0x10
    SET_JOINT_ANGLE_SINGLE = 0x11
    GET_TELEMETRY = 0x20
    SYSTEM_RESET = 0x30
    CALIBRATE_IMU = 0x31
    SET_PID_GAINS = 0x40
    SET_MODE = 0x50

class TelemetryType(IntEnum):
    """Telemetry types (STM32 → Python)"""
    FULL = 0x01
    ANGLES_ONLY = 0x02
    IMU_ONLY = 0x03
    ERROR = 0xF0
    ACK = 0xF1

def encode_packet(cmd_type: int, data: bytes) -> bytes:
    """
    Encode command into binary packet.

    Args:
        cmd_type: Command type from CommandType enum
        data: Payload bytes (struct-packed)

    Returns:
        Complete packet with header, length, CRC
    """

def decode_packet(packet: bytes) -> Optional[Dict[str, Any]]:
    """
    Decode binary packet.

    Args:
        packet: Complete packet bytes

    Returns:
        Dict with 'type', 'data' keys if valid, None if invalid CRC

    Raises:
        ValueError: If packet format invalid
    """

def crc8(data: bytes) -> int:
    """
    Compute CRC-8 checksum.

    Args:
        data: Bytes to checksum

    Returns:
        CRC-8 value (0x00-0xFF)
    """

# Convenience functions for common commands

def encode_set_joint_angles(shoulder: float, elbow: float) -> bytes:
    """Encode SET_JOINT_ANGLES command."""

def encode_set_pid_gains(shoulder_gains: Dict[str, float],
                         elbow_gains: Dict[str, float]) -> bytes:
    """Encode SET_PID_GAINS command."""

def decode_telemetry_full(packet: bytes) -> Dict[str, Any]:
    """
    Decode TELEMETRY_FULL packet.

    Returns:
        Dict with keys: timestamp_ms, joint_angles, joint_velocities,
                       imu_accel, imu_gyro, imu_orientation
    """
```

---

### Module: mujoco_controller.py

**Purpose**: MuJoCo simulation wrapper.

#### Public API

```python
import mujoco
import numpy as np
from typing import Tuple

class MuJoCoController:
    """Wrapper for MuJoCo arm simulation."""

    def __init__(self, model_path: str = '../mujoco_model/arm.xml'):
        """
        Load MuJoCo model.

        Args:
            model_path: Path to MJCF XML file
        """

    def reset(self) -> None:
        """Reset simulation to initial state."""

    def set_joint_angles(self, angles: np.ndarray) -> None:
        """
        Set joint angles directly (no physics step).

        Args:
            angles: Joint angles [shoulder, elbow] in radians
        """

    def get_joint_angles(self) -> np.ndarray:
        """
        Get current joint angles.

        Returns:
            Joint angles [shoulder, elbow] in radians
        """

    def step(self, dt: float = 0.002) -> None:
        """
        Step physics simulation.

        Args:
            dt: Timestep in seconds (default: 2ms, 500Hz internal)
        """

    def get_end_effector_position(self) -> Tuple[float, float]:
        """
        Get end-effector position in world frame.

        Returns:
            (x, y) position in meters
        """

    def render(self) -> np.ndarray:
        """
        Render current frame.

        Returns:
            RGB image array (height x width x 3)
        """
```

---

### Module: kinematics.py

**Purpose**: Forward and inverse kinematics for 2-DOF arm.

#### Public API

```python
import numpy as np
from typing import Tuple

class ArmKinematics:
    """2-DOF arm kinematics."""

    def __init__(self, L1: float = 0.1, L2: float = 0.08):
        """
        Initialize with link lengths.

        Args:
            L1: Shoulder to elbow length (meters)
            L2: Elbow to end-effector length (meters)
        """

    def forward(self, theta1: float, theta2: float) -> Tuple[float, float]:
        """
        Compute forward kinematics.

        Args:
            theta1: Shoulder angle (radians)
            theta2: Elbow angle (radians)

        Returns:
            (x, y) end-effector position in meters
        """

    def inverse(self, x: float, y: float) -> Tuple[float, float]:
        """
        Compute inverse kinematics (elbow-up solution).

        Args:
            x: Target X position (meters)
            y: Target Y position (meters)

        Returns:
            (theta1, theta2) joint angles in radians

        Raises:
            ValueError: If target unreachable
        """

    def jacobian(self, theta1: float, theta2: float) -> np.ndarray:
        """
        Compute Jacobian matrix (velocity kinematics).

        Args:
            theta1: Shoulder angle
            theta2: Elbow angle

        Returns:
            2x2 Jacobian matrix [dX/dtheta]
        """
```

---

## Data Structures

### Configuration (YAML)

**File**: `simulation/config.yaml`

```yaml
# Serial Communication
serial:
  port: "COM3"                 # Windows: COM3, Linux: /dev/ttyUSB0
  baud: 115200
  timeout: 0.1                 # seconds

# Control Loop
control:
  update_rate_hz: 50           # Control loop frequency
  telemetry_rate_hz: 50        # Telemetry send rate

# Robot Parameters
robot:
  link_lengths: [0.10, 0.08]   # [L1, L2] in meters
  joint_limits:                # Joint angle limits (radians)
    shoulder: [-1.57, 1.57]    # ±90°
    elbow: [-1.57, 1.57]

# PID Gains (initial values)
pid:
  shoulder:
    kp: 1.5
    ki: 0.05
    kd: 0.15
  elbow:
    kp: 1.2
    ki: 0.03
    kd: 0.12

# Sensor Fusion
sensor_fusion:
  complementary_filter_alpha: 0.98
  imu_calibration_samples: 500

# Visualization
visualization:
  window_size: [1200, 800]
  plot_history_seconds: 10
  render_fps: 30
```

---

## Error Handling Contract

### Firmware Error Responses

When firmware encounters an error, it sends `TELEM_ERROR` packet:

```c
typedef struct {
    uint8_t error_code;
    uint8_t failed_cmd;
    char message[32];
} ErrorResponse_t;
```

**Error Codes**:
- `0x01`: Invalid command
- `0x02`: CRC mismatch
- `0x03`: Out of range
- `0x04`: Timeout
- `0x05`: Hardware failure
- `0x06`: System busy

### Python Exception Hierarchy

```python
class HILException(Exception):
    """Base exception for HIL system."""

class SerialException(HILException):
    """Serial communication error."""

class ProtocolException(HILException):
    """Protocol parsing/validation error."""

class KinematicsException(HILException):
    """Kinematics computation error (unreachable target)."""

class HardwareException(HILException):
    """Hardware error reported by firmware."""
```

---

## Thread Safety

### Firmware (Single-threaded)

- **Main loop**: 50Hz control loop
- **Interrupts**: UART RX/TX (higher priority than main)
- **Synchronization**: Use volatile for shared variables, disable interrupts for critical sections

### Python (Multi-threaded)

- **Serial thread**: Dedicated thread for serial I/O
- **Main thread**: MuJoCo simulation and control logic
- **UI thread**: Visualization rendering (PyQt5)
- **Synchronization**: Use `threading.Lock` for shared state

```python
import threading

class HILSynchronizer:
    def __init__(self):
        self._state_lock = threading.Lock()
        self._telemetry_data = None

    def update_telemetry(self, data):
        with self._state_lock:
            self._telemetry_data = data

    def get_telemetry(self):
        with self._state_lock:
            return self._telemetry_data.copy()
```

---

## Versioning & Compatibility

### Protocol Version

Current: **v1.0**

Future versions may add:
- Version field in packet header
- Negotiation handshake
- Backward compatibility support

### Firmware-Simulation Compatibility

**Contract**:
- Same protocol version required
- CRC algorithm must match exactly
- Endianness: Little-endian (ARM Cortex-M4, x86/x64)

**Verification**:
- Unit tests with shared test vectors
- CRC test packet: `[0x10, 0x08, ...]` → Expected CRC: `0x7D`

---

## Document Status

✅ **Complete** - Ready for implementation

**Next Steps**:
1. Implement Python protocol.py with unit tests
2. Implement firmware protocol.c/.h
3. Validate CRC implementation consistency
4. Create integration test suite

---

**Last Updated**: January 13, 2026
