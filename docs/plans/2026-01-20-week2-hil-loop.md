# Week 2: HIL Communication Loop Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Establish bidirectional communication between MuJoCo simulation and STM32 hardware, implement servo control with PID, and demonstrate the first hardware-in-the-loop synchronization.

**Architecture:** Structured firmware with 5 modules (UART, Protocol, Servo, PID, IMU) running at 50Hz control loop. Python HIL synchronizer commands hardware at 25Hz, receives telemetry, and updates MuJoCo visualization in real-time. Binary protocol with CRC-8 validation ensures reliable communication.

**Tech Stack:** STM32 HAL (C), Python 3.10+, pySerial, MuJoCo 3.0+, pytest

---

## Overview

**Timeline**: 6 days (January 14-20, 2026)
**Approach**: Days 1-3 focus on communication foundation, Days 4-6 add servos and HIL loop

**Success Criteria** (all must pass):
1. ✅ Communication working: Reliable Python ↔ STM32 with CRC validation
2. ✅ Servos responding: Physical movement matches commanded angles
3. ✅ Basic HIL loop: MuJoCo → Hardware → Telemetry → Visualization
4. ✅ IMU data streaming: Raw accel/gyro in telemetry packets

**Reference Documents**:
- `docs/WEEK2_DESIGN.md` - High-level architecture
- `docs/WEEK2_FIRMWARE_GUIDE.md` - Detailed firmware code
- `docs/PROTOCOL.md` - Binary protocol specification

---

## Phase 1: Communication Foundation (Days 1-3)

### Task 1: STM32CubeIDE Project Creation

**Files:**
- Create: STM32CubeIDE project via GUI
- Create: `firmware/Core/Src/main.c` (generated)
- Create: `firmware/Core/Inc/main.h` (generated)
- Create: `firmware/hil_firmware.ioc` (configuration)

**Step 1: Launch STM32CubeIDE and create project**

Follow instructions in `firmware/STM32CubeIDE_SETUP_INSTRUCTIONS.md`:

1. File → New → STM32 Project
2. Board Selector → "NUCLEO-F446RE"
3. Project Name: `hil_firmware`
4. Location: `D:\Projects\Robot\firmware`
5. Language: C
6. Initialize with default peripherals: Yes

**Step 2: Configure USART2 (USB-Serial)**

In `.ioc` file:
- Mode: Asynchronous
- Baud: 115200
- DMA Settings:
  - USART2_RX: DMA1 Stream 5, Circular mode
  - USART2_TX: DMA1 Stream 6, Normal mode

**Step 3: Configure TIM2 and TIM3 (Servo PWM)**

TIM2 (Servo 1):
- Clock: Internal
- Channel 1: PWM Generation CH1
- Prescaler: 179 (1MHz timer clock)
- Counter Period: 19999 (50Hz, 20ms)
- Pulse: 1500 (1.5ms neutral)

TIM3 (Servo 2):
- Clock: Internal
- Channel 2: PWM Generation CH2
- Prescaler: 179
- Counter Period: 19999
- Pulse: 1500

**Step 4: Configure I2C1 (MPU6050)**

I2C1:
- Mode: I2C
- Speed: 100kHz
- Pins: PB8 (SCL), PB9 (SDA)

**Step 5: Configure GPIO**

PA5: GPIO_Output (LED)

**Step 6: Generate code**

Project → Generate Code (Ctrl+S in .ioc)

**Step 7: Build firmware to verify setup**

Project → Build Project (Ctrl+B)

Expected: BUILD SUCCESSFUL (0 errors, some warnings OK)

**Step 8: Commit**

```bash
git add firmware/Core/ firmware/.project firmware/.cproject firmware/hil_firmware.ioc
git commit -m "feat(firmware): initialize STM32CubeIDE project

- Configure USART2 for serial (115200 baud, DMA)
- Configure TIM2/TIM3 for servo PWM (50Hz)
- Configure I2C1 for MPU6050
- Project builds successfully"
```

---

### Task 2: Firmware UART Handler Module

**Files:**
- Create: `firmware/Core/Inc/uart_handler.h`
- Create: `firmware/Core/Src/uart_handler.c`
- Modify: `firmware/Core/Src/stm32f4xx_it.c` (add callbacks)

**Step 1: Create uart_handler.h header**

Create `firmware/Core/Inc/uart_handler.h`:

```c
#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define UART_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 128

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
    uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile bool tx_busy;
} UART_Handle_t;

void UART_Init(UART_Handle_t *handle, UART_HandleTypeDef *huart);
bool UART_BytesAvailable(UART_Handle_t *handle);
uint8_t UART_ReadByte(UART_Handle_t *handle);
bool UART_SendPacket(UART_Handle_t *handle, const uint8_t *data, uint16_t len);
void UART_RxCallback(UART_Handle_t *handle);
void UART_TxCallback(UART_Handle_t *handle);

#endif // UART_HANDLER_H
```

**Step 2: Implement uart_handler.c**

Create `firmware/Core/Src/uart_handler.c`:

```c
#include "uart_handler.h"
#include <string.h>

void UART_Init(UART_Handle_t *handle, UART_HandleTypeDef *huart) {
    handle->huart = huart;
    handle->rx_head = 0;
    handle->rx_tail = 0;
    handle->tx_busy = false;
    HAL_UART_Receive_DMA(huart, handle->rx_buffer, UART_RX_BUFFER_SIZE);
}

bool UART_BytesAvailable(UART_Handle_t *handle) {
    handle->rx_head = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(handle->huart->hdmarx);
    return (handle->rx_head != handle->rx_tail);
}

uint8_t UART_ReadByte(UART_Handle_t *handle) {
    uint8_t byte = handle->rx_buffer[handle->rx_tail];
    handle->rx_tail = (handle->rx_tail + 1) % UART_RX_BUFFER_SIZE;
    return byte;
}

bool UART_SendPacket(UART_Handle_t *handle, const uint8_t *data, uint16_t len) {
    if (handle->tx_busy || len > UART_TX_BUFFER_SIZE) {
        return false;
    }
    memcpy(handle->tx_buffer, data, len);
    handle->tx_busy = true;
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(handle->huart, handle->tx_buffer, len);
    if (status != HAL_OK) {
        handle->tx_busy = false;
        return false;
    }
    return true;
}

void UART_RxCallback(UART_Handle_t *handle) {
    // Called from HAL_UART_RxCpltCallback - nothing needed for circular DMA
}

void UART_TxCallback(UART_Handle_t *handle) {
    handle->tx_busy = false;
}
```

**Step 3: Add HAL callbacks to stm32f4xx_it.c**

Add to `firmware/Core/Src/stm32f4xx_it.c` (after includes):

```c
// Add near top after includes
extern UART_Handle_t uart_handle;

// Add these functions at the end of file
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        UART_TxCallback(&uart_handle);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        UART_RxCallback(&uart_handle);
    }
}
```

**Step 4: Build to verify no errors**

Run: Build Project (Ctrl+B)
Expected: 0 errors (warnings about unused functions OK)

**Step 5: Commit**

```bash
git add firmware/Core/Inc/uart_handler.h firmware/Core/Src/uart_handler.c firmware/Core/Src/stm32f4xx_it.c
git commit -m "feat(firmware): add UART handler with DMA support

- Circular DMA for RX buffer
- Non-blocking TX with DMA
- HAL interrupt callbacks"
```

---

### Task 3: Firmware Protocol Parser Module

**Files:**
- Create: `firmware/Core/Inc/protocol.h`
- Create: `firmware/Core/Src/protocol.c`

**Step 1: Create protocol.h header**

Create `firmware/Core/Inc/protocol.h`:

```c
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define PACKET_HEADER 0xAA
#define MAX_PAYLOAD_SIZE 64

typedef enum {
    CMD_SET_JOINT_ANGLES = 0x10,
    CMD_SET_JOINT_ANGLE_SINGLE = 0x11,
    CMD_GET_TELEMETRY = 0x20,
    CMD_SYSTEM_RESET = 0x30,
    CMD_CALIBRATE_IMU = 0x31,
    CMD_SET_PID_GAINS = 0x40,
    CMD_SET_MODE = 0x50
} CommandType_t;

typedef enum {
    TEL_FULL = 0x01,
    TEL_ANGLES_ONLY = 0x02,
    TEL_IMU_ONLY = 0x03,
    TEL_ERROR = 0xF0,
    TEL_ACK = 0xF1
} TelemetryType_t;

typedef struct {
    uint8_t type;
    uint8_t length;
    uint8_t data[MAX_PAYLOAD_SIZE];
} Packet_t;

uint8_t Protocol_CRC8(const uint8_t *data, uint16_t len);
bool Protocol_DecodePacket(const uint8_t *raw_buffer, uint16_t len, Packet_t *packet);
uint16_t Protocol_EncodePacket(const Packet_t *packet, uint8_t *output_buffer);

#endif // PROTOCOL_H
```

**Step 2: Implement protocol.c**

Create `firmware/Core/Src/protocol.c`:

```c
#include "protocol.h"
#include <string.h>

uint8_t Protocol_CRC8(const uint8_t *data, uint16_t len) {
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool Protocol_DecodePacket(const uint8_t *raw_buffer, uint16_t len, Packet_t *packet) {
    if (len < 4) {
        return false;
    }
    if (raw_buffer[0] != PACKET_HEADER) {
        return false;
    }
    uint8_t type = raw_buffer[1];
    uint8_t length = raw_buffer[2];
    if (length > MAX_PAYLOAD_SIZE || len != (4 + length)) {
        return false;
    }
    uint8_t crc_expected = Protocol_CRC8(&raw_buffer[1], 2 + length);
    uint8_t crc_received = raw_buffer[3 + length];
    if (crc_expected != crc_received) {
        return false;
    }
    packet->type = type;
    packet->length = length;
    if (length > 0) {
        memcpy(packet->data, &raw_buffer[3], length);
    }
    return true;
}

uint16_t Protocol_EncodePacket(const Packet_t *packet, uint8_t *output_buffer) {
    if (packet->length > MAX_PAYLOAD_SIZE) {
        return 0;
    }
    output_buffer[0] = PACKET_HEADER;
    output_buffer[1] = packet->type;
    output_buffer[2] = packet->length;
    if (packet->length > 0) {
        memcpy(&output_buffer[3], packet->data, packet->length);
    }
    uint8_t crc = Protocol_CRC8(&output_buffer[1], 2 + packet->length);
    output_buffer[3 + packet->length] = crc;
    return 4 + packet->length;
}
```

**Step 3: Build to verify**

Run: Build Project
Expected: 0 errors

**Step 4: Commit**

```bash
git add firmware/Core/Inc/protocol.h firmware/Core/Src/protocol.c
git commit -m "feat(firmware): add binary protocol parser

- CRC-8 validation (matches Python)
- Packet encode/decode
- Command and telemetry types"
```

---

### Task 4: Firmware Echo Test

**Files:**
- Modify: `firmware/Core/Src/main.c`
- Modify: `firmware/Core/Inc/main.h`

**Step 1: Add includes and globals to main.c**

Add to `firmware/Core/Src/main.c` (after USER CODE BEGIN Includes):

```c
/* USER CODE BEGIN Includes */
#include "uart_handler.h"
#include "protocol.h"
/* USER CODE END Includes */
```

Add global variables (after USER CODE BEGIN PV):

```c
/* USER CODE BEGIN PV */
UART_Handle_t uart_handle;
/* USER CODE END PV */
```

**Step 2: Initialize UART handler in main()**

Add to main() after MX_USART2_UART_Init() (in USER CODE BEGIN 2):

```c
/* USER CODE BEGIN 2 */
UART_Init(&uart_handle, &huart2);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // LED off
/* USER CODE END 2 */
```

**Step 3: Add echo loop in main() while loop**

Add to main() while(1) loop (in USER CODE BEGIN 3):

```c
/* USER CODE BEGIN 3 */
// Echo test: receive packet, validate, echo back
static uint8_t packet_buffer[128];
static uint16_t buffer_pos = 0;

while (UART_BytesAvailable(&uart_handle)) {
    uint8_t byte = UART_ReadByte(&uart_handle);

    if (buffer_pos == 0 && byte != PACKET_HEADER) {
        continue;
    }

    packet_buffer[buffer_pos++] = byte;

    if (buffer_pos >= 3) {
        uint8_t expected_len = 4 + packet_buffer[2];
        if (buffer_pos >= expected_len) {
            Packet_t packet;
            if (Protocol_DecodePacket(packet_buffer, buffer_pos, &packet)) {
                // Valid packet - toggle LED and echo back
                HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

                // Send ACK
                Packet_t ack;
                ack.type = TEL_ACK;
                ack.length = 0;
                uint8_t output[16];
                uint16_t len = Protocol_EncodePacket(&ack, output);
                UART_SendPacket(&uart_handle, output, len);
            }
            buffer_pos = 0;
        }
    }

    if (buffer_pos >= sizeof(packet_buffer)) {
        buffer_pos = 0;
    }
}
/* USER CODE END 3 */
```

**Step 4: Build firmware**

Run: Build Project
Expected: 0 errors

**Step 5: Flash firmware to board**

Run: Debug (F11) or Run (Ctrl+F11)
Expected: Firmware flashed successfully, LED should be on

**Step 6: Commit**

```bash
git add firmware/Core/Src/main.c
git commit -m "feat(firmware): add echo test firmware

- Receive packets via UART
- Validate CRC-8
- Toggle LED on valid packet
- Echo ACK back to sender"
```

---

### Task 5: Python Serial Manager (TDD)

**Files:**
- Create: `simulation/middleware/serial_manager.py`
- Create: `simulation/middleware/tests/test_serial_manager.py`

**Step 1: Write failing test for SerialManager initialization**

Create `simulation/middleware/tests/test_serial_manager.py`:

```python
"""Tests for serial_manager module."""

import pytest
from unittest.mock import Mock, patch, MagicMock
from middleware.serial_manager import SerialManager


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
```

**Step 2: Run test to verify it fails**

Run:
```bash
cd simulation
pytest middleware/tests/test_serial_manager.py::TestSerialManagerInit -v
```

Expected: `ModuleNotFoundError: No module named 'middleware.serial_manager'`

**Step 3: Implement minimal SerialManager class**

Create `simulation/middleware/serial_manager.py`:

```python
"""Serial communication manager for hardware interface."""

import serial
import threading
import time
from typing import Optional, Dict
from middleware.protocol import encode_packet, decode_packet, CommandType, TelemetryType


class SerialManager:
    """Manages serial communication with STM32 hardware.

    Features:
    - Automatic reconnection on disconnect
    - Thread-safe send/receive
    - Timeout detection
    """

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.1):
        """Initialize serial manager.

        Args:
            port: Serial port name (e.g., 'COM3', '/dev/ttyUSB0')
            baud: Baud rate (default: 115200)
            timeout: Read timeout in seconds (default: 0.1)
        """
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._connected = False

    def is_connected(self) -> bool:
        """Check if serial port is connected."""
        return self._connected
```

**Step 4: Run test to verify it passes**

Run:
```bash
pytest middleware/tests/test_serial_manager.py::TestSerialManagerInit -v
```

Expected: 2 tests PASS

**Step 5: Commit**

```bash
git add simulation/middleware/serial_manager.py simulation/middleware/tests/test_serial_manager.py
git commit -m "feat(middleware): add SerialManager init and is_connected

- Store port, baud, timeout
- Initialize as disconnected
- Thread-safe with lock"
```

---

### Task 6: Python Serial Manager Connect/Disconnect (TDD)

**Files:**
- Modify: `simulation/middleware/serial_manager.py`
- Modify: `simulation/middleware/tests/test_serial_manager.py`

**Step 1: Write failing test for connect/disconnect**

Add to `test_serial_manager.py`:

```python
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
```

**Step 2: Run test to verify it fails**

Run:
```bash
pytest middleware/tests/test_serial_manager.py::TestSerialManagerConnection -v
```

Expected: `AttributeError: 'SerialManager' object has no attribute 'connect'`

**Step 3: Implement connect and disconnect methods**

Add to `serial_manager.py` in SerialManager class:

```python
    def connect(self) -> bool:
        """Connect to serial port.

        Returns:
            True if connected successfully, False otherwise
        """
        try:
            with self._lock:
                self._serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
                self._connected = True
            return True
        except serial.SerialException as e:
            self._connected = False
            return False

    def disconnect(self) -> None:
        """Disconnect from serial port."""
        with self._lock:
            if self._serial is not None:
                self._serial.close()
                self._serial = None
            self._connected = False
```

**Step 4: Run test to verify it passes**

Run:
```bash
pytest middleware/tests/test_serial_manager.py::TestSerialManagerConnection -v
```

Expected: 3 tests PASS

**Step 5: Commit**

```bash
git add simulation/middleware/serial_manager.py simulation/middleware/tests/test_serial_manager.py
git commit -m "feat(middleware): add SerialManager connect/disconnect

- Connect to serial port with error handling
- Disconnect and close port
- Thread-safe operations"
```

---

### Task 7: Python Serial Manager Send/Receive (TDD)

**Files:**
- Modify: `simulation/middleware/serial_manager.py`
- Modify: `simulation/middleware/tests/test_serial_manager.py`

**Step 1: Write failing test for send_command**

Add to `test_serial_manager.py`:

```python
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
        mock_serial.read.return_value = bytes([0xAA, 0xF1, 0x00, 0xF0])
        mock_serial_class.return_value = mock_serial

        manager = SerialManager(port='COM3')
        manager.connect()

        telemetry = manager.receive_telemetry(timeout=0.1)

        assert telemetry is not None
        assert telemetry['type'] == TelemetryType.ACK
```

**Step 2: Run test to verify it fails**

Run:
```bash
pytest middleware/tests/test_serial_manager.py::TestSerialManagerSendReceive -v
```

Expected: `AttributeError: 'SerialManager' object has no attribute 'send_command'`

**Step 3: Implement send_command and receive_telemetry**

Add to `serial_manager.py`:

```python
    def send_command(self, cmd_type: int, data: bytes) -> bool:
        """Send command packet to hardware.

        Args:
            cmd_type: Command type (from CommandType enum)
            data: Command data bytes

        Returns:
            True if sent successfully, False otherwise
        """
        if not self._connected or self._serial is None:
            return False

        try:
            packet = encode_packet(cmd_type, data)
            with self._lock:
                self._serial.write(packet)
            return True
        except serial.SerialException:
            self._connected = False
            return False

    def receive_telemetry(self, timeout: float = 0.05) -> Optional[Dict]:
        """Receive and decode telemetry packet.

        Args:
            timeout: Timeout in seconds

        Returns:
            Decoded telemetry dict or None if no packet
        """
        if not self._connected or self._serial is None:
            return None

        try:
            # Read until we get a valid packet
            start_time = time.time()
            buffer = bytearray()

            while time.time() - start_time < timeout:
                if self._serial.in_waiting > 0:
                    byte = self._serial.read(1)
                    if len(byte) == 0:
                        break

                    # Look for header
                    if len(buffer) == 0 and byte[0] != 0xAA:
                        continue

                    buffer.extend(byte)

                    # Check if we have enough for length field
                    if len(buffer) >= 3:
                        expected_len = 4 + buffer[2]
                        if len(buffer) >= expected_len:
                            # Try to decode
                            packet = decode_packet(bytes(buffer))
                            if packet is not None:
                                return packet
                            buffer.clear()

            return None
        except serial.SerialException:
            self._connected = False
            return None
```

**Step 4: Run test to verify it passes**

Run:
```bash
pytest middleware/tests/test_serial_manager.py::TestSerialManagerSendReceive -v
```

Expected: 3 tests PASS

**Step 5: Commit**

```bash
git add simulation/middleware/serial_manager.py simulation/middleware/tests/test_serial_manager.py
git commit -m "feat(middleware): add SerialManager send/receive

- Send command packets (encode via protocol)
- Receive telemetry packets (decode via protocol)
- Timeout handling and error recovery"
```

---

### Task 8: Python Echo Test Script

**Files:**
- Create: `simulation/scripts/test_echo.py`

**Step 1: Create echo test script**

Create `simulation/scripts/test_echo.py`:

```python
"""Echo test: Verify Python ↔ STM32 communication."""

import sys
import time
from middleware.serial_manager import SerialManager
from middleware.protocol import CommandType, TelemetryType


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM3'

    print(f"Echo Test: Testing communication with {port}")
    print("=" * 50)

    # Connect to hardware
    manager = SerialManager(port=port, baud=115200)
    if not manager.connect():
        print(f"ERROR: Failed to connect to {port}")
        return 1

    print(f"✅ Connected to {port}")

    # Send test commands and wait for ACK
    test_count = 10
    success_count = 0

    for i in range(test_count):
        # Send SET_JOINT_ANGLES command with dummy data
        data = bytes([0] * 8)  # Two floats (0.0, 0.0)

        if not manager.send_command(CommandType.SET_JOINT_ANGLES, data):
            print(f"✗ Test {i+1}: Send failed")
            continue

        # Wait for ACK
        telemetry = manager.receive_telemetry(timeout=0.1)

        if telemetry is not None and telemetry['type'] == TelemetryType.ACK:
            print(f"✓ Test {i+1}: Echo successful")
            success_count += 1
        else:
            print(f"✗ Test {i+1}: No ACK received")

        time.sleep(0.1)

    # Disconnect
    manager.disconnect()

    # Summary
    print("=" * 50)
    print(f"Results: {success_count}/{test_count} successful")
    print(f"Success rate: {success_count/test_count*100:.1f}%")

    if success_count == test_count:
        print("✅ Echo test PASSED")
        return 0
    else:
        print("❌ Echo test FAILED")
        return 1


if __name__ == '__main__':
    sys.exit(main())
```

**Step 2: Test with hardware**

Run (with STM32 connected and flashed with echo firmware):
```bash
cd simulation
python scripts/test_echo.py COM3
```

Expected output:
```
Echo Test: Testing communication with COM3
==================================================
✅ Connected to COM3
✓ Test 1: Echo successful
✓ Test 2: Echo successful
...
✓ Test 10: Echo successful
==================================================
Results: 10/10 successful
Success rate: 100.0%
✅ Echo test PASSED
```

**Step 3: Commit**

```bash
git add simulation/scripts/test_echo.py
git commit -m "test(middleware): add echo test script

- Verify Python ↔ STM32 communication
- Send commands, receive ACK
- Measure success rate"
```

---

## Phase 2: Servo Control & HIL Loop (Days 4-6)

### Task 9: Firmware Servo Control Module

**Files:**
- Create: `firmware/Core/Inc/servo_control.h`
- Create: `firmware/Core/Src/servo_control.c`

**Step 1: Create servo_control.h**

Create `firmware/Core/Inc/servo_control.h`:

```c
#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define SERVO_PWM_MIN 1000
#define SERVO_PWM_MAX 2000
#define SERVO_PWM_CENTER 1500

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float current_angle_rad;
    float min_angle_rad;
    float max_angle_rad;
} Servo_t;

void Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel);
void Servo_SetAngle(Servo_t *servo, float angle_rad);
float Servo_GetAngle(Servo_t *servo);

#endif // SERVO_CONTROL_H
```

**Step 2: Implement servo_control.c**

Create `firmware/Core/Src/servo_control.c`:

```c
#include "servo_control.h"
#include <math.h>

static uint16_t angle_to_pwm(float angle_rad) {
    float angle_clamped = fmaxf(-M_PI_2, fminf(M_PI_2, angle_rad));
    float angle_deg = angle_clamped * 180.0f / M_PI;
    float normalized = (angle_deg + 90.0f) / 180.0f;
    uint16_t pwm_us = SERVO_PWM_MIN + (uint16_t)(normalized * (SERVO_PWM_MAX - SERVO_PWM_MIN));
    return pwm_us;
}

void Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel) {
    servo->htim = htim;
    servo->channel = channel;
    servo->current_angle_rad = 0.0f;
    servo->min_angle_rad = -M_PI_2;
    servo->max_angle_rad = M_PI_2;
    HAL_TIM_PWM_Start(htim, channel);
    Servo_SetAngle(servo, 0.0f);
}

void Servo_SetAngle(Servo_t *servo, float angle_rad) {
    float angle_clamped = fmaxf(servo->min_angle_rad, fminf(servo->max_angle_rad, angle_rad));
    uint16_t pwm_us = angle_to_pwm(angle_clamped);
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pwm_us);
    servo->current_angle_rad = angle_clamped;
}

float Servo_GetAngle(Servo_t *servo) {
    return servo->current_angle_rad;
}
```

**Step 3: Build to verify**

Run: Build Project
Expected: 0 errors

**Step 4: Commit**

```bash
git add firmware/Core/Inc/servo_control.h firmware/Core/Src/servo_control.c
git commit -m "feat(firmware): add servo control module

- Angle-to-PWM conversion
- Joint angle clamping [-π/2, π/2]
- PWM output via HAL timers"
```

---

### Task 10: Firmware PID Controller Module

**Files:**
- Create: `firmware/Core/Inc/pid_controller.h`
- Create: `firmware/Core/Src/pid_controller.c`

**Step 1: Create pid_controller.h**

Create `firmware/Core/Inc/pid_controller.h`:

```c
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
    float dt;
} PID_Controller_t;

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float dt);
void PID_SetGains(PID_Controller_t *pid, float Kp, float Ki, float Kd);
void PID_SetSetpoint(PID_Controller_t *pid, float setpoint);
float PID_Update(PID_Controller_t *pid, float measurement);
void PID_Reset(PID_Controller_t *pid);

#endif // PID_CONTROLLER_H
```

**Step 2: Implement pid_controller.c**

Create `firmware/Core/Src/pid_controller.c`:

```c
#include "pid_controller.h"
#include <math.h>

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float dt) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = -M_PI_2;
    pid->output_max = M_PI_2;
}

void PID_SetGains(PID_Controller_t *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_SetSetpoint(PID_Controller_t *pid, float setpoint) {
    pid->setpoint = setpoint;
}

float PID_Update(PID_Controller_t *pid, float measurement) {
    float error = pid->setpoint - measurement;
    float P = pid->Kp * error;
    pid->integral += error * pid->dt;
    float I = pid->Ki * pid->integral;
    float derivative = (error - pid->prev_error) / pid->dt;
    float D = pid->Kd * derivative;
    float output = P + I + D;

    if (output > pid->output_max) {
        output = pid->output_max;
        pid->integral -= error * pid->dt;
    } else if (output < pid->output_min) {
        output = pid->output_min;
        pid->integral -= error * pid->dt;
    }

    pid->prev_error = error;
    return output;
}

void PID_Reset(PID_Controller_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
```

**Step 3: Build to verify**

Run: Build Project
Expected: 0 errors

**Step 4: Commit**

```bash
git add firmware/Core/Inc/pid_controller.h firmware/Core/Src/pid_controller.c
git commit -m "feat(firmware): add PID controller module

- Standard discrete PID algorithm
- Anti-windup with output clamping
- Configurable gains (Kp, Ki, Kd)"
```

---

### Task 11: Firmware IMU Driver Module

**Files:**
- Create: `firmware/Core/Inc/imu_driver.h`
- Create: `firmware/Core/Src/imu_driver.c`

**Step 1: Create imu_driver.h**

Create `firmware/Core/Inc/imu_driver.h`:

```c
#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define MPU6050_ADDR (0x68 << 1)

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} IMU_Data_t;

bool IMU_Init(I2C_HandleTypeDef *hi2c);
bool IMU_ReadData(I2C_HandleTypeDef *hi2c, IMU_Data_t *data);
bool IMU_WhoAmI(I2C_HandleTypeDef *hi2c, uint8_t *device_id);

#endif // IMU_DRIVER_H
```

**Step 2: Implement imu_driver.c**

Create `firmware/Core/Src/imu_driver.c`:

```c
#include "imu_driver.h"

#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_PWR_MGMT_1_REG 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

bool IMU_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t who_am_i;
    if (!IMU_WhoAmI(hi2c, &who_am_i) || who_am_i != 0x68) {
        return false;
    }
    uint8_t pwr_mgmt = 0x00;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR,
                                                  MPU6050_PWR_MGMT_1_REG, 1,
                                                  &pwr_mgmt, 1, 100);
    return (status == HAL_OK);
}

bool IMU_WhoAmI(I2C_HandleTypeDef *hi2c, uint8_t *device_id) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR,
                                                 MPU6050_WHO_AM_I_REG, 1,
                                                 device_id, 1, 100);
    return (status == HAL_OK);
}

bool IMU_ReadData(I2C_HandleTypeDef *hi2c, IMU_Data_t *data) {
    uint8_t buffer[14];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR,
                                                 MPU6050_ACCEL_XOUT_H, 1,
                                                 buffer, 14, 100);
    if (status != HAL_OK) {
        return false;
    }
    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);
    return true;
}
```

**Step 3: Build to verify**

Run: Build Project
Expected: 0 errors

**Step 4: Commit**

```bash
git add firmware/Core/Inc/imu_driver.h firmware/Core/Src/imu_driver.c
git commit -m "feat(firmware): add MPU6050 IMU driver

- I2C communication at 100kHz
- WHO_AM_I verification
- Raw accel/gyro read (6 values)"
```

---

### Task 12: Firmware Main Control Loop

**Files:**
- Modify: `firmware/Core/Src/main.c`
- Modify: `firmware/Core/Inc/main.h`

**Step 1: Add includes to main.c**

Replace the echo test includes with all module includes:

```c
/* USER CODE BEGIN Includes */
#include "uart_handler.h"
#include "protocol.h"
#include "servo_control.h"
#include "pid_controller.h"
#include "imu_driver.h"
#include <string.h>
/* USER CODE END Includes */
```

**Step 2: Add global variables**

Replace echo test globals:

```c
/* USER CODE BEGIN PV */
UART_Handle_t uart_handle;
Servo_t servo_shoulder;
Servo_t servo_elbow;
PID_Controller_t pid_shoulder;
PID_Controller_t pid_elbow;

volatile bool control_loop_flag = false;
bool imu_available = false;
/* USER CODE END PV */
```

**Step 3: Add function prototypes**

```c
/* USER CODE BEGIN PFP */
void Process_UART_Commands(void);
void Handle_Command(Packet_t *packet);
void Run_Control_Loop(void);
void Send_Telemetry_Angles(void);
void Send_Telemetry_Full(void);
void Send_ACK(void);
void Send_Error_Telemetry(void);
/* USER CODE END PFP */
```

**Step 4: Replace main() initialization**

Replace echo test initialization in USER CODE BEGIN 2:

```c
/* USER CODE BEGIN 2 */
// Initialize UART
UART_Init(&uart_handle, &huart2);

// Initialize servos
Servo_Init(&servo_shoulder, &htim2, TIM_CHANNEL_1);
Servo_Init(&servo_elbow, &htim3, TIM_CHANNEL_2);

// Initialize PID controllers (50Hz = 0.02s)
PID_Init(&pid_shoulder, 2.0f, 0.05f, 0.2f, 0.02f);
PID_Init(&pid_elbow, 2.0f, 0.05f, 0.2f, 0.02f);

// Initialize IMU
imu_available = IMU_Init(&hi2c1);

// Set initial targets
PID_SetSetpoint(&pid_shoulder, 0.0f);
PID_SetSetpoint(&pid_elbow, 0.0f);

// LED off
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
/* USER CODE END 2 */
```

**Step 5: Replace main() loop**

Replace echo test loop in USER CODE BEGIN 3:

```c
/* USER CODE BEGIN 3 */
Process_UART_Commands();

if (control_loop_flag) {
    control_loop_flag = false;
    Run_Control_Loop();
}
/* USER CODE END 3 */
```

**Step 6: Add command processing function**

Add before main():

```c
/* USER CODE BEGIN 4 */
void Process_UART_Commands(void) {
    static uint8_t packet_buffer[128];
    static uint16_t buffer_pos = 0;

    while (UART_BytesAvailable(&uart_handle)) {
        uint8_t byte = UART_ReadByte(&uart_handle);

        if (buffer_pos == 0 && byte != PACKET_HEADER) {
            continue;
        }

        packet_buffer[buffer_pos++] = byte;

        if (buffer_pos >= 3) {
            uint8_t expected_len = 4 + packet_buffer[2];
            if (buffer_pos >= expected_len) {
                Packet_t packet;
                if (Protocol_DecodePacket(packet_buffer, buffer_pos, &packet)) {
                    Handle_Command(&packet);
                    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
                } else {
                    Send_Error_Telemetry();
                }
                buffer_pos = 0;
            }
        }

        if (buffer_pos >= sizeof(packet_buffer)) {
            buffer_pos = 0;
        }
    }
}

void Handle_Command(Packet_t *packet) {
    switch (packet->type) {
        case CMD_SET_JOINT_ANGLES: {
            if (packet->length == 8) {
                float shoulder, elbow;
                memcpy(&shoulder, &packet->data[0], 4);
                memcpy(&elbow, &packet->data[4], 4);
                PID_SetSetpoint(&pid_shoulder, shoulder);
                PID_SetSetpoint(&pid_elbow, elbow);
                Send_ACK();
            }
            break;
        }
        case CMD_GET_TELEMETRY: {
            Send_Telemetry_Full();
            break;
        }
        case CMD_SET_PID_GAINS: {
            if (packet->length == 24) {
                float kp1, ki1, kd1, kp2, ki2, kd2;
                memcpy(&kp1, &packet->data[0], 4);
                memcpy(&ki1, &packet->data[4], 4);
                memcpy(&kd1, &packet->data[8], 4);
                memcpy(&kp2, &packet->data[12], 4);
                memcpy(&ki2, &packet->data[16], 4);
                memcpy(&kd2, &packet->data[20], 4);
                PID_SetGains(&pid_shoulder, kp1, ki1, kd1);
                PID_SetGains(&pid_elbow, kp2, ki2, kd2);
                Send_ACK();
            }
            break;
        }
        default:
            Send_Error_Telemetry();
            break;
    }
}

void Run_Control_Loop(void) {
    float current_shoulder = Servo_GetAngle(&servo_shoulder);
    float current_elbow = Servo_GetAngle(&servo_elbow);

    float output_shoulder = PID_Update(&pid_shoulder, current_shoulder);
    float output_elbow = PID_Update(&pid_elbow, current_elbow);

    Servo_SetAngle(&servo_shoulder, output_shoulder);
    Servo_SetAngle(&servo_elbow, output_elbow);

    static uint8_t telemetry_counter = 0;
    if (++telemetry_counter >= 10) {
        Send_Telemetry_Angles();
        telemetry_counter = 0;
    }
}

void Send_Telemetry_Angles(void) {
    Packet_t packet;
    packet.type = TEL_ANGLES_ONLY;
    packet.length = 8;

    float shoulder = Servo_GetAngle(&servo_shoulder);
    float elbow = Servo_GetAngle(&servo_elbow);
    memcpy(&packet.data[0], &shoulder, 4);
    memcpy(&packet.data[4], &elbow, 4);

    uint8_t output[128];
    uint16_t len = Protocol_EncodePacket(&packet, output);
    UART_SendPacket(&uart_handle, output, len);
}

void Send_Telemetry_Full(void) {
    Packet_t packet;
    packet.type = TEL_FULL;
    packet.length = 52;

    uint32_t timestamp = HAL_GetTick();
    memcpy(&packet.data[0], &timestamp, 4);

    float shoulder = Servo_GetAngle(&servo_shoulder);
    float elbow = Servo_GetAngle(&servo_elbow);
    memcpy(&packet.data[4], &shoulder, 4);
    memcpy(&packet.data[8], &elbow, 4);

    float vel_shoulder = 0.0f;
    float vel_elbow = 0.0f;
    memcpy(&packet.data[12], &vel_shoulder, 4);
    memcpy(&packet.data[16], &vel_elbow, 4);

    IMU_Data_t imu_data;
    if (imu_available && IMU_ReadData(&hi2c1, &imu_data)) {
        float accel_x = imu_data.accel_x / 16384.0f;
        float accel_y = imu_data.accel_y / 16384.0f;
        float accel_z = imu_data.accel_z / 16384.0f;
        memcpy(&packet.data[20], &accel_x, 4);
        memcpy(&packet.data[24], &accel_y, 4);
        memcpy(&packet.data[28], &accel_z, 4);

        float gyro_x = imu_data.gyro_x / 131.0f;
        float gyro_y = imu_data.gyro_y / 131.0f;
        float gyro_z = imu_data.gyro_z / 131.0f;
        memcpy(&packet.data[32], &gyro_x, 4);
        memcpy(&packet.data[36], &gyro_y, 4);
        memcpy(&packet.data[40], &gyro_z, 4);
    } else {
        memset(&packet.data[20], 0, 24);
    }

    float roll = 0.0f;
    float pitch = 0.0f;
    memcpy(&packet.data[44], &roll, 4);
    memcpy(&packet.data[48], &pitch, 4);

    uint8_t output[128];
    uint16_t len = Protocol_EncodePacket(&packet, output);
    UART_SendPacket(&uart_handle, output, len);
}

void Send_ACK(void) {
    Packet_t packet;
    packet.type = TEL_ACK;
    packet.length = 0;
    uint8_t output[128];
    uint16_t len = Protocol_EncodePacket(&packet, output);
    UART_SendPacket(&uart_handle, output, len);
}

void Send_Error_Telemetry(void) {
    Packet_t packet;
    packet.type = TEL_ERROR;
    packet.length = 0;
    uint8_t output[128];
    uint16_t len = Protocol_EncodePacket(&packet, output);
    UART_SendPacket(&uart_handle, output, len);
}
/* USER CODE END 4 */
```

**Step 7: Add SysTick configuration**

Add after MX_GPIO_Init() in main():

```c
/* USER CODE BEGIN SysInit */
HAL_SYSTICK_Config(SystemCoreClock / 50);  // 50Hz control loop
/* USER CODE END SysInit */
```

**Step 8: Modify SysTick_Handler**

In `firmware/Core/Src/stm32f4xx_it.c`, modify SysTick_Handler:

```c
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  extern volatile bool control_loop_flag;
  control_loop_flag = true;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}
```

**Step 9: Build firmware**

Run: Build Project
Expected: 0 errors

**Step 10: Flash firmware to board**

Run: Debug (F11)
Expected: Firmware flashed, servos should move to center position

**Step 11: Commit**

```bash
git add firmware/Core/Src/main.c firmware/Core/Inc/main.h firmware/Core/Src/stm32f4xx_it.c
git commit -m "feat(firmware): implement main control loop

- 50Hz control loop with SysTick
- Command processing (SET_JOINT_ANGLES, GET_TELEMETRY, SET_PID_GAINS)
- PID control for both servos
- Telemetry transmission (ANGLES_ONLY, FULL)"
```

---

### Task 13: Python HIL Synchronizer (TDD)

**Files:**
- Create: `simulation/middleware/hil_synchronizer.py`
- Create: `simulation/middleware/tests/test_hil_synchronizer.py`

**Step 1: Write failing test for HILSynchronizer initialization**

Create `simulation/middleware/tests/test_hil_synchronizer.py`:

```python
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
```

**Step 2: Run test to verify it fails**

Run:
```bash
cd simulation
pytest middleware/tests/test_hil_synchronizer.py::TestHILSynchronizerInit -v
```

Expected: `ModuleNotFoundError: No module named 'middleware.hil_synchronizer'`

**Step 3: Implement minimal HILSynchronizer class**

Create `simulation/middleware/hil_synchronizer.py`:

```python
"""Hardware-in-the-Loop synchronizer for MuJoCo simulation."""

import mujoco
import time
import numpy as np
from typing import Optional
from middleware.serial_manager import SerialManager
from middleware.protocol import CommandType, TelemetryType, encode_set_joint_angles


class HILSynchronizer:
    """Synchronizes MuJoCo simulation with physical hardware.

    Commands hardware at specified rate (default 25Hz), receives telemetry,
    and updates simulation state accordingly.
    """

    def __init__(self, port: str, model_path: str, update_rate: float = 25.0):
        """Initialize HIL synchronizer.

        Args:
            port: Serial port for hardware communication
            model_path: Path to MuJoCo MJCF model file
            update_rate: Update frequency in Hz (default: 25.0)
        """
        self.port = port
        self.update_rate = update_rate
        self.period = 1.0 / update_rate

        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Initialize serial manager
        self.serial = SerialManager(port=port)

        self._running = False
```

**Step 4: Run test to verify it passes**

Run:
```bash
pytest middleware/tests/test_hil_synchronizer.py::TestHILSynchronizerInit -v
```

Expected: 1 test PASS

**Step 5: Commit**

```bash
git add simulation/middleware/hil_synchronizer.py simulation/middleware/tests/test_hil_synchronizer.py
git commit -m "feat(middleware): add HILSynchronizer initialization

- Load MuJoCo model
- Initialize serial manager
- Store update rate and period"
```

---

### Task 14: Python HIL Synchronizer Control Loop

**Files:**
- Modify: `simulation/middleware/hil_synchronizer.py`

**Step 1: Add start/stop methods and control loop**

Add to `hil_synchronizer.py`:

```python
    def start(self) -> None:
        """Start HIL loop (blocking).

        Connects to hardware, enters control loop, and runs until stopped.
        """
        # Connect to hardware
        if not self.serial.connect():
            raise RuntimeError(f"Failed to connect to {self.port}")

        print(f"✅ Connected to {self.port}")
        print(f"HIL Loop: {self.update_rate}Hz ({self.period*1000:.1f}ms period)")
        print("Press Ctrl+C to stop")

        self._running = True

        try:
            while self._running:
                loop_start = time.time()

                # Command hardware
                self._send_commands()

                # Receive telemetry
                telemetry = self.serial.receive_telemetry(timeout=0.02)
                if telemetry is not None:
                    self._process_telemetry(telemetry)

                # Step physics
                mujoco.mj_step(self.model, self.data)

                # Maintain update rate
                elapsed = time.time() - loop_start
                if elapsed < self.period:
                    time.sleep(self.period - elapsed)
                else:
                    print(f"⚠️  Loop overrun: {elapsed*1000:.1f}ms > {self.period*1000:.1f}ms")

        except KeyboardInterrupt:
            print("\n⏹️  Stopped by user")
        finally:
            self.stop()

    def stop(self) -> None:
        """Stop HIL loop and disconnect."""
        self._running = False
        self.serial.disconnect()
        print("✅ Disconnected")

    def _send_commands(self) -> None:
        """Send joint angle commands to hardware."""
        # Get commanded angles from MuJoCo actuators
        shoulder = self.data.ctrl[0]  # Actuator 0: shoulder
        elbow = self.data.ctrl[1]     # Actuator 1: elbow

        # Encode and send
        packet = encode_set_joint_angles(shoulder, elbow)
        self.serial._serial.write(packet)

    def _process_telemetry(self, telemetry: dict) -> None:
        """Update MuJoCo state from hardware telemetry."""
        if telemetry['type'] == TelemetryType.ANGLES_ONLY:
            # Update joint positions in MuJoCo
            angles = telemetry['data']
            if len(angles) >= 8:
                import struct
                shoulder, elbow = struct.unpack('<ff', angles[:8])
                self.data.qpos[0] = shoulder
                self.data.qpos[1] = elbow

        elif telemetry['type'] == TelemetryType.FULL:
            # TODO: Process full telemetry (IMU data, velocities)
            pass
```

**Step 2: Create simple test script**

Create `simulation/scripts/test_hil_basic.py`:

```python
"""Basic HIL test: Manual servo control."""

import sys
from middleware.hil_synchronizer import HILSynchronizer


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM3'
    model_path = 'mujoco_model/arm.xml'

    print("Basic HIL Test")
    print("=" * 50)

    hil = HILSynchronizer(port=port, model_path=model_path, update_rate=25.0)

    # Set test angles
    hil.data.ctrl[0] = 0.5   # Shoulder: 30 degrees
    hil.data.ctrl[1] = -0.3  # Elbow: -17 degrees

    # Start loop
    hil.start()


if __name__ == '__main__':
    main()
```

**Step 3: Test with hardware**

Run (with STM32 connected and servos powered):
```bash
cd simulation
python scripts/test_hil_basic.py COM3
```

Expected: Servos move to commanded positions, telemetry prints to console

**Step 4: Commit**

```bash
git add simulation/middleware/hil_synchronizer.py simulation/scripts/test_hil_basic.py
git commit -m "feat(middleware): implement HIL control loop

- 25Hz command/telemetry loop
- Send joint angles to hardware
- Receive and process telemetry
- MuJoCo physics step integration"
```

---

## Final Verification

### Task 15: Integration Testing & Success Validation

**Files:**
- Create: `docs/plans/2026-01-20-week2-completion-checklist.md`
- Create: `simulation/scripts/test_servo_sweep.py`

**Step 1: Create servo sweep test**

Create `simulation/scripts/test_servo_sweep.py`:

```python
"""Servo sweep test: Verify smooth motion."""

import sys
import time
import math
from middleware.serial_manager import SerialManager
from middleware.protocol import encode_set_joint_angles


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM3'

    print("Servo Sweep Test")
    print("=" * 50)

    manager = SerialManager(port=port, baud=115200)
    if not manager.connect():
        print(f"ERROR: Failed to connect to {port}")
        return 1

    print(f"✅ Connected to {port}")
    print("Sweeping shoulder: 0° → 90° → -90° → 0°")

    # Sweep shoulder
    duration = 10.0  # seconds
    steps = 100
    dt = duration / steps

    for i in range(steps):
        t = i / steps
        # Sine wave: 0 → π/2 → 0 → -π/2 → 0
        angle = (math.pi / 2) * math.sin(2 * math.pi * t)

        packet = encode_set_joint_angles(angle, 0.0)
        manager._serial.write(packet)

        print(f"\r[{i+1}/{steps}] Angle: {math.degrees(angle):+6.1f}°", end='')
        time.sleep(dt)

    print("\n✅ Sweep complete")
    manager.disconnect()
    return 0


if __name__ == '__main__':
    sys.exit(main())
```

**Step 2: Run sweep test**

Run:
```bash
cd simulation
python scripts/test_servo_sweep.py COM3
```

Expected: Shoulder servo sweeps smoothly through full range

**Step 3: Create completion checklist**

Create `docs/plans/2026-01-20-week2-completion-checklist.md`:

```markdown
# Week 2 Completion Checklist

**Date**: January 20, 2026
**Status**: [To be filled by executor]

---

## Success Criteria Validation

### 1. Communication Working ✅
- [ ] Python sends command packet
- [ ] STM32 receives, validates CRC
- [ ] STM32 sends ACK/telemetry packet
- [ ] Python receives and decodes packet
- [ ] Measured latency <10ms one-way
- [ ] 0% packet loss over 100 commands
- [ ] Echo test passes 100%

**Test Command**:
```bash
cd simulation
python scripts/test_echo.py COM3
```

Expected: 10/10 successful, 100% success rate

---

### 2. Servos Responding ✅
- [ ] Command shoulder to 45°, observe movement
- [ ] Command elbow to -30°, observe movement
- [ ] Angles accurate within 5° (visual/protractor)
- [ ] No excessive jitter or instability
- [ ] Sweep test shows smooth motion

**Test Commands**:
```bash
python scripts/test_servo_sweep.py COM3
```

Expected: Smooth sine wave motion, no jitter

---

### 3. Basic HIL Loop ✅
- [ ] MuJoCo simulation runs
- [ ] Simulation commands hardware @ 25Hz
- [ ] Hardware moves servos based on commands
- [ ] Hardware sends telemetry back
- [ ] MuJoCo visualization updates from telemetry
- [ ] Measured HIL loop latency <50ms

**Test Command**:
```bash
python scripts/test_hil_basic.py COM3
```

Expected: Servos track MuJoCo actuator commands, no loop overruns

---

### 4. IMU Data Streaming ✅
- [ ] MPU6050 responds to WHO_AM_I (0x68)
- [ ] Raw accel/gyro data readable via I2C
- [ ] Data included in FULL telemetry packet
- [ ] Python decodes and displays IMU data
- [ ] Data changes when board is moved/rotated

**Test**: Request FULL telemetry, inspect IMU fields

---

## Test Results Summary

**Total Tests**: [X]
- ✅ **Passing**: [X]
- ❌ **Failing**: [X]

**Performance Metrics**:
- UART latency: [X]ms
- HIL loop latency: [X]ms
- Packet loss rate: [X]%
- Servo position accuracy: [X]°

---

## Code Quality

**Firmware**:
- [ ] All modules build without errors
- [ ] Code follows STM32 HAL conventions
- [ ] No memory leaks (static allocation only)
- [ ] Control loop runs at 50Hz consistently

**Python**:
- [ ] SerialManager tests pass (X/X)
- [ ] HILSynchronizer tests pass (X/X)
- [ ] Integration with protocol.py works
- [ ] No errors in test scripts

---

## Documentation

- [ ] Firmware code documented (comments)
- [ ] Python modules have docstrings
- [ ] Test scripts have usage instructions
- [ ] Week 2 completion checklist filled

---

## Known Issues

[Document any issues encountered and their workarounds]

---

## Week 2 Goals Achievement

| Goal | Status | Notes |
|------|--------|-------|
| Communication working | ✅/❌ | |
| Servos responding | ✅/❌ | |
| Basic HIL loop | ✅/❌ | |
| IMU data streaming | ✅/❌ | |

---

**Week 2 Status**: [COMPLETE / INCOMPLETE]

If all 4 success criteria pass: **✅ READY FOR WEEK 3**
If any criteria fail: **❌ REQUIRES DEBUGGING**
```

**Step 4: Run all tests and fill checklist**

Execute all test commands, measure performance, document results.

**Step 5: Commit completion checklist**

```bash
git add docs/plans/2026-01-20-week2-completion-checklist.md simulation/scripts/test_servo_sweep.py
git commit -m "docs(plans): add Week 2 completion checklist and sweep test

- Validation checklist for 4 success criteria
- Servo sweep test for smooth motion verification
- Performance metrics tracking"
```

---

## Execution Notes

### TDD Discipline
- Python modules: Strict TDD (write test → fail → implement → pass)
- Firmware modules: No TDD (C testing infrastructure complex, use hardware verification)
- Integration: Manual testing with hardware

### Common Pitfalls
1. **Forgetting to flash firmware**: After each firmware build, flash to STM32
2. **Servo power**: Servos MUST be powered by external 5V supply, NOT STM32
3. **Serial port selection**: Use Device Manager (Windows) to find correct COM port
4. **MuJoCo viewer**: May not work on Python 3.14, use 3.11/3.12 if needed
5. **PID tuning**: Start conservative, tune slowly, document final gains

### Estimated Time
- Task 1 (STM32CubeIDE): ~1 hour (GUI work)
- Tasks 2-4 (Firmware communication): ~2 hours
- Tasks 5-8 (Python serial manager + echo test): ~2 hours
- Tasks 9-12 (Firmware servo/PID/IMU/main): ~3 hours
- Tasks 13-14 (HIL synchronizer): ~2 hours
- Task 15 (Integration testing): ~2 hours
- **Total**: ~12 hours (2 hours/day × 6 days)

---

## Next Steps (Week 3)

**Focus**:
- Sensor fusion (complementary filter for IMU)
- Trajectory generation (smooth multi-point motion)
- Visualization dashboard (PyQt5 with real-time plots)
- PID tuning optimization (systematic methods)
- Error recovery (watchdog, safe mode)

---

**Plan Status**: ✅ Ready for execution
**Saved to**: `docs/plans/2026-01-20-week2-hil-loop.md`
