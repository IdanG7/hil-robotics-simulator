# Week 2 Design: HIL Communication Loop

**Goal**: Establish bidirectional communication between MuJoCo simulation and STM32 hardware, implement servo control with PID, and demonstrate the first hardware-in-the-loop synchronization.

**Timeline**: January 14-20, 2026 (6 days)
**Phase**: HIL Loop Foundation

---

## Executive Summary

Week 2 builds on the foundation from Week 1 (protocol, kinematics, MuJoCo model) by implementing the critical hardware-software interface. By the end of Week 2, the system will demonstrate a complete HIL loop: MuJoCo simulation commands hardware, servos move, telemetry streams back, and visualization updates in real-time.

**Key Deliverables**:
1. Bidirectional UART communication (Python ↔ STM32)
2. Structured firmware with PID control @ 50Hz
3. Servo position control with basic tuning
4. Automatic HIL synchronization @ 25Hz
5. MPU6050 raw data streaming

**Success Criteria** (all must pass):
- ✅ Communication working: Reliable packet transmission with CRC validation
- ✅ Servos responding: Physical movement matches commanded angles
- ✅ Basic HIL loop: End-to-end MuJoCo → Hardware → Telemetry → Visualization
- ✅ IMU data streaming: Raw accelerometer/gyroscope data in telemetry

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    PYTHON SIMULATION (25Hz)                      │
│                                                                   │
│   ┌──────────────┐         ┌────────────────────┐              │
│   │   MuJoCo     │◄───────►│  HIL Synchronizer  │              │
│   │  Simulation  │  State  │  (hil_sync.py)     │              │
│   │              │ Update  │                     │              │
│   └──────────────┘         └─────────┬──────────┘              │
│                                      │                           │
│                                      │ Commands (25Hz)           │
│                                      ▼                           │
│   ┌─────────────────────────────────────────────────┐          │
│   │         Serial Manager (serial_manager.py)       │          │
│   │  • Auto-reconnection                             │          │
│   │  • Packet encode/decode (protocol.py)           │          │
│   │  • Timeout detection                             │          │
│   └──────────────────┬──────────────────────────────┘          │
│                      │ USB-Serial @ 115200 baud                 │
└──────────────────────┼──────────────────────────────────────────┘
                       │
                       │ UART (PA2/PA3)
                       ▼
┌─────────────────────────────────────────────────────────────────┐
│              STM32 FIRMWARE (50Hz Control Loop)                  │
│                                                                   │
│   ┌──────────────┐      ┌─────────────────┐                    │
│   │ UART Handler │◄────►│ Protocol Parser │                    │
│   │ (DMA RX/TX)  │      │ (CRC validation)│                    │
│   └──────────────┘      └────────┬────────┘                    │
│                                  │                               │
│                        Commands  │  Telemetry                   │
│                                  ▼                               │
│   ┌────────────────────────────────────────────┐               │
│   │         Main Control Loop (50Hz)            │               │
│   │  • Parse commands                           │               │
│   │  • Run PID controllers (2x servos)         │               │
│   │  • Update PWM outputs                       │               │
│   │  • Read IMU (optional this iteration)      │               │
│   │  • Build telemetry packet                   │               │
│   └──┬─────────────┬─────────────────┬─────────┘               │
│      │             │                  │                          │
│      │ PWM         │ PWM              │ I2C                     │
│      ▼             ▼                  ▼                          │
│  ┌────────┐   ┌────────┐      ┌──────────────┐                │
│  │ Servo1 │   │ Servo2 │      │   MPU6050    │                │
│  │ (TIM2) │   │ (TIM3) │      │     IMU      │                │
│  └────────┘   └────────┘      └──────────────┘                │
│   Shoulder       Elbow          Accel/Gyro                      │
└─────────────────────────────────────────────────────────────────┘
```

---

## Phase Breakdown

### Days 1-3: Communication Foundation

**Goal**: Rock-solid Python ↔ STM32 communication

**Firmware Tasks**:
1. UART handler with DMA (uart_handler.c)
   - DMA circular buffer for RX
   - DMA normal mode for TX
   - Interrupt-driven receive notification

2. Protocol parser (protocol.c)
   - CRC-8 validation (matches Python)
   - Packet decode/encode
   - Command dispatch

3. Echo test firmware
   - Receive command → validate CRC → echo back
   - LED toggle on valid packet
   - Error indication on CRC fail

**Python Tasks**:
1. Serial manager (serial_manager.py)
   - pySerial wrapper
   - Send/receive with timeout
   - Automatic reconnection on disconnect
   - Thread-safe operation

2. Integration tests
   - Echo test: Python → STM32 → Python
   - CRC validation test
   - Latency measurement
   - Stress test (rapid commands)

**Milestone**: Reliable bidirectional communication verified with echo test

---

### Days 4-6: Servo Control & HIL Loop

**Goal**: Physical servo movement synchronized with simulation

**Firmware Tasks**:
1. Servo control (servo_control.c)
   - Angle-to-PWM conversion (0° = 1ms, 180° = 2ms, centered at 1.5ms)
   - Joint angle limits [-π/2, π/2]
   - PWM update via HAL_TIM_PWM_SetCompare

2. PID controller (pid_controller.c)
   - Structure: Kp, Ki, Kd gains + state
   - 50Hz update rate (20ms timestep)
   - Output: PWM duty cycle
   - Manual tuning: start with Kp=2.0, Ki=0.05, Kd=0.2

3. Main control loop (main.c)
   - 50Hz timer interrupt (SysTick or TIM)
   - Process commands from protocol
   - Update PID controllers
   - Send telemetry (ANGLES_ONLY packet)

4. IMU driver (imu_driver.c - optional Day 6)
   - I2C initialization (100kHz)
   - MPU6050 WHO_AM_I verification
   - Raw accel/gyro read (6 values)
   - Included in FULL telemetry packet

**Python Tasks**:
1. HIL synchronizer (hil_synchronizer.py)
   - Load MuJoCo model
   - Command servos @ 25Hz (40ms period)
   - Receive telemetry
   - Update MuJoCo visualization
   - Latency tracking (command → telemetry round-trip)

2. Simple test scripts
   - Manual servo command (test_servo_command.py)
   - Sweep test: 0° → 90° → -90° → 0°
   - Step response: measure overshoot, settling time

**Milestone**: Servos track commanded angles, visible in MuJoCo visualization

---

## Technical Specifications

### Timing Budget

| Component | Rate | Period | Latency Budget |
|-----------|------|--------|----------------|
| Firmware control loop | 50Hz | 20ms | - |
| HIL synchronizer | 25Hz | 40ms | <50ms total |
| UART transmission (64-byte packet) | - | ~5.5ms @ 115200 | - |
| Python processing | - | ~2ms | - |
| **Total HIL loop** | - | - | **<50ms target** |

**Latency breakdown**:
- Python command encode: ~0.5ms
- UART TX: ~5.5ms
- STM32 process + PID: <20ms (one control cycle)
- UART RX (telemetry): ~5.5ms
- Python decode + update: ~2ms
- **Total**: ~33.5ms (within 50ms budget)

### Memory Budget (STM32F446RE)

| Resource | Allocated | Available | Usage |
|----------|-----------|-----------|-------|
| Flash (code) | ~20KB | 512KB | 4% |
| RAM (globals) | ~4KB | 128KB | 3% |
| Stack | ~2KB | - | - |
| Heap | 0 (no malloc) | - | - |

**Flash breakdown**:
- HAL drivers: ~10KB
- Custom code: ~10KB (uart_handler, protocol, servo, pid, imu)
- Constants/strings: ~1KB

---

## Module Design

### Firmware Modules

#### 1. uart_handler.c/h (UART Communication)

```c
typedef struct {
    uint8_t rx_buffer[256];  // DMA circular buffer
    uint8_t tx_buffer[128];  // TX staging buffer
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile bool tx_busy;
} UART_Handle_t;

void UART_Init(UART_Handle_t *handle);
bool UART_BytesAvailable(UART_Handle_t *handle);
uint8_t UART_ReadByte(UART_Handle_t *handle);
bool UART_SendPacket(UART_Handle_t *handle, uint8_t *data, uint16_t len);
```

**Key features**:
- DMA circular buffer for RX (handles bursts)
- Non-blocking TX with busy flag
- Interrupt-driven notifications

#### 2. protocol.c/h (Binary Protocol)

```c
typedef enum {
    CMD_SET_JOINT_ANGLES = 0x10,
    CMD_GET_TELEMETRY = 0x20,
    CMD_SET_PID_GAINS = 0x40,
    // ... (matches Python protocol.py)
} CommandType_t;

typedef struct {
    uint8_t header;   // 0xAA
    uint8_t type;     // CommandType_t
    uint8_t length;   // Payload bytes
    uint8_t data[64]; // Payload
    uint8_t crc;      // CRC-8
} Packet_t;

bool Protocol_DecodePacket(uint8_t *raw, Packet_t *packet);
void Protocol_EncodePacket(Packet_t *packet, uint8_t *output);
uint8_t Protocol_CRC8(uint8_t *data, uint16_t len);
```

**Key features**:
- Exact CRC-8 match with Python (polynomial 0x07)
- Static buffers (no dynamic allocation)
- Defensive validation (length checks, header verify)

#### 3. servo_control.c/h (PWM Servo Control)

```c
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float current_angle_rad;
    float min_angle_rad;   // -π/2
    float max_angle_rad;   //  π/2
    uint16_t pwm_min;      // 1000 (1ms)
    uint16_t pwm_max;      // 2000 (2ms)
    uint16_t pwm_center;   // 1500 (1.5ms)
} Servo_t;

void Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel);
void Servo_SetAngle(Servo_t *servo, float angle_rad);
float Servo_GetAngle(Servo_t *servo);
uint16_t Servo_AngleToPWM(float angle_rad);
```

**Key features**:
- Angle limits enforced (clamping)
- Linear PWM mapping: angle ∈ [-π/2, π/2] → PWM ∈ [1000, 2000]
- Current angle tracking for feedback

#### 4. pid_controller.c/h (PID Control)

```c
typedef struct {
    float Kp, Ki, Kd;
    float setpoint;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
    float dt;  // 0.02 (50Hz)
} PID_Controller_t;

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float dt);
float PID_Update(PID_Controller_t *pid, float measurement);
void PID_SetGains(PID_Controller_t *pid, float Kp, float Ki, float Kd);
void PID_Reset(PID_Controller_t *pid);
```

**Key features**:
- Standard PID algorithm: u = Kp*e + Ki*∫e + Kd*de/dt
- Integral anti-windup (output clamping)
- Manual tuning starting point: Kp=2.0, Ki=0.05, Kd=0.2

#### 5. imu_driver.c/h (MPU6050 I2C)

```c
typedef struct {
    int16_t accel_x, accel_y, accel_z;  // Raw 16-bit
    int16_t gyro_x, gyro_y, gyro_z;     // Raw 16-bit
} IMU_Data_t;

bool IMU_Init(void);
bool IMU_ReadData(IMU_Data_t *data);
bool IMU_WhoAmI(uint8_t *device_id);  // Should return 0x68
```

**Key features**:
- Simple polled I2C (HAL_I2C_Mem_Read)
- No sensor fusion (raw values only)
- WHO_AM_I check on init
- ~4ms read time @ 100kHz I2C

### Python Modules

#### 1. serial_manager.py (Serial Communication)

```python
class SerialManager:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.1):
        """Initialize serial connection with auto-reconnect."""

    def connect(self) -> bool:
        """Connect to serial port."""

    def disconnect(self) -> None:
        """Close serial connection."""

    def send_command(self, cmd_type: int, data: bytes) -> bool:
        """Send command packet to hardware."""

    def receive_telemetry(self, timeout: float = 0.05) -> Optional[Dict]:
        """Receive and decode telemetry packet."""

    def is_connected(self) -> bool:
        """Check if connection is alive."""
```

**Key features**:
- Automatic reconnection on timeout
- Thread-safe (lock for send/receive)
- Uses protocol.py for encode/decode
- Configurable timeouts

#### 2. hil_synchronizer.py (Hardware-in-Loop)

```python
class HILSynchronizer:
    def __init__(self, port: str, model_path: str, update_rate: float = 25.0):
        """Initialize HIL loop with MuJoCo model."""

    def start(self) -> None:
        """Start HIL loop (blocking)."""

    def stop(self) -> None:
        """Stop HIL loop gracefully."""

    def _control_loop(self) -> None:
        """Main loop: command hardware, receive telemetry, update sim."""

    def _send_commands(self, shoulder: float, elbow: float) -> None:
        """Send joint angle commands to hardware."""

    def _process_telemetry(self, telemetry: Dict) -> None:
        """Update MuJoCo state from hardware telemetry."""
```

**Key features**:
- 25Hz update rate (40ms period)
- MuJoCo integration (set actuators, step physics)
- Latency tracking
- Graceful shutdown

---

## Control System Design

### PID Controller Theory

Standard discrete PID:

```
e[k] = setpoint - measurement
P = Kp * e[k]
I = I + Ki * e[k] * dt
D = Kd * (e[k] - e[k-1]) / dt
output = P + I + D (clamped to [output_min, output_max])
```

### Initial PID Gains (Manual Tuning)

**Starting values** (conservative):
- **Kp = 2.0**: Proportional gain (primary control)
- **Ki = 0.05**: Integral gain (eliminate steady-state error)
- **Kd = 0.2**: Derivative gain (reduce overshoot)

**Tuning procedure** (manual):
1. Set Ki=0, Kd=0, start with Kp=1.0
2. Increase Kp until servo oscillates
3. Reduce Kp by 30-40%
4. Add Ki=0.05 to eliminate steady-state error
5. Add Kd=0.2 if overshoot is excessive
6. Fine-tune by observing step response

**Expected behavior** (SG90 servos):
- Rise time: <200ms
- Overshoot: <10%
- Settling time: <500ms
- Steady-state error: <2° (before adding Ki)

### PWM Servo Mapping

SG90 servo specifications:
- **Pulse width range**: 1000µs (0°) to 2000µs (180°)
- **Center**: 1500µs (90°)
- **PWM frequency**: 50Hz (20ms period)

**Angle-to-PWM formula**:
```c
// angle_rad ∈ [-π/2, π/2] (clamped)
// PWM ∈ [1000, 2000] µs
float angle_deg = angle_rad * 180.0 / M_PI;  // Convert to degrees
angle_deg = fmax(-90.0, fmin(90.0, angle_deg));  // Clamp
float normalized = (angle_deg + 90.0) / 180.0;  // Map to [0, 1]
uint16_t pwm_us = 1000 + (uint16_t)(normalized * 1000.0);  // Map to [1000, 2000]
```

---

## Testing Strategy

### Automated Tests (Python)

**serial_manager unit tests** (test_serial_manager.py):
- Mock serial port (unittest.mock)
- Test connect/disconnect
- Test send_command encoding
- Test receive_telemetry decoding
- Test reconnection logic
- Test timeout handling

**protocol integration tests** (existing test_protocol.py):
- Already have 12 tests @ 93% coverage
- No changes needed (protocol is stable)

### Hardware Verification Tests (Manual)

**Day 1-2: Communication tests**:
1. **Echo test**: Python → STM32 → Python
   - Send SET_JOINT_ANGLES command
   - Verify CRC validation on STM32 (LED toggle)
   - Receive ACK packet
   - Measure latency (target <10ms one-way)

2. **Stress test**: Rapid command burst
   - Send 100 commands in 1 second
   - Verify 0 packet loss
   - Check CRC error rate

**Day 3-4: Servo tests**:
1. **Manual command test**: test_servo_command.py
   - Command shoulder to 45°, observe movement
   - Command elbow to -30°, observe movement
   - Verify angles with protractor/visual inspection

2. **Sweep test**: test_servo_sweep.py
   - Sweep shoulder: 0° → 90° → -90° → 0° over 10 seconds
   - Repeat for elbow
   - Verify smooth motion, no jitter

3. **Step response test**: test_servo_step.py
   - Command step change: 0° → 90°
   - Measure rise time, overshoot, settling time
   - Tune PID gains if needed

**Day 5-6: HIL loop tests**:
1. **Basic HIL test**: hil_synchronizer.py
   - Start loop, observe MuJoCo visualization updates
   - Manually move servos, verify telemetry updates sim
   - Check latency display (<50ms target)

2. **IMU test** (if time): test_imu_read.py
   - Read raw IMU data
   - Shake board, verify gyro changes
   - Rotate board, verify accel changes

### Oscilloscope Verification (Optional but Recommended)

**PWM signals**:
- Probe PA0 (shoulder PWM): Verify 50Hz, pulse width 1000-2000µs
- Probe PA1 (elbow PWM): Verify 50Hz, pulse width 1000-2000µs
- Command 0°, 90°, -90°, verify pulse widths: 1500µs, 2000µs, 1000µs

**UART signals**:
- Probe PA2 (TX): Verify 115200 baud, packet structure [0xAA][TYPE][LEN][DATA][CRC]
- Decode packet manually, verify CRC

---

## Error Handling & Robustness

### Firmware Error Handling

**CRC validation failure**:
- Action: Ignore packet, send ERROR telemetry (0xF0)
- LED indication: Red LED blink pattern
- Recovery: Wait for next valid packet

**Invalid command type**:
- Action: Ignore packet, send ERROR telemetry
- Recovery: Continue normal operation

**Servo angle out of range**:
- Action: Clamp to [-π/2, π/2], log warning
- Recovery: Continue with clamped value

**Watchdog timeout** (future Week 3):
- Not implemented in Week 2
- Plan: 500ms watchdog, reset servos to neutral on timeout

### Python Error Handling

**Serial port disconnect**:
- Detection: Timeout on receive (>100ms)
- Action: Close port, attempt reconnection every 1 second
- Recovery: Reconnect when port available, resume operation

**Packet decode failure**:
- Action: Log warning, discard packet
- Recovery: Wait for next packet

**MuJoCo simulation error**:
- Action: Stop HIL loop, display error message
- Recovery: User restarts script

---

## Performance Targets

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| UART latency (one-way) | <10ms | Timestamp commands, measure ACK |
| HIL loop latency (round-trip) | <50ms | Command timestamp → telemetry timestamp |
| Packet loss rate | <0.1% | Count sent vs. received over 1000 packets |
| Control loop jitter | <2ms | Measure SysTick interrupt timing |
| Servo position accuracy | <5° | Protractor or angle sensor |
| PID settling time | <500ms | Step response test |

---

## File Structure

```
Robot/
├── firmware/
│   ├── Core/
│   │   ├── Src/
│   │   │   ├── main.c                 # Main control loop, initialization
│   │   │   ├── uart_handler.c         # DMA UART communication
│   │   │   ├── protocol.c             # Binary protocol parser
│   │   │   ├── servo_control.c        # PWM servo control
│   │   │   ├── pid_controller.c       # PID algorithm
│   │   │   └── imu_driver.c           # MPU6050 I2C driver
│   │   └── Inc/
│   │       ├── uart_handler.h
│   │       ├── protocol.h
│   │       ├── servo_control.h
│   │       ├── pid_controller.h
│   │       └── imu_driver.h
│   └── README.md                      # Build/flash instructions
│
├── simulation/
│   ├── middleware/
│   │   ├── serial_manager.py          # Serial communication wrapper
│   │   ├── hil_synchronizer.py        # Hardware-in-loop controller
│   │   ├── protocol.py                # (existing, no changes)
│   │   ├── kinematics.py              # (existing, no changes)
│   │   └── tests/
│   │       ├── test_serial_manager.py # Unit tests for serial_manager
│   │       └── test_protocol.py       # (existing, 12 tests)
│   │
│   ├── scripts/
│   │   ├── test_servo_command.py      # Manual servo test
│   │   ├── test_servo_sweep.py        # Sweep test
│   │   ├── test_servo_step.py         # Step response
│   │   └── test_imu_read.py           # IMU verification
│   │
│   └── mujoco_model/
│       └── arm.xml                    # (existing, no changes)
│
└── docs/
    ├── WEEK2_DESIGN.md                # This document
    ├── WEEK2_FIRMWARE_GUIDE.md        # Detailed firmware implementation
    └── plans/
        └── 2026-01-20-week2-hil-loop.md  # Implementation plan (TBD)
```

---

## Risk Assessment

### High Risk (must mitigate)

1. **STM32CubeIDE project creation fails**
   - Mitigation: Follow Week 1 setup instructions carefully
   - Backup: Provide pre-configured .ioc file
   - Timeline impact: Could delay Day 1 by 2-4 hours

2. **UART DMA not working**
   - Mitigation: Test with polled UART first, add DMA after basic communication works
   - Backup: Use interrupt-driven RX instead of DMA
   - Timeline impact: 4-6 hours debugging

3. **Servo jitter/instability**
   - Mitigation: Start with conservative PID gains, tune slowly
   - Backup: Use open-loop PWM control (no PID) if tuning fails
   - Timeline impact: Could push IMU to Week 3

### Medium Risk

4. **Python-STM32 communication unreliable**
   - Mitigation: Add CRC validation, timeout detection, automatic reconnection
   - Timeline impact: Extra 2-3 hours for robustness

5. **MuJoCo visualization lag**
   - Mitigation: Reduce HIL update rate from 25Hz → 10Hz
   - Timeline impact: Minimal (performance vs. responsiveness trade-off)

6. **I2C communication with MPU6050 fails**
   - Mitigation: IMU is optional for Week 2, can defer to Week 3
   - Timeline impact: No impact (IMU is stretch goal)

### Low Risk

7. **PID tuning takes longer than expected**
   - Mitigation: Document manual tuning process, acceptable to have non-optimal gains
   - Timeline impact: 1-2 hours extra

---

## Success Validation

**Week 2 is complete when all criteria pass**:

### 1. Communication Working ✅
- [ ] Python sends command packet
- [ ] STM32 receives, validates CRC
- [ ] STM32 sends ACK packet
- [ ] Python receives and decodes ACK
- [ ] Measured latency <10ms one-way
- [ ] 0% packet loss over 100 commands

### 2. Servos Responding ✅
- [ ] Command shoulder to 45°, observe physical movement
- [ ] Command elbow to -30°, observe physical movement
- [ ] Angles accurate within 5° (visual/protractor)
- [ ] No excessive jitter or instability
- [ ] PWM signals verified on oscilloscope (optional)

### 3. Basic HIL Loop ✅
- [ ] MuJoCo simulation runs
- [ ] Simulation commands hardware @ 25Hz
- [ ] Hardware moves servos based on commands
- [ ] Hardware sends telemetry back
- [ ] MuJoCo visualization updates from telemetry
- [ ] Measured HIL loop latency <50ms

### 4. IMU Data Streaming ✅
- [ ] MPU6050 responds to WHO_AM_I (0x68)
- [ ] Raw accel/gyro data readable via I2C
- [ ] Data included in FULL telemetry packet
- [ ] Python decodes and displays IMU data
- [ ] Data changes when board is moved/rotated

---

## Timeline & Milestones

| Day | Focus | Milestone |
|-----|-------|-----------|
| **Day 1** (Jan 14) | STM32CubeIDE project, UART DMA | Project builds, LED blinks |
| **Day 2** (Jan 15) | Protocol parser, echo test | Python ↔ STM32 echo verified |
| **Day 3** (Jan 16) | Python serial_manager, tests | Reliable communication + reconnection |
| **Day 4** (Jan 17) | Servo control, PWM output | Servos respond to manual commands |
| **Day 5** (Jan 18) | PID controller, HIL sync | Basic HIL loop working |
| **Day 6** (Jan 19) | IMU driver, final integration | All 4 success criteria met |

**Buffer**: Day 7 (Jan 20) for debugging, documentation, video recording

---

## Next Steps (After Week 2)

**Week 3 Focus** (if Week 2 succeeds):
- Sensor fusion (complementary filter)
- Trajectory generation (smooth multi-point motion)
- Visualization dashboard (PyQt5 with real-time plots)
- Error recovery (watchdog, safe mode)

**Week 4 Focus**:
- Performance optimization
- Demo video production
- Technical documentation polish
- Portfolio presentation

---

## References

**Week 1 Outputs**:
- `docs/DESIGN.md` - System architecture
- `docs/PROTOCOL.md` - Binary protocol specification
- `simulation/middleware/protocol.py` - Python protocol (93% coverage)
- `simulation/mujoco_model/arm.xml` - MuJoCo model

**External References**:
- STM32F446RE Reference Manual (RM0390)
- STM32 HAL Driver Documentation
- MPU6050 Register Map and Descriptions
- MuJoCo Python Bindings Documentation
- PID Control Tutorial (Brett Beauregard)

---

**Document Status**: ✅ Ready for implementation planning
**Created**: January 14, 2026
**Author**: Claude Code (Brainstorming Phase)
**Next Step**: Create detailed Week 2 implementation plan with TDD approach
