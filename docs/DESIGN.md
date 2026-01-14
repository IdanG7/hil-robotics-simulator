# HIL Robotics Simulator - Comprehensive Design Document

**Project**: Hardware-in-the-Loop 2-DOF Robotic Arm Simulator
**Author**: Portfolio Project for NVIDIA / Boston Dynamics / Palantir
**Date**: January 13, 2026
**Timeline**: 3-4 weeks (MVP by Feb 7, 2026)
**Hardware Arrival**: January 14, 2026

---

## Executive Summary

This project implements a **Hardware-in-the-Loop (HIL) robotics simulator** that bridges the sim-to-real gap by synchronizing a MuJoCo physics simulation with real embedded hardware. A 2-DOF robotic arm controlled by STM32 NUCLEO-F446RE receives commands from the simulation, executes them physically, and reports sensor feedback (servo positions + IMU orientation) back to the simulation for real-time visualization.

**Key Innovation**: Bidirectional communication loop where simulation drives hardware while hardware validates and corrects simulation state, demonstrating critical robotics concepts relevant to autonomous systems, digital twins, and robot learning.

**Target Audience**: Technical recruiters and hiring managers at robotics/AI companies seeking candidates with:
- Embedded firmware expertise (C/C++, real-time systems)
- Robotics simulation and control algorithms
- System integration and communication protocols
- DevOps/CI-CD background applied to robotics

---

## System Architecture Overview

### High-Level Block Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                     SIMULATION ENVIRONMENT                       │
│  ┌───────────────┐         ┌─────────────────┐                 │
│  │   MuJoCo      │◄────────┤  Python         │                 │
│  │   Physics     │         │  Middleware     │                 │
│  │   Engine      │─────────►  (Controller)   │                 │
│  └───────┬───────┘         └────────┬────────┘                 │
│          │                           │                           │
│          │ Render                    │ Serial I/O               │
│          ▼                           ▼                           │
│  ┌───────────────┐         ┌─────────────────┐                 │
│  │ Visualization │         │  Serial         │                 │
│  │ Dashboard     │         │  Interface      │                 │
│  │ (Web/Desktop) │         │  (pySerial)     │                 │
│  └───────────────┘         └────────┬────────┘                 │
└─────────────────────────────────────┼──────────────────────────┘
                                       │
                                       │ USB-Serial
                                       │ 115200 baud
                                       │ Binary Protocol
                                       ▼
┌─────────────────────────────────────┼──────────────────────────┐
│                     HARDWARE LAYER                │              │
│                                      ▼              │              │
│  ┌───────────────────────────────────────────┐    │              │
│  │     STM32 NUCLEO-F446RE (Cortex-M4)       │    │              │
│  │                                             │    │              │
│  │  ┌──────────────┐      ┌────────────────┐ │    │              │
│  │  │ UART Handler │      │ Command Parser │ │    │              │
│  │  │ (RX/TX ISR)  │◄────►│ (Protocol)     │ │    │              │
│  │  └──────────────┘      └────────┬───────┘ │    │              │
│  │                                  │          │    │              │
│  │  ┌──────────────┐      ┌────────▼───────┐ │    │              │
│  │  │ Control Loop │◄────►│ Sensor Fusion  │ │    │              │
│  │  │ (50Hz)       │      │ (Comp Filter)  │ │    │              │
│  │  └──────┬───────┘      └────────▲───────┘ │    │              │
│  │         │                       │          │    │              │
│  │         │ PWM                   │ I2C      │    │              │
│  │         ▼                       ▼          │    │              │
│  │  ┌──────────────┐      ┌────────────────┐ │    │              │
│  │  │ TIM2/TIM3    │      │ MPU6050 Driver │ │    │              │
│  │  │ PWM Gen      │      │ (IMU Sensor)   │ │    │              │
│  │  └──────┬───────┘      └────────────────┘ │    │              │
│  └─────────┼────────────────────────────────┘    │              │
│            │                                      │              │
│            ▼                                      ▼              │
│    ┌───────────────┐                  ┌────────────────┐       │
│    │  SG90 Servo 1 │                  │   MPU6050 IMU  │       │
│    │  (Shoulder)   │                  │   (Forearm)    │       │
│    └───────────────┘                  └────────────────┘       │
│            │                                                     │
│            ▼                                                     │
│    ┌───────────────┐                                           │
│    │  SG90 Servo 2 │                                           │
│    │  (Elbow)      │                                           │
│    └───────────────┘                                           │
│                                                                  │
│  Power: ELEGOO 5V Supply (Common Ground with STM32)            │
└─────────────────────────────────────────────────────────────────┘
```

### Component Roles

1. **MuJoCo Physics Engine**: Simulates 2-DOF arm dynamics, collision detection, gravity effects
2. **Python Middleware**: Orchestrates communication, implements control algorithms, manages state sync
3. **Serial Interface**: Handles USB-Serial communication with binary protocol
4. **Visualization Dashboard**: Real-time 3D rendering + telemetry plots
5. **STM32 Firmware**: Real-time control loop, hardware abstraction, protocol handling
6. **Sensor Fusion Module**: Combines servo encoder + IMU data for accurate state estimation
7. **Hardware (Servos + IMU)**: Physical actuators and sensors

### Communication Flow

**Simulation → Hardware (Command Path)**:
1. User/Algorithm generates desired joint angles in Python
2. Middleware validates and packages commands into binary protocol
3. Serial transmission over USB (115200 baud)
4. STM32 UART ISR receives packet, validates CRC
5. Command parser extracts target angles
6. Control loop computes PWM signals via PID controller
7. TIM2/TIM3 generate PWM to drive servos

**Hardware → Simulation (Feedback Path)**:
1. Control loop reads servo positions and MPU6050 data (50Hz)
2. Complementary filter fuses gyro + accel for orientation
3. Firmware packages sensor data into binary telemetry packet
4. UART TX sends packet to Python middleware
5. Middleware updates MuJoCo simulation state
6. Visualization renders synchronized 3D arm + plots telemetry

---

## Technical Decisions & Rationale

### 1. Binary Protocol with Struct Packing

**Decision**: Use fixed-size binary packets instead of JSON/text protocol.

**Rationale**:
- **Deterministic timing**: Fixed 8-16 byte packets ensure <50ms latency requirement
- **Efficiency**: 3-5x smaller than JSON, critical for 115200 baud serial
- **Real-time suitability**: Constant parsing time on STM32 (no dynamic allocation)
- **Industry standard**: Used in robotics (MAVLink, ROS serial), demonstrates professional knowledge

**Trade-off**: Harder to debug than text (mitigated with debug mode and packet visualizer tools)

### 2. PID Control with Feedforward

**Decision**: Implement PID position control with optional feedforward term.

**Rationale**:
- **Timeline fit**: Well-documented, tunable in 2-3 days
- **Portfolio value**: Industry standard, easy to explain in interviews
- **Achievable performance**: Can meet <5° accuracy with proper tuning
- **Extensible**: Can add feedforward, gravity compensation later

**Trade-off**: Less sophisticated than model-based control (acceptable for MVP, can discuss extensions in interviews)

### 3. Basic Complementary Filter for Sensor Fusion

**Decision**: Use simple complementary filter (α*gyro + (1-α)*accel) instead of Kalman filter.

**Rationale**:
- **Computational efficiency**: Runs easily on Cortex-M4 without FPU burden
- **Sufficient accuracy**: Good enough for validation (±2° typical error)
- **Simple implementation**: ~50 lines of code, easy to debug
- **Fast integration**: Focus time on HIL loop, not sensor algorithms

**Trade-off**: Madgwick filter would be better but adds complexity (can mention as improvement in documentation)

### 4. Monorepo Structure

**Decision**: Single Git repository with organized subdirectories for all components.

**Rationale**:
- **Portfolio presentation**: Single impressive repo for recruiters to clone
- **Version consistency**: Firmware + simulation + docs always in sync
- **Simplified workflow**: One README, one clone, unified issue tracking
- **DevOps relevance**: Shows understanding of monorepo benefits (your Jenkins/CI background)

**Trade-off**: Larger repo size (acceptable for portfolio project, not production scale)

### 5. Python Simulation Tests Only (MVP)

**Decision**: Focus testing effort on Python middleware with pytest, defer firmware unit tests.

**Rationale**:
- **Time efficiency**: Python tests are faster to write and iterate
- **Pre-hardware development**: Can test protocol, kinematics, control before Jan 14
- **Good enough validation**: HIL integration test validates firmware behavior
- **Resource allocation**: Firmware unit test setup (Unity/Google Test) is time-consuming

**Trade-off**: Less firmware coverage (acceptable for MVP, can add tests in Week 3 if ahead of schedule)

### 6. ROS2 as Stretch Goal Only

**Decision**: Exclude ROS2 from MVP, keep as optional Week 4 addition.

**Rationale**:
- **Scope management**: ROS2 adds significant complexity (workspace, colcon, message definitions)
- **Core value**: HIL concept is compelling without ROS2 wrapper
- **Documentation benefit**: Mentioning "ROS2-ready architecture" in README shows awareness
- **Time risk**: ROS2 integration could consume Week 2-3 debugging time

**Trade-off**: Less "industry standard" than ROS2-based system (mitigated by explaining architecture is compatible)

---

## Component Specifications

### STM32 Firmware (C/C++ with HAL)

**File Structure**:
```
firmware/
├── Core/
│   ├── Src/
│   │   ├── main.c              # Initialization, main control loop
│   │   ├── uart_handler.c      # UART RX/TX ISR, DMA handling
│   │   ├── protocol.c          # Binary protocol parser/encoder
│   │   ├── servo_control.c     # PWM generation, PID controller
│   │   ├── imu_driver.c        # MPU6050 I2C communication
│   │   └── sensor_fusion.c     # Complementary filter
│   └── Inc/
│       ├── protocol.h          # Packet structures, CRC functions
│       ├── config.h            # Pin definitions, constants
│       └── ...
├── Drivers/ (STM32 HAL)
└── README.md
```

**Key Modules**:

1. **UART Handler** (`uart_handler.c`)
   - RX: Interrupt-driven reception with circular DMA buffer (256 bytes)
   - TX: Non-blocking transmission queue (64 bytes)
   - Framing: Detect packet start (0xAA header), validate length, compute CRC8
   - Error handling: Timeout recovery (100ms), checksum validation

2. **Protocol Parser** (`protocol.c`)
   - Command packet: `[0xAA][CMD][LEN][DATA][CRC8]`
   - Telemetry packet: `[0xAA][0x01][LEN][ANGLES][IMU][CRC8]`
   - Commands:
     - `0x10`: Set joint angles (2x float32)
     - `0x20`: Request telemetry
     - `0x30`: Reset/calibrate
     - `0x40`: Set PID gains
   - See `PROTOCOL.md` for detailed specification

3. **Servo Control** (`servo_control.c`)
   - PWM generation: TIM2_CH1 (shoulder), TIM3_CH2 (elbow)
   - Frequency: 50Hz (20ms period)
   - Pulse width: 1000-2000μs (0-180°, configurable)
   - PID controller: Position control at 50Hz update rate
   - Safety limits: Angle clamping, velocity limiting, watchdog timeout

4. **IMU Driver** (`imu_driver.c`)
   - I2C1 interface (100kHz initially, 400kHz after testing)
   - MPU6050 initialization: Set gyro/accel ranges, sample rate (100Hz)
   - Data reading: Burst read 14 bytes (accel XYZ, temp, gyro XYZ)
   - Calibration: Store bias offsets at startup (500 sample average)

5. **Sensor Fusion** (`sensor_fusion.c`)
   - Complementary filter: `angle = α*(angle + gyro*dt) + (1-α)*accel_angle`
   - α = 0.98 (trust gyro 98%, accel 2%)
   - Output: Roll, pitch angles in radians
   - Update rate: 50Hz (synchronized with control loop)

**Real-Time Loop** (50Hz / 20ms cycle):
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // 1. Read sensors (2ms)
    MPU6050_ReadData(&imu_data);

    // 2. Sensor fusion (0.5ms)
    SensorFusion_Update(&imu_data, &orientation);

    // 3. PID control (1ms)
    float pwm1 = PID_Compute(&pid_shoulder, target_angle[0], current_angle[0]);
    float pwm2 = PID_Compute(&pid_elbow, target_angle[1], current_angle[1]);

    // 4. Apply PWM (0.1ms)
    Servo_SetPWM(SERVO_SHOULDER, pwm1);
    Servo_SetPWM(SERVO_ELBOW, pwm2);

    // 5. Send telemetry (1ms)
    Protocol_SendTelemetry(current_angle, &orientation);
}
```

**Memory Budget**:
- Flash: ~32KB / 512KB (6% - plenty of room)
- RAM: ~16KB / 128KB (12% - safe margin)
- Stack: 4KB (real-time loop is not recursive)
- Heap: Minimal (avoid malloc in control loop)

### Python Middleware

**File Structure**:
```
simulation/middleware/
├── __init__.py
├── serial_manager.py      # Serial port handling, reconnection logic
├── protocol.py            # Binary protocol encode/decode (mirrors firmware)
├── mujoco_controller.py   # MuJoCo sim wrapper, state management
├── kinematics.py          # Forward/inverse kinematics for 2-DOF
├── pid_controller.py      # PID implementation (for sim-only mode)
├── hil_synchronizer.py    # Main orchestration loop
└── config.yaml            # Serial port, PID gains, timeouts
```

**Key Modules**:

1. **Serial Manager** (`serial_manager.py`)
   ```python
   class SerialManager:
       def __init__(self, port='COM3', baud=115200, timeout=0.1):
           self.serial = serial.Serial(port, baud, timeout=timeout)
           self.rx_buffer = bytearray()

       def send_command(self, cmd_type, data):
           packet = Protocol.encode(cmd_type, data)
           self.serial.write(packet)

       def read_telemetry(self):
           # Non-blocking read, parse packets from buffer
           data = self.serial.read(self.serial.in_waiting)
           self.rx_buffer.extend(data)
           return Protocol.decode_telemetry(self.rx_buffer)
   ```

2. **Protocol** (`protocol.py`)
   - Mirrors firmware protocol exactly
   - Uses `struct.pack/unpack` for binary encoding
   - CRC8 implementation (same polynomial as firmware)
   - Packet validation and error reporting

3. **MuJoCo Controller** (`mujoco_controller.py`)
   ```python
   class MuJoCoController:
       def __init__(self, model_path='../mujoco_model/arm.xml'):
           self.model = mujoco.MjModel.from_xml_path(model_path)
           self.data = mujoco.MjData(self.model)

       def set_joint_angles(self, angles):
           self.data.qpos[:] = angles
           mujoco.mj_forward(self.model, self.data)

       def get_joint_angles(self):
           return self.data.qpos.copy()

       def step_simulation(self, dt=0.02):
           mujoco.mj_step(self.model, self.data)
   ```

4. **HIL Synchronizer** (`hil_synchronizer.py`)
   - Main control loop orchestrating sim ↔ hardware sync
   - Mode selection: `SIM_ONLY`, `HIL_LOOP`, `HARDWARE_ONLY`
   - Telemetry logging for offline analysis
   - Latency measurement and monitoring

**Main Loop** (50Hz):
```python
def hil_loop():
    while running:
        start_time = time.time()

        # 1. Get desired state from simulation or user
        target_angles = mujoco_ctrl.get_target_angles()

        # 2. Send command to hardware
        serial_mgr.send_command(CMD_SET_ANGLES, target_angles)

        # 3. Read telemetry from hardware (non-blocking)
        telemetry = serial_mgr.read_telemetry()

        if telemetry:
            # 4. Update simulation with hardware feedback
            mujoco_ctrl.set_joint_angles(telemetry['angles'])

            # 5. Update visualization
            viz.update(telemetry)

        # 6. Maintain 50Hz rate
        elapsed = time.time() - start_time
        time.sleep(max(0, 0.02 - elapsed))
```

### MuJoCo Simulation Model

**File Structure**:
```
simulation/mujoco_model/
├── arm.xml              # Main MJCF model
├── assets/
│   └── (optional meshes/textures)
└── README.md
```

**MJCF Model Structure** (`arm.xml`):
```xml
<mujoco model="2dof_arm">
  <compiler angle="radian" />

  <option timestep="0.001" gravity="0 0 -9.81" />

  <worldbody>
    <!-- Base (fixed to world) -->
    <body name="base" pos="0 0 0">
      <geom type="box" size="0.05 0.05 0.02" rgba="0.3 0.3 0.3 1" />

      <!-- Shoulder link (Link 1) -->
      <body name="shoulder_link" pos="0 0 0.02">
        <joint name="shoulder" type="hinge" axis="0 0 1"
               range="-1.57 1.57" damping="0.1" />
        <geom type="cylinder" size="0.015 0.05" rgba="0.8 0.2 0.2 1"
              pos="0.05 0 0" euler="0 1.57 0" />
        <geom type="box" size="0.05 0.02 0.015" rgba="0.8 0.2 0.2 1"
              pos="0.05 0 0" />

        <!-- Elbow link (Link 2) -->
        <body name="elbow_link" pos="0.1 0 0">
          <joint name="elbow" type="hinge" axis="0 1 0"
                 range="-1.57 1.57" damping="0.08" />
          <geom type="cylinder" size="0.012 0.04" rgba="0.2 0.8 0.2 1"
                pos="0.04 0 0" euler="0 1.57 0" />
          <geom type="box" size="0.04 0.015 0.012" rgba="0.2 0.8 0.2 1"
                pos="0.04 0 0" />

          <!-- End effector -->
          <body name="end_effector" pos="0.08 0 0">
            <geom type="sphere" size="0.01" rgba="0.2 0.2 0.8 1" />

            <!-- IMU sensor (visualization) -->
            <site name="imu" pos="0 0 0" size="0.005" rgba="1 1 0 1" />
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <!-- Position servos (simulating SG90 behavior) -->
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

**Physical Parameters** (to be measured after hardware assembly):
- Link 1 length: ~10cm (base to elbow)
- Link 2 length: ~8cm (elbow to end-effector)
- Link masses: ~20-30g each (estimate for cardboard + servo)
- Joint damping: Tuned empirically to match real servo friction

### Visualization Dashboard

**Technology**: Python with Matplotlib/PyQt5 (simpler than web stack for MVP)

**Layout**:
```
┌─────────────────────────────────────────────────────────────┐
│  HIL Robotics Simulator - Real-Time Dashboard              │
├─────────────────────────────┬───────────────────────────────┤
│                             │  Telemetry Plots              │
│   MuJoCo 3D Rendering       │                               │
│   (800x600)                 │  ┌─────────────────────────┐ │
│                             │  │ Joint Angles (deg)      │ │
│   [Real-time arm visual]    │  │ • Shoulder: 45.2°       │ │
│                             │  │ • Elbow: -30.5°         │ │
│                             │  └─────────────────────────┘ │
│                             │                               │
│                             │  ┌─────────────────────────┐ │
│                             │  │ IMU Orientation         │ │
│                             │  │ • Roll: 12.3°           │ │
│                             │  │ • Pitch: -5.1°          │ │
│                             │  └─────────────────────────┘ │
│                             │                               │
│                             │  ┌─────────────────────────┐ │
│                             │  │ Communication           │ │
│                             │  │ • Latency: 23ms         │ │
│                             │  │ • Packet loss: 0.1%     │ │
├─────────────────────────────┴───────────────────────────────┤
│  Time-Series Plots (last 10 seconds)                        │
│  [Joint angles over time chart]                             │
│  [IMU data over time chart]                                 │
└─────────────────────────────────────────────────────────────┘
```

**Implementation** (`visualization/dashboard.py`):
```python
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import mujoco
import mujoco.viewer

class Dashboard:
    def __init__(self, model, data):
        self.model = model
        self.data = data

        # Create figure with subplots
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))

        # Initialize plots
        self.init_plots()

        # Start animation
        self.anim = FuncAnimation(self.fig, self.update, interval=50)

    def update(self, frame):
        # Update MuJoCo viewer
        mujoco.viewer.launch_passive(self.model, self.data)

        # Update telemetry plots
        # (fetch data from HIL synchronizer)
```

**Stretch**: Web dashboard with React + Three.js if time permits (Week 4)

---

## Communication Protocol Specification

See `docs/PROTOCOL.md` for complete specification. Summary:

### Packet Structure

**Command Packet** (Simulation → Hardware):
```
[HEADER][CMD][LEN][DATA...][CRC8]
  1B     1B   1B   N bytes   1B
```

**Telemetry Packet** (Hardware → Simulation):
```
[HEADER][TYPE][LEN][TIMESTAMP][ANGLES][IMU_DATA][CRC8]
  1B     1B    1B     4B        8B      24B      1B
  (Total: 41 bytes)
```

### Command Types

| CMD | Name | Data | Description |
|-----|------|------|-------------|
| 0x10 | SET_ANGLES | 2x float32 (8B) | Set target joint angles (radians) |
| 0x20 | GET_TELEMETRY | - | Request immediate telemetry packet |
| 0x30 | RESET | - | Reset system, zero angles, recalibrate IMU |
| 0x40 | SET_PID_GAINS | 6x float32 (24B) | Update PID gains (Kp, Ki, Kd per joint) |
| 0x50 | SET_MODE | 1x uint8 | Mode: 0=IDLE, 1=POSITION, 2=TRAJECTORY |

### Telemetry Data

```c
struct TelemetryPacket {
    uint32_t timestamp_ms;      // STM32 system time (ms)
    float joint_angles[2];      // Shoulder, elbow (radians)
    float imu_accel[3];         // X, Y, Z acceleration (m/s²)
    float imu_gyro[3];          // X, Y, Z angular velocity (rad/s)
    float imu_orientation[2];   // Roll, pitch from fusion (radians)
};
```

### CRC-8 Checksum

- Polynomial: 0x07 (x^8 + x^2 + x + 1)
- Initial value: 0x00
- Computed over `[CMD][LEN][DATA]` fields

### Error Handling

- **Timeout**: If no telemetry received for 100ms, simulation shows "DISCONNECTED"
- **CRC mismatch**: Packet discarded, request retransmit
- **Invalid command**: Firmware sends error response (CMD=0xFF)
- **Serial overflow**: Firmware circular buffer (256B), drops oldest data

---

## Control System Design

### PID Controller

**Equation**:
```
u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
```

Where:
- `e(t)` = target_angle - current_angle (position error)
- `Kp` = Proportional gain (stiffness)
- `Ki` = Integral gain (eliminate steady-state error)
- `Kd` = Derivative gain (damping)

**Initial Tuning** (Ziegler-Nichols method):
1. Start with Ki=0, Kd=0
2. Increase Kp until oscillation occurs (Ku)
3. Measure oscillation period (Pu)
4. Calculate: Kp=0.6*Ku, Ki=2*Kp/Pu, Kd=Kp*Pu/8

**Expected Gains** (to be tuned empirically):
- Shoulder: Kp=1.5, Ki=0.05, Kd=0.15
- Elbow: Kp=1.2, Ki=0.03, Kd=0.12

**Anti-Windup**: Clamp integral term to ±10° to prevent overshoot

### Feedforward Compensation (Optional Week 3)

Add gravity compensation term:
```
u_total = u_PID + u_ff
u_ff = τ_gravity / k_motor
```

Where `τ_gravity` is computed from arm kinematics and link masses.

### Kinematics

**Forward Kinematics** (joint angles → end-effector position):
```python
def forward_kinematics(theta1, theta2, L1=0.1, L2=0.08):
    x = L1 * cos(theta1) + L2 * cos(theta1 + theta2)
    y = L1 * sin(theta1) + L2 * sin(theta1 + theta2)
    return (x, y)
```

**Inverse Kinematics** (end-effector position → joint angles):
```python
def inverse_kinematics(x, y, L1=0.1, L2=0.08):
    # Law of cosines
    c2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = atan2(sqrt(1 - c2**2), c2)  # Elbow up solution

    k1 = L1 + L2 * cos(theta2)
    k2 = L2 * sin(theta2)
    theta1 = atan2(y, x) - atan2(k2, k1)

    return (theta1, theta2)
```

---

## Data Flow Diagrams

### Initialization Sequence

```
User                 Python               STM32               Hardware
 │                     │                     │                    │
 │──Start Dashboard──►│                     │                    │
 │                     │──Open Serial────►│                    │
 │                     │                     │                    │
 │                     │◄─────ACK──────────│                    │
 │                     │                     │                    │
 │                     │──GET_TELEMETRY──►│                    │
 │                     │                     │──Read Sensors──►│
 │                     │                     │◄───Raw Data──────│
 │                     │                     │                    │
 │                     │◄──Telemetry───────│                    │
 │◄──Display Ready───│                     │                    │
```

### Normal Operation (HIL Loop)

```
MuJoCo Sim       Python Middleware      STM32 Firmware       Hardware
    │                     │                     │                  │
    │──Compute Target──►│                     │                  │
    │   Angles           │                     │                  │
    │                     │──SET_ANGLES────►│                  │
    │                     │   (23.5°, -15°)    │                  │
    │                     │                     │──PID Compute──►│
    │                     │                     │                  │
    │                     │                     │◄──PWM to Servo──│
    │                     │                     │                  │
    │                     │                     │──Read IMU─────►│
    │                     │                     │◄──Orientation───│
    │                     │                     │                  │
    │                     │◄──TELEMETRY────────│                  │
    │                     │   (angles, IMU)    │                  │
    │◄──Update State────│                     │                  │
    │   Render Frame     │                     │                  │
    │──────────►         │                     │                  │
    │                     │                     │                  │
    │  (Repeat at 50Hz)  │                     │                  │
```

### Error Recovery

```
Python               STM32
   │                     │
   │──SET_ANGLES────►│
   │                     │
   │                 [TIMEOUT]
   │                     │
   │  (100ms delay)     │
   │                     │
   │──GET_TELEMETRY──►│
   │                     │
   │◄──TELEMETRY──────│ (Connection restored)
   │                     │
   │──SET_ANGLES────►│  (Resume normal operation)
```

---

## Hardware Configuration

### STM32 Pin Assignment

| Function | Pin | Peripheral | Notes |
|----------|-----|------------|-------|
| UART TX | PA2 | USART2_TX | USB Serial via ST-Link |
| UART RX | PA3 | USART2_RX | USB Serial via ST-Link |
| Servo 1 PWM | PA0 | TIM2_CH1 | Shoulder joint (50Hz) |
| Servo 2 PWM | PA1 | TIM3_CH2 | Elbow joint (50Hz) |
| I2C SCL | PB8 | I2C1_SCL | MPU6050 clock |
| I2C SDA | PB9 | I2C1_SDA | MPU6050 data |
| LED (status) | PA5 | GPIO | Nucleo onboard LED |

### Power Supply Wiring

```
ELEGOO Power Module (5V Output)
    │
    ├──► Servo 1 (Red wire)
    ├──► Servo 2 (Red wire)
    ├──► MPU6050 VCC
    │
    └──► STM32 GND (COMMON GROUND - CRITICAL!)

STM32 5V Pin: NOT USED for servos (insufficient current)

Servo Signal Wires:
    Servo 1 Orange → PA0 (TIM2_CH1)
    Servo 2 Orange → PA1 (TIM3_CH2)
    Both Black → GND

MPU6050:
    VCC → 3.3V or 5V (check module voltage)
    GND → STM32 GND
    SCL → PB8
    SDA → PB9
```

### Physical Assembly

```
        [STM32 Board]
             │
       ┌─────┴─────┐
       │           │
   [Servo 1]   [Servo 2]
   (Shoulder)   (Elbow)
       │           │
       │      ┌────┴────┐
       │      │ Link 2  │
       │      │[MPU6050]│
       │      └─────────┘
   ┌───┴───┐
   │ Link 1│
   └───────┘
       │
  [Base/Mount]
```

**Materials**:
- Cardboard or 3D printed brackets
- Hot glue or screws for servo mounting
- Double-sided tape for IMU attachment
- Zip ties for cable management

---

## Development Workflow

### Repository Structure

```
Robot/
├── .git/
├── .gitignore
├── README.md                  # Main project documentation
├── LICENSE
│
├── docs/
│   ├── DESIGN.md             # This document
│   ├── ARCHITECTURE.md       # System architecture diagrams
│   ├── PROTOCOL.md           # Communication protocol spec
│   ├── SETUP.md              # Hardware setup guide
│   ├── TESTING.md            # Test procedures
│   └── images/               # Diagrams, screenshots
│
├── firmware/
│   ├── .project              # STM32CubeIDE project
│   ├── .cproject
│   ├── Core/
│   │   ├── Src/
│   │   └── Inc/
│   ├── Drivers/
│   ├── README.md
│   └── .gitignore
│
├── simulation/
│   ├── setup.py              # Python package setup
│   ├── requirements.txt
│   ├── mujoco_model/
│   │   └── arm.xml
│   ├── middleware/
│   │   ├── __init__.py
│   │   ├── serial_manager.py
│   │   ├── protocol.py
│   │   ├── mujoco_controller.py
│   │   └── hil_synchronizer.py
│   ├── tests/
│   │   ├── test_protocol.py
│   │   ├── test_kinematics.py
│   │   └── test_serial.py
│   └── README.md
│
├── visualization/
│   ├── dashboard.py
│   └── README.md
│
└── scripts/
    ├── build_firmware.sh     # Automated build scripts
    ├── flash_stm32.sh
    └── run_simulation.sh
```

### Git Workflow

**Branching Strategy**:
- `main`: Stable, demo-ready code
- `develop`: Integration branch
- `feature/*`: Individual features (e.g., `feature/imu-driver`, `feature/pid-controller`)

**Commit Message Convention**:
```
<type>(<scope>): <subject>

Types: feat, fix, docs, test, refactor
Examples:
  feat(firmware): Add MPU6050 I2C driver
  fix(protocol): Correct CRC8 calculation
  docs(readme): Add hardware setup section
```

### Build System

**Firmware**:
- STM32CubeIDE project (Eclipse-based)
- Automated build: `arm-none-eabi-gcc` via Makefile
- Flash: ST-Link via STM32CubeProgrammer

**Python**:
```bash
# Setup virtual environment
python -m venv venv
source venv/bin/activate  # or `venv\Scripts\activate` on Windows

# Install dependencies
pip install -r requirements.txt

# Run tests
pytest simulation/tests/

# Run simulation
python simulation/middleware/hil_synchronizer.py
```

### Development Environment Setup (Windows + WSL2)

**Windows Side** (Firmware):
1. Install STM32CubeIDE
2. Install STM32 drivers (ST-Link)
3. Install Git for Windows

**WSL2 Side** (Simulation):
```bash
# Install MuJoCo
pip install mujoco
pip install mujoco-python-viewer

# Install Python dependencies
pip install numpy matplotlib pyserial pytest

# Note: Serial port access in WSL2 requires USB passthrough
# Use `usbipd-win` or develop firmware tests natively on Windows
```

---

## Testing Strategy

### Python Simulation Tests (pytest)

**Test Coverage**:
1. **Protocol Tests** (`test_protocol.py`)
   - Packet encoding/decoding
   - CRC validation
   - Error handling

2. **Kinematics Tests** (`test_kinematics.py`)
   - Forward kinematics accuracy
   - Inverse kinematics accuracy
   - Singularity handling

3. **Serial Manager Tests** (`test_serial.py`)
   - Mock serial communication
   - Timeout handling
   - Buffer management

**Example Test**:
```python
def test_protocol_encode_decode():
    """Test round-trip encoding/decoding of command packet"""
    cmd = 0x10  # SET_ANGLES
    data = struct.pack('ff', 0.5, -0.3)

    packet = Protocol.encode(cmd, data)
    assert packet[0] == 0xAA  # Header
    assert packet[1] == cmd

    decoded = Protocol.decode(packet)
    assert decoded['cmd'] == cmd
    assert np.allclose(decoded['data'], [0.5, -0.3])
```

### Hardware-in-Loop Integration Tests

**Test Procedure** (Week 2, after hardware arrives):
1. **Connectivity Test**: Verify serial communication
2. **Servo Test**: Command single joint, measure response time
3. **IMU Test**: Read sensor data, validate calibration
4. **Closed-Loop Test**: Command trajectory, plot tracking error
5. **Latency Test**: Measure round-trip time (command → telemetry)
6. **Stress Test**: Rapid angle changes, verify stability

**Acceptance Criteria**:
- Latency: <50ms (target: 20-30ms)
- Position accuracy: <5° steady-state error
- Packet loss: <1%
- No servo jitter or oscillation
- IMU drift: <5°/minute

---

## Timeline & Milestones

### Week 1 (Jan 13-19): Architecture & Foundation
**Hardware Status**: Waiting for delivery (arrives Jan 14)

**Milestones**:
- [x] Design document complete (this document)
- [ ] Repository structure created
- [ ] MuJoCo arm model implemented (hand-written XML)
- [ ] Python protocol module with tests (pytest)
- [ ] STM32CubeMX project initialized
- [ ] Basic firmware skeleton (HAL init, UART echo test)

**Deliverables**:
- `docs/DESIGN.md`
- `simulation/mujoco_model/arm.xml`
- `simulation/middleware/protocol.py` with passing tests
- `firmware/` project compiling successfully

**Risk**: Hardware delayed → Continue with pure simulation development

---

### Week 2 (Jan 20-26): HIL Communication Loop
**Hardware Status**: Assembled and powered

**Milestones**:
- [ ] STM32 firmware: UART + protocol handler working
- [ ] Servo PWM control operational (manual angle setting)
- [ ] Python-to-STM32 command sending verified
- [ ] STM32-to-Python telemetry receiving verified
- [ ] Basic HIL loop: Command angle, receive feedback, update sim
- [ ] MPU6050 driver reading raw data

**Deliverables**:
- Firmware can execute SET_ANGLES command
- Python can send commands and receive telemetry
- Demo video: Sim sends command → Hardware moves → Sim updates

**Acceptance Test**: Command shoulder to 45°, elbow to -30°. Hardware moves within 2 seconds, telemetry shows <5° error.

**Risk**: Serial communication issues → Use logic analyzer, add debug prints

---

### Week 3 (Jan 27-Feb 2): Control & Sensor Fusion
**Hardware Status**: Fully operational

**Milestones**:
- [ ] PID controller implemented and tuned (firmware)
- [ ] Complementary filter for IMU fusion working
- [ ] Trajectory generation (Python) → smooth motion
- [ ] Visualization dashboard showing live telemetry
- [ ] Kinematics (FK/IK) integrated for Cartesian control
- [ ] Error handling: Timeouts, disconnection recovery

**Deliverables**:
- Hardware tracks commanded trajectories with <5° error
- Dashboard displays synchronized 3D arm + telemetry plots
- System recovers gracefully from temporary disconnections

**Acceptance Test**: Command figure-8 trajectory in Cartesian space. Hardware executes smoothly, simulation matches within 5° tolerance.

**Risk**: PID tuning difficult → Use auto-tuning, consult online resources

---

### Week 4 (Feb 3-9): Documentation & Portfolio Polish

**Milestones**:
- [ ] Professional README with architecture diagrams
- [ ] Hardware setup guide with photos
- [ ] Demo video (3-5 minutes) showing key features
- [ ] Code cleanup, comments, docstrings
- [ ] Optional: ROS2 integration (if ahead of schedule)
- [ ] Optional: Web dashboard (React + Three.js)

**Deliverables**:
- GitHub repo ready for recruiter review
- Demo video uploaded (YouTube or GitHub)
- Technical blog post or writeup
- Slides for interview presentation

**Acceptance Test**: Show repo to mentor/peer for feedback. Can they understand the project in 5 minutes?

---

## Risk Analysis & Mitigation

### Technical Risks

1. **Serial Communication Instability**
   - **Impact**: High (blocks HIL loop)
   - **Probability**: Medium
   - **Mitigation**:
     - Use CRC validation on every packet
     - Implement timeout and retry logic
     - Add debug mode with packet logging
     - Test with loopback (TX→RX) first

2. **PID Tuning Difficulty**
   - **Impact**: Medium (affects demo quality)
   - **Probability**: Medium
   - **Mitigation**:
     - Start with conservative gains
     - Use Ziegler-Nichols as starting point
     - Record step response data for offline tuning
     - Implement gain scheduler if needed

3. **Servo Jitter/Instability**
   - **Impact**: Medium (looks unprofessional in demo)
   - **Probability**: Medium (SG90 servos are cheap)
   - **Mitigation**:
     - Add deadband in PID controller (±1°)
     - Low-pass filter on commanded angles
     - Ensure stable power supply (capacitors)
     - Consider servo replacement if defective

4. **IMU Calibration Issues**
   - **Impact**: Low (not critical for MVP)
   - **Probability**: High (MPU6050 notorious for drift)
   - **Mitigation**:
     - Use servo position as ground truth
     - Keep IMU as secondary validation only
     - Implement on-demand recalibration command
     - Document limitations honestly

5. **MuJoCo-Hardware Mismatch**
   - **Impact**: Medium (affects visual fidelity)
   - **Probability**: Medium
   - **Mitigation**:
     - Measure physical arm parameters accurately
     - Tune MuJoCo damping/friction to match reality
     - Accept small discrepancies, document in README
     - Use hardware as ground truth (not sim)

### Schedule Risks

1. **Hardware Delivery Delay**
   - **Impact**: High (can't test HIL loop)
   - **Probability**: Low (Amazon Prime)
   - **Mitigation**:
     - Maximize Week 1 simulation development
     - Implement firmware tests with emulated sensors
     - Have contingency: Pure simulation demo + plan for hardware

2. **Scope Creep**
   - **Impact**: High (miss deadline)
   - **Probability**: High (exciting project!)
   - **Mitigation**:
     - Strict MVP definition (this document)
     - ROS2, 3rd DOF, CV are explicitly stretch goals
     - Weekly milestone reviews
     - Cut features aggressively if behind schedule

3. **Debugging Time Sink**
   - **Impact**: Medium (delays milestones)
   - **Probability**: High (embedded systems are hard)
   - **Mitigation**:
     - Allocate 50% buffer time for debugging
     - Implement comprehensive logging from day 1
     - Ask for help early (online communities, mentors)
     - Don't rabbit-hole: Set 4-hour limit per bug

---

## Success Metrics

### Technical Metrics
- [ ] Latency: <50ms (measured via timestamp deltas)
- [ ] Position accuracy: <5° steady-state error
- [ ] Packet loss: <1% over 10-minute test
- [ ] System uptime: >95% (handles reconnections)
- [ ] Test coverage: >80% for Python middleware

### Portfolio Metrics
- [ ] GitHub stars: >5 (share with robotics communities)
- [ ] README views: >100 (track via GitHub Insights)
- [ ] Demo video views: >50 (YouTube/LinkedIn)
- [ ] Mentioned in interview: "Tell me about your robotics project"
- [ ] Technical depth: Can whiteboard architecture in 30 min

### Learning Outcomes
- [ ] Understand HIL concepts and sim-to-real gap
- [ ] Proficient with MuJoCo simulation framework
- [ ] Embedded C/C++ skills reinforced (STM32 HAL)
- [ ] Control theory applied (PID, kinematics)
- [ ] Professional documentation and presentation

---

## Next Steps

### Immediate Actions (Today - Jan 13)

1. **Review this design document** with mentor/peer
2. **Create GitHub repository** with initial structure
3. **Set up development environment**:
   - STM32CubeIDE installed
   - Python venv + MuJoCo tested
   - Git configured

### Tomorrow (Jan 14 - Hardware Arrives)

1. **Inventory hardware**: Verify all components present
2. **Initial power test**: Plug in STM32, test USB-Serial
3. **Begin firmware skeleton**: STM32CubeMX project with UART
4. **Start MuJoCo model**: Hand-write initial arm.xml

### Week 1 Focus

- Complete software foundation (firmware + simulation)
- Test hardware components individually (servo, IMU)
- Establish basic serial communication
- Create initial visualizations

---

## References & Resources

### Documentation
- [STM32 HAL Documentation](https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages.html)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [MPU6050 Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- [SG90 Servo Specs](http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf)

### Tutorials
- Complementary Filter: [Philip Salmony YouTube](https://www.youtube.com/watch?v=whSw42XddsU)
- PID Tuning: [Brett Beauregard Arduino PID Library](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/)
- MuJoCo Tutorials: [DeepMind Control Suite](https://github.com/deepmind/dm_control)

### Inspiration Projects
- [OpenDog Quadruped](https://www.youtube.com/c/JamesABruton) - Professional embedded + mech
- [MIT Mini Cheetah](https://www.youtube.com/watch?v=xNeZWP5Mx9s) - HIL simulation
- [Acrobatic Quadcopter](https://www.youtube.com/watch?v=Vm_x-7jmxig) - Sim-to-real transfer

---

## Appendix: Design Questions & Answers

*This section documents the clarifying questions and design decisions made during brainstorming.*

**Q1: Binary vs Text Protocol?**
**A**: Binary with struct packing for efficiency and deterministic timing.

**Q2: PID vs Model-Based Control?**
**A**: PID with feedforward for timeline and portfolio balance.

**Q3: Visualization Priorities?**
**A**: Real-time 3D MuJoCo + live telemetry plots (angles, IMU, latency).

**Q4: Development Workflow?**
**A**: Monorepo structure with organized subdirectories.

**Q5: ROS2 Integration?**
**A**: Stretch goal only, not part of MVP.

**Q6: Sensor Fusion Complexity?**
**A**: Basic complementary filter for efficiency and timeline.

**Q7: Testing Strategy?**
**A**: Python simulation tests (pytest), HIL integration validation.

**Q8: Servo Power Supply?**
**A**: ELEGOO power module (5V rail), common ground with STM32.

**Q9: MuJoCo Model Creation?**
**A**: Hand-write XML based on measured dimensions for full control.

**Q10: Key Constraints?**
**A**: Windows+WSL2 dev environment, GitHub portfolio focus, budget-friendly (FOSS only), interview-explainable architecture.

---

**Document Status**: ✅ Complete - Ready for implementation planning phase

**Next Document**: Use `/superpowers:write-plan` to generate detailed implementation plan with task breakdown.
