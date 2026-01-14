# Middleware Module

**Purpose**: Communication protocol and control middleware for HIL robotics simulator.

## Components

### `protocol.py`

Binary communication protocol implementation for simulation ↔ hardware communication.

**Key Functions**:
- `crc8(data: bytes) -> int`: Compute CRC-8 checksum (polynomial 0x07)
- `encode_packet(cmd_type: int, data: bytes) -> bytes`: Encode command into binary packet
- `decode_packet(packet: bytes) -> Optional[Dict]`: Decode binary packet with validation
- `encode_set_joint_angles(shoulder: float, elbow: float) -> bytes`: Convenience function for joint angle commands
- `decode_telemetry_full(packet: bytes) -> Dict`: Decode full telemetry packet

**Enums**:
- `CommandType`: Command types sent from simulation to hardware (0x10-0x50)
- `TelemetryType`: Telemetry types sent from hardware to simulation (0x01-0xF1)

**Packet Structure**:
```
[HEADER=0xAA][TYPE][LENGTH][DATA][CRC8]
```

**Example Usage**:
```python
from middleware.protocol import encode_set_joint_angles, decode_telemetry_full

# Send command to hardware
packet = encode_set_joint_angles(0.785, -0.524)  # 45°, -30° in radians
serial.write(packet)

# Decode telemetry from hardware
response = serial.read(56)  # 52 bytes data + 4 bytes overhead
telemetry = decode_telemetry_full(response)
print(f"Joint angles: {telemetry['joint_angles']}")
print(f"IMU orientation: {telemetry['imu_orientation']}")
```

## Testing

Run protocol tests:
```bash
cd simulation
pytest middleware/tests/test_protocol.py -v
```

Run with coverage:
```bash
pytest middleware/tests/test_protocol.py -v --cov=middleware.protocol --cov-report=term-missing
```

**Test Coverage**: 93% (12 tests passing)

## Protocol Specification

See `docs/PROTOCOL.md` for complete binary protocol specification including:
- CRC-8 algorithm details
- Command type definitions
- Telemetry packet structures
- Error handling procedures

## Future Components

- `serial_manager.py`: Serial port communication manager
- `mujoco_controller.py`: MuJoCo simulation controller
- `kinematics.py`: Forward/inverse kinematics
- `hil_synchronizer.py`: Hardware-in-the-loop synchronization
