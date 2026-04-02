"""Simple telemetry test - verify we can receive data from hardware."""

import sys
import time
import struct
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from middleware.serial_manager import SerialManager
from middleware.protocol import encode_set_joint_angles, TelemetryType


def main():
    port = "COM3"
    
    print("=" * 60)
    print("Telemetry Test")
    print("=" * 60)

    # Connect
    manager = SerialManager(port=port)
    if not manager.connect():
        print(f"ERROR: Failed to connect to {port}")
        return 1

    print(f"Connected to {port}\n")

    # Send a few commands and check for telemetry
    test_angles = [
        (0.0, 0.0),
        (0.3, 0.3),
        (-0.3, -0.3),
        (0.0, 0.0),
    ]

    for i, (theta1, theta2) in enumerate(test_angles):
        print(f"Test {i+1}: Commanding ({theta1:.2f}, {theta2:.2f}) rad")
        
        # Send command
        packet = encode_set_joint_angles(theta1, theta2)
        manager._serial.write(packet)
        
        # Wait a bit for servo to move
        time.sleep(0.2)
        
        # Try to receive telemetry multiple times
        telemetry_received = False
        for attempt in range(5):
            telemetry = manager.receive_telemetry(timeout=0.1)
            
            if telemetry:
                telemetry_received = True
                telem_type = telemetry['type']
                data = telemetry['data']
                
                print(f"  [OK] Telemetry received! Type: {telem_type}")
                
                if len(data) >= 8:
                    actual_theta1, actual_theta2 = struct.unpack("<ff", data[:8])
                    print(f"    Actual angles: ({actual_theta1:.3f}, {actual_theta2:.3f}) rad")
                    print(f"    Actual angles: ({actual_theta1*57.3:.1f}°, {actual_theta2*57.3:.1f}°)")
                break
            
            time.sleep(0.05)
        
        if not telemetry_received:
            print(f"  [FAIL] No telemetry received after 5 attempts")
        
        print()

    manager.disconnect()
    print("Test complete!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
