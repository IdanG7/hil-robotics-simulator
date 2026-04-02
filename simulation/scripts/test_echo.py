"""Echo test: Verify Python <-> STM32 communication."""

import sys
import time
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

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

    print(f"Connected to {port}")

    # Send test commands and wait for ACK
    test_count = 10
    success_count = 0

    for i in range(test_count):
        # Send SET_JOINT_ANGLES command with dummy data
        data = bytes([0] * 8)  # Two floats (0.0, 0.0)

        if not manager.send_command(CommandType.SET_JOINT_ANGLES, data):
            print(f"[FAIL] Test {i+1}: Send failed")
            continue

        # Wait for ACK
        telemetry = manager.receive_telemetry(timeout=0.1)

        if telemetry is not None and telemetry['type'] == TelemetryType.ACK:
            print(f"[PASS] Test {i+1}: Echo successful")
            success_count += 1
        else:
            print(f"[FAIL] Test {i+1}: No ACK received")

        time.sleep(0.1)

    # Disconnect
    manager.disconnect()

    # Summary
    print("=" * 50)
    print(f"Results: {success_count}/{test_count} successful")
    print(f"Success rate: {success_count/test_count*100:.1f}%")

    if success_count == test_count:
        print("Echo test PASSED")
        return 0
    else:
        print("Echo test FAILED")
        return 1


if __name__ == '__main__':
    sys.exit(main())
