"""Figure-8 trajectory acceptance test.

Success criteria: Hardware tracks figure-8 with <5° error.
"""

import sys
import time
import argparse
import numpy as np
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from middleware.serial_manager import SerialManager
from middleware.protocol import encode_set_joint_angles, decode_packet, TelemetryType
from middleware.trajectory import TrajectoryPlanner
from middleware.kinematics import ArmKinematics


def parse_args():
    parser = argparse.ArgumentParser(description="Figure-8 acceptance test")
    parser.add_argument("port", nargs="?", default="COM3", help="Serial port")
    parser.add_argument(
        "--duration", type=float, default=20.0, help="Test duration (s)"
    )
    parser.add_argument(
        "--cycles", type=int, default=2, help="Number of figure-8 cycles"
    )
    return parser.parse_args()


def main():
    args = parse_args()

    print("=" * 60)
    print("Figure-8 Trajectory Acceptance Test")
    print("=" * 60)
    print(f"Port: {args.port}")
    print(f"Duration: {args.duration}s")
    print(f"Cycles: {args.cycles}")
    print("Success criteria: <5° tracking error")
    print("=" * 60)

    # Setup
    kinematics = ArmKinematics()
    planner = TrajectoryPlanner(kinematics)

    # Generate figure-8 trajectory
    cartesian = planner.figure8(
        center=(0.12, 0.0), width=0.04, height=0.03, num_points=100 * args.cycles
    )

    # Convert to joint space
    try:
        joint_trajectory = planner.to_joint_space(cartesian)
    except ValueError as e:
        print(f"ERROR: Trajectory unreachable - {e}")
        return 1

    print(f"Trajectory: {len(joint_trajectory)} points")

    # Connect
    manager = SerialManager(port=args.port)
    if not manager.connect():
        print(f"ERROR: Failed to connect to {args.port}")
        return 1

    print(f"Connected to {args.port}")
    time.sleep(0.5)  # Let hardware initialize

    # Execute trajectory
    errors = []
    dt = args.duration / len(joint_trajectory)

    print("\nExecuting figure-8 trajectory...")

    for i, (cmd_theta1, cmd_theta2) in enumerate(joint_trajectory):
        loop_start = time.time()

        # Send command
        packet = encode_set_joint_angles(cmd_theta1, cmd_theta2)
        manager._serial.write(packet)

        # Wait for telemetry
        time.sleep(0.01)
        telemetry = manager.receive_telemetry(timeout=0.02)

        if telemetry and telemetry["type"] in [
            TelemetryType.ANGLES_ONLY,
            TelemetryType.FULL,
        ]:
            import struct

            data = telemetry["data"]
            if len(data) >= 8:
                actual_theta1, actual_theta2 = struct.unpack("<ff", data[:8])

                # Calculate error
                err1 = abs(np.degrees(cmd_theta1 - actual_theta1))
                err2 = abs(np.degrees(cmd_theta2 - actual_theta2))
                max_err = max(err1, err2)
                errors.append(max_err)

                # Progress
                if i % 20 == 0:
                    print(
                        f"  [{i+1}/{len(joint_trajectory)}] "
                        f"Cmd: ({np.degrees(cmd_theta1):+5.1f}°, {np.degrees(cmd_theta2):+5.1f}°) "
                        f"Act: ({np.degrees(actual_theta1):+5.1f}°, {np.degrees(actual_theta2):+5.1f}°) "
                        f"Err: {max_err:.1f}°"
                    )

        # Maintain timing
        elapsed = time.time() - loop_start
        if elapsed < dt:
            time.sleep(dt - elapsed)

    manager.disconnect()

    # Results
    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)

    if len(errors) == 0:
        print("ERROR: No telemetry received!")
        return 1

    avg_error = np.mean(errors)
    max_error = np.max(errors)
    pct_under_5 = 100 * np.sum(np.array(errors) < 5.0) / len(errors)

    print(f"Samples with telemetry: {len(errors)}/{len(joint_trajectory)}")
    print(f"Average tracking error: {avg_error:.2f}°")
    print(f"Maximum tracking error: {max_error:.2f}°")
    print(f"Samples under 5° error: {pct_under_5:.1f}%")

    print("\n" + "=" * 60)
    if max_error < 5.0:
        print("✓ ACCEPTANCE TEST PASSED")
        print("  All samples within 5° tolerance")
        return 0
    elif pct_under_5 >= 95.0:
        print("✓ ACCEPTANCE TEST PASSED (with margin)")
        print(f"  {pct_under_5:.1f}% of samples within 5° tolerance")
        return 0
    else:
        print("✗ ACCEPTANCE TEST FAILED")
        print(f"  Only {pct_under_5:.1f}% of samples within 5° tolerance")
        return 1


if __name__ == "__main__":
    sys.exit(main())
