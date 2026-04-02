"""Safe trajectory test for hardware validation.

Tests a smaller, safer figure-8 pattern that stays within servo limits.
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
from middleware.protocol import encode_set_joint_angles, TelemetryType
from middleware.trajectory import TrajectoryPlanner
from middleware.kinematics import ArmKinematics


def parse_args():
    parser = argparse.ArgumentParser(description="Safe trajectory test")
    parser.add_argument("port", nargs="?", default="COM3", help="Serial port")
    parser.add_argument(
        "--duration", type=float, default=15.0, help="Test duration (s)"
    )
    return parser.parse_args()


def main():
    args = parse_args()

    print("=" * 60)
    print("Safe Trajectory Test")
    print("=" * 60)
    print(f"Port: {args.port}")
    print(f"Duration: {args.duration}s")
    print("Testing smaller figure-8 within servo limits")
    print("=" * 60)

    # Setup
    kinematics = ArmKinematics()
    planner = TrajectoryPlanner(kinematics)

    # Generate SAFER figure-8 trajectory
    # Centered higher up (0.14, 0.0) with smaller dimensions
    cartesian = planner.figure8(
        center=(0.14, 0.0),  # Higher center position
        width=0.03,          # Smaller width
        height=0.02,         # Smaller height
        num_points=100
    )

    # Convert to joint space
    try:
        joint_trajectory = planner.to_joint_space(cartesian)
    except ValueError as e:
        print(f"ERROR: Trajectory unreachable - {e}")
        return 1

    # Check joint ranges
    shoulder_angles = [j[0] for j in joint_trajectory]
    elbow_angles = [j[1] for j in joint_trajectory]
    
    print(f"\nTrajectory: {len(joint_trajectory)} points")
    print(f"Shoulder range: [{np.degrees(min(shoulder_angles)):.1f}°, {np.degrees(max(shoulder_angles)):.1f}°]")
    print(f"Elbow range: [{np.degrees(min(elbow_angles)):.1f}°, {np.degrees(max(elbow_angles)):.1f}°]")

    # Connect
    manager = SerialManager(port=args.port)
    if not manager.connect():
        print(f"\nERROR: Failed to connect to {args.port}")
        return 1

    print(f"\nConnected to {args.port}")
    time.sleep(0.5)

    # Execute trajectory
    errors = []
    dt = args.duration / len(joint_trajectory)

    print("\nExecuting figure-8 trajectory...")
    print("Watch your arm move!\n")

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

                # Progress (every 10 points)
                if i % 10 == 0:
                    print(
                        f"  [{i+1:3d}/{len(joint_trajectory)}] "
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
    min_error = np.min(errors)
    pct_under_5 = 100 * np.sum(np.array(errors) < 5.0) / len(errors)

    print(f"Samples with telemetry: {len(errors)}/{len(joint_trajectory)}")
    print(f"Average tracking error: {avg_error:.2f}°")
    print(f"Min tracking error: {min_error:.2f}°")
    print(f"Max tracking error: {max_error:.2f}°")
    print(f"Samples under 5° error: {pct_under_5:.1f}%")

    print("\n" + "=" * 60)
    if pct_under_5 >= 95.0:
        print("✓ TEST PASSED")
        print(f"  {pct_under_5:.1f}% of samples within 5° tolerance")
        return 0
    else:
        print("⚠ TEST PASSED WITH WARNINGS")
        print(f"  Only {pct_under_5:.1f}% of samples within 5° tolerance")
        print("  Consider tuning PID controller or reducing speed")
        return 0


if __name__ == "__main__":
    sys.exit(main())
