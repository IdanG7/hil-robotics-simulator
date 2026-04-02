"""Servo sweep test: Verify smooth motion."""

import argparse
import sys
import time
import math
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from middleware.serial_manager import SerialManager
from middleware.protocol import encode_set_joint_angles


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Servo sweep test')
    parser.add_argument('port', nargs='?', default='COM3', help='Serial port (e.g. COM3)')
    parser.add_argument('--both', action='store_true', help='Sweep elbow opposite shoulder')
    parser.add_argument('--amplitude-deg', type=float, default=90.0, help='Sweep amplitude in degrees')
    parser.add_argument('--duration', type=float, default=10.0, help='Sweep duration in seconds')
    parser.add_argument('--steps', type=int, default=100, help='Number of sweep steps')
    return parser.parse_args()


def main():
    args = parse_args()
    port = args.port
    amplitude_deg = max(0.0, min(args.amplitude_deg, 90.0))
    if amplitude_deg != args.amplitude_deg:
        print(f"Note: Clamping amplitude to {amplitude_deg:.1f} degrees")
    amplitude_rad = math.radians(amplitude_deg)

    print("Servo Sweep Test")
    print("=" * 50)

    manager = SerialManager(port=port, baud=115200)
    if not manager.connect():
        print(f"ERROR: Failed to connect to {port}")
        return 1

    print(f"Connected to {port}")
    if args.both:
        print(f"Sweeping shoulder/elbow: ±{amplitude_deg:.0f}° (opposite phase)")
    else:
        print(f"Sweeping shoulder: ±{amplitude_deg:.0f}° (elbow holds 0°)")

    # Sweep shoulder
    duration = max(0.1, args.duration)  # seconds
    steps = max(1, args.steps)
    dt = duration / steps

    for i in range(steps):
        t = i / steps
        # Sine wave: 0 -> pi/2 -> 0 -> -pi/2 -> 0
        shoulder = amplitude_rad * math.sin(2 * math.pi * t)
        elbow = -shoulder if args.both else 0.0

        packet = encode_set_joint_angles(shoulder, elbow)
        manager._serial.write(packet)

        if args.both:
            print(
                f"\r[{i+1}/{steps}] Shoulder: {math.degrees(shoulder):+6.1f} deg "
                f"Elbow: {math.degrees(elbow):+6.1f} deg",
                end='',
            )
        else:
            print(
                f"\r[{i+1}/{steps}] Shoulder: {math.degrees(shoulder):+6.1f} deg",
                end='',
            )
        time.sleep(dt)

    print("\nSweep complete")
    manager.disconnect()
    return 0


if __name__ == '__main__':
    sys.exit(main())
