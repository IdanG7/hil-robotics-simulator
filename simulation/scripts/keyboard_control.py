"""Interactive keyboard controller for the robotic arm.

Controls:
  W/S - Move shoulder up/down
  A/D - Move elbow left/right
  R - Reset to home position (0, 0)
  Q - Quit
"""

import sys
import time
import msvcrt
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from middleware.serial_manager import SerialManager
from middleware.protocol import encode_set_joint_angles


def main():
    port = "COM3"
    
    print("=" * 60)
    print("Interactive Arm Controller")
    print("=" * 60)
    print("Controls:")
    print("  W/S - Shoulder up/down")
    print("  A/D - Elbow left/right")
    print("  R - Reset to home (0, 0)")
    print("  Q - Quit")
    print("=" * 60)

    # Connect
    manager = SerialManager(port=port)
    if not manager.connect():
        print(f"ERROR: Failed to connect to {port}")
        return 1

    print(f"Connected to {port}")
    print("\nReady! Use keys to control the arm...\n")

    # Current angles (radians)
    shoulder = 0.0
    elbow = 0.0
    step_size = 0.1  # radians (~5.7 degrees)
    
    # Send initial position
    packet = encode_set_joint_angles(shoulder, elbow)
    manager._serial.write(packet)
    time.sleep(0.1)

    try:
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8').lower()
                
                if key == 'q':
                    print("\nQuitting...")
                    break
                    
                elif key == 'w':
                    shoulder = min(shoulder + step_size, 1.5)  # Max ~85 degrees
                    print(f"Shoulder UP:   {shoulder:.2f} rad ({shoulder*57.3:.1f}deg)")
                    
                elif key == 's':
                    shoulder = max(shoulder - step_size, -1.5)  # Min ~-85 degrees
                    print(f"Shoulder DOWN: {shoulder:.2f} rad ({shoulder*57.3:.1f}deg)")
                    
                elif key == 'd':
                    elbow = min(elbow + step_size, 1.5)  # Max ~85 degrees
                    print(f"Elbow RIGHT:   {elbow:.2f} rad ({elbow*57.3:.1f}deg)")
                    
                elif key == 'a':
                    elbow = max(elbow - step_size, -1.5)  # Min ~-85 degrees
                    print(f"Elbow LEFT:    {elbow:.2f} rad ({elbow*57.3:.1f}deg)")
                    
                elif key == 'r':
                    shoulder = 0.0
                    elbow = 0.0
                    print(f"RESET to home: (0.0, 0.0)")
                
                # Send command
                try:
                    packet = encode_set_joint_angles(shoulder, elbow)
                    manager._serial.write(packet)
                except ValueError as e:
                    print(f"  ERROR: {e}")
                    
            time.sleep(0.05)  # Small delay to prevent CPU spinning
            
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    # Return to home
    print("\nReturning to home position...")
    packet = encode_set_joint_angles(0.0, 0.0)
    manager._serial.write(packet)
    time.sleep(0.5)
    
    manager.disconnect()
    print("Disconnected")
    return 0


if __name__ == "__main__":
    sys.exit(main())
