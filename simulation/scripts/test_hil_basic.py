"""Basic HIL test: Manual servo control."""

import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

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
