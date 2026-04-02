"""Launch HIL synchronizer with real-time dashboard."""

import sys
import argparse
from pathlib import Path
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from middleware.hil_synchronizer import HILSynchronizer
from visualization.dashboard import Dashboard


def parse_args():
    parser = argparse.ArgumentParser(description="HIL Dashboard")
    parser.add_argument("port", nargs="?", default="COM3", help="Serial port")
    parser.add_argument(
        "--model", default="mujoco_model/arm.xml", help="MuJoCo model path"
    )
    parser.add_argument("--rate", type=float, default=25.0, help="Update rate (Hz)")
    return parser.parse_args()


def main():
    args = parse_args()

    app = QApplication(sys.argv)

    # Create dashboard
    dashboard = Dashboard()
    dashboard.show()

    # Model path
    model_path = ROOT_DIR / args.model
    if not model_path.exists():
        print(f"ERROR: Model not found: {model_path}")
        return 1

    # Create HIL synchronizer with dashboard callback
    hil = HILSynchronizer(
        port=args.port,
        model_path=str(model_path),
        update_rate=args.rate,
        telemetry_callback=dashboard.update_state,
    )

    # Connect
    try:
        if not hil.serial.connect():
            print(f"ERROR: Failed to connect to {args.port}")
            dashboard.set_connected(False)
        else:
            dashboard.set_connected(True)
            print(f"Connected to {args.port}")
    except Exception as e:
        print(f"Connection error: {e}")
        dashboard.set_connected(False)

    # Run HIL in timer (non-blocking for Qt event loop)
    def hil_step():
        if hil.serial.is_connected():
            hil._send_commands()
            telemetry = hil.serial.receive_telemetry(timeout=0.01)
            if telemetry:
                hil._process_telemetry(telemetry)

    hil_timer = QTimer()
    hil_timer.timeout.connect(hil_step)
    hil_timer.start(int(1000 / args.rate))

    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
