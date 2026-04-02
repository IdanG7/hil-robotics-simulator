"""PyQt5 real-time dashboard for HIL robotics simulator."""

import sys
import numpy as np
from typing import Optional, Dict
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QGroupBox,
    QGridLayout,
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
import pyqtgraph as pg


class Dashboard(QMainWindow):
    """Real-time dashboard displaying arm state and telemetry.

    Layout:
    ┌─────────────────┬─────────────────┐
    │   Arm Visual    │   Joint Angles  │
    │   (2D plot)     │   (text + plot) │
    ├─────────────────┼─────────────────┤
    │   IMU Data      │   System Status │
    │   (plots)       │   (text)        │
    └─────────────────┴─────────────────┘
    """

    def __init__(self, L1: float = 0.1, L2: float = 0.08):
        """Initialize dashboard.

        Args:
            L1: Shoulder link length (meters)
            L2: Elbow link length (meters)
        """
        super().__init__()
        self.L1 = L1
        self.L2 = L2

        # State
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.latency_ms = 0.0
        self.packet_count = 0

        # History buffers for plots (last 10 seconds at 50Hz = 500 points)
        self.history_len = 500
        self.angle_history = {
            "shoulder": np.zeros(self.history_len),
            "elbow": np.zeros(self.history_len),
        }
        self.imu_history = {
            "roll": np.zeros(self.history_len),
            "pitch": np.zeros(self.history_len),
        }
        self.time_axis = np.linspace(-10, 0, self.history_len)

        self._setup_ui()

        # Update timer (30 FPS)
        self.timer = QTimer()
        self.timer.timeout.connect(self._update_plots)
        self.timer.start(33)  # ~30 FPS

    def _setup_ui(self):
        """Create the UI layout."""
        self.setWindowTitle("HIL Robotics Simulator - Dashboard")
        self.setGeometry(100, 100, 1200, 800)

        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        layout = QGridLayout(central)

        # 1. Arm visualization (top-left)
        arm_group = QGroupBox("Arm Position")
        arm_layout = QVBoxLayout(arm_group)
        self.arm_plot = pg.PlotWidget()
        self.arm_plot.setAspectLocked(True)
        self.arm_plot.setXRange(-0.2, 0.2)
        self.arm_plot.setYRange(-0.1, 0.25)
        self.arm_plot.setLabel("bottom", "X (m)")
        self.arm_plot.setLabel("left", "Y (m)")
        self.arm_line = self.arm_plot.plot(pen=pg.mkPen("r", width=3))
        self.arm_joints = self.arm_plot.plot(
            pen=None, symbol="o", symbolSize=10, symbolBrush="b"
        )
        arm_layout.addWidget(self.arm_plot)
        layout.addWidget(arm_group, 0, 0)

        # 2. Joint angles (top-right)
        angles_group = QGroupBox("Joint Angles")
        angles_layout = QVBoxLayout(angles_group)

        # Text display
        self.shoulder_label = QLabel("Shoulder: 0.0°")
        self.elbow_label = QLabel("Elbow: 0.0°")
        self.shoulder_label.setFont(QFont("Monospace", 14))
        self.elbow_label.setFont(QFont("Monospace", 14))
        angles_layout.addWidget(self.shoulder_label)
        angles_layout.addWidget(self.elbow_label)

        # Time series plot
        self.angles_plot = pg.PlotWidget()
        self.angles_plot.setLabel("bottom", "Time (s)")
        self.angles_plot.setLabel("left", "Angle (°)")
        self.angles_plot.addLegend()
        self.shoulder_curve = self.angles_plot.plot(pen="r", name="Shoulder")
        self.elbow_curve = self.angles_plot.plot(pen="g", name="Elbow")
        angles_layout.addWidget(self.angles_plot)
        layout.addWidget(angles_group, 0, 1)

        # 3. IMU data (bottom-left)
        imu_group = QGroupBox("IMU Orientation")
        imu_layout = QVBoxLayout(imu_group)

        self.roll_label = QLabel("Roll: 0.0°")
        self.pitch_label = QLabel("Pitch: 0.0°")
        self.roll_label.setFont(QFont("Monospace", 14))
        self.pitch_label.setFont(QFont("Monospace", 14))
        imu_layout.addWidget(self.roll_label)
        imu_layout.addWidget(self.pitch_label)

        self.imu_plot = pg.PlotWidget()
        self.imu_plot.setLabel("bottom", "Time (s)")
        self.imu_plot.setLabel("left", "Angle (°)")
        self.imu_plot.addLegend()
        self.roll_curve = self.imu_plot.plot(pen="c", name="Roll")
        self.pitch_curve = self.imu_plot.plot(pen="m", name="Pitch")
        imu_layout.addWidget(self.imu_plot)
        layout.addWidget(imu_group, 1, 0)

        # 4. System status (bottom-right)
        status_group = QGroupBox("System Status")
        status_layout = QVBoxLayout(status_group)

        self.latency_label = QLabel("Latency: 0 ms")
        self.packets_label = QLabel("Packets: 0")
        self.status_label = QLabel("Status: Disconnected")

        for label in [self.latency_label, self.packets_label, self.status_label]:
            label.setFont(QFont("Monospace", 12))
            status_layout.addWidget(label)

        status_layout.addStretch()
        layout.addWidget(status_group, 1, 1)

    def update_state(
        self,
        theta1: float,
        theta2: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        latency_ms: float = 0.0,
    ):
        """Update dashboard with new telemetry data.

        Args:
            theta1: Shoulder angle (radians)
            theta2: Elbow angle (radians)
            roll: IMU roll angle (radians)
            pitch: IMU pitch angle (radians)
            latency_ms: Communication latency (milliseconds)
        """
        self.theta1 = theta1
        self.theta2 = theta2
        self.imu_roll = roll
        self.imu_pitch = pitch
        self.latency_ms = latency_ms
        self.packet_count += 1

        # Update history (shift left, add new value at end)
        self.angle_history["shoulder"] = np.roll(self.angle_history["shoulder"], -1)
        self.angle_history["shoulder"][-1] = np.degrees(theta1)
        self.angle_history["elbow"] = np.roll(self.angle_history["elbow"], -1)
        self.angle_history["elbow"][-1] = np.degrees(theta2)

        self.imu_history["roll"] = np.roll(self.imu_history["roll"], -1)
        self.imu_history["roll"][-1] = np.degrees(roll)
        self.imu_history["pitch"] = np.roll(self.imu_history["pitch"], -1)
        self.imu_history["pitch"][-1] = np.degrees(pitch)

    def _update_plots(self):
        """Refresh all plots (called by timer)."""
        # Update arm visualization
        x0, y0 = 0, 0  # Base
        x1 = self.L1 * np.cos(self.theta1)
        y1 = self.L1 * np.sin(self.theta1)
        x2 = x1 + self.L2 * np.cos(self.theta1 + self.theta2)
        y2 = y1 + self.L2 * np.sin(self.theta1 + self.theta2)

        self.arm_line.setData([x0, x1, x2], [y0, y1, y2])
        self.arm_joints.setData([x0, x1, x2], [y0, y1, y2])

        # Update labels
        self.shoulder_label.setText(f"Shoulder: {np.degrees(self.theta1):+6.1f}°")
        self.elbow_label.setText(f"Elbow: {np.degrees(self.theta2):+6.1f}°")
        self.roll_label.setText(f"Roll: {np.degrees(self.imu_roll):+6.1f}°")
        self.pitch_label.setText(f"Pitch: {np.degrees(self.imu_pitch):+6.1f}°")
        self.latency_label.setText(f"Latency: {self.latency_ms:.1f} ms")
        self.packets_label.setText(f"Packets: {self.packet_count}")

        # Update time series plots
        self.shoulder_curve.setData(self.time_axis, self.angle_history["shoulder"])
        self.elbow_curve.setData(self.time_axis, self.angle_history["elbow"])
        self.roll_curve.setData(self.time_axis, self.imu_history["roll"])
        self.pitch_curve.setData(self.time_axis, self.imu_history["pitch"])

    def set_connected(self, connected: bool):
        """Update connection status display."""
        if connected:
            self.status_label.setText("Status: Connected")
            self.status_label.setStyleSheet("color: green")
        else:
            self.status_label.setText("Status: Disconnected")
            self.status_label.setStyleSheet("color: red")


def run_dashboard():
    """Launch standalone dashboard for testing."""
    app = QApplication(sys.argv)
    dashboard = Dashboard()
    dashboard.show()

    # Demo: animate with sine wave
    t = [0]

    def animate():
        t[0] += 0.05
        theta1 = 0.5 * np.sin(t[0])
        theta2 = 0.3 * np.sin(t[0] * 1.5)
        dashboard.update_state(theta1, theta2, theta1 * 0.5, theta2 * 0.3, 25.0)

    demo_timer = QTimer()
    demo_timer.timeout.connect(animate)
    demo_timer.start(50)

    sys.exit(app.exec_())


if __name__ == "__main__":
    run_dashboard()
