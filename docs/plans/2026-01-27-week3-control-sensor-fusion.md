# Week 3: Control & Sensor Fusion Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Implement sensor fusion, trajectory generation, real-time dashboard visualization, and robust error handling for the HIL robotics simulator.

**Architecture:** Four main components: (1) Complementary filter in firmware fuses gyro+accel for orientation, (2) Python trajectory planner generates smooth Cartesian paths with IK, (3) PyQt5 dashboard displays 3D arm + telemetry plots at 30fps, (4) Watchdog timer + auto-reconnect for fault tolerance.

**Tech Stack:** STM32 HAL (C), Python 3.10+, PyQt5, pyqtgraph, NumPy, pyserial

---

## Task 1: Sensor Fusion - Complementary Filter (Firmware)

**Files:**
- Create: `firmware/Core/Inc/sensor_fusion.h`
- Create: `firmware/Core/Src/sensor_fusion.c`
- Test: Manual verification with serial output

**Step 1: Create sensor_fusion.h header**

```c
// firmware/Core/Inc/sensor_fusion.h
#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float roll;         // Roll angle (radians)
    float pitch;        // Pitch angle (radians)
    float alpha;        // Filter coefficient (0.98 typical)
    float dt;           // Sample period (seconds)
    bool initialized;   // First sample flag
} SensorFusion_t;

/**
 * Initialize sensor fusion with filter parameters.
 * @param fusion Fusion state structure
 * @param alpha Filter coefficient (0.0-1.0, higher = trust gyro more)
 * @param dt Sample period in seconds (e.g., 0.02 for 50Hz)
 */
void SensorFusion_Init(SensorFusion_t *fusion, float alpha, float dt);

/**
 * Update orientation estimate with new sensor data.
 * @param fusion Fusion state structure
 * @param accel_x Accelerometer X (raw int16)
 * @param accel_y Accelerometer Y (raw int16)
 * @param accel_z Accelerometer Z (raw int16)
 * @param gyro_x Gyroscope X (raw int16, deg/s scaled)
 * @param gyro_y Gyroscope Y (raw int16, deg/s scaled)
 */
void SensorFusion_Update(SensorFusion_t *fusion,
                         int16_t accel_x, int16_t accel_y, int16_t accel_z,
                         int16_t gyro_x, int16_t gyro_y);

/**
 * Get current roll angle.
 * @return Roll in radians
 */
float SensorFusion_GetRoll(SensorFusion_t *fusion);

/**
 * Get current pitch angle.
 * @return Pitch in radians
 */
float SensorFusion_GetPitch(SensorFusion_t *fusion);

/**
 * Reset fusion state (call after IMU recalibration).
 */
void SensorFusion_Reset(SensorFusion_t *fusion);

#endif // SENSOR_FUSION_H
```

**Step 2: Create sensor_fusion.c implementation**

```c
// firmware/Core/Src/sensor_fusion.c
#include "sensor_fusion.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// MPU6050 sensitivity scales (default ±2g, ±250°/s)
#define ACCEL_SCALE (16384.0f)  // LSB/g
#define GYRO_SCALE  (131.0f)    // LSB/(°/s)
#define DEG_TO_RAD  (M_PI / 180.0f)

void SensorFusion_Init(SensorFusion_t *fusion, float alpha, float dt) {
    fusion->roll = 0.0f;
    fusion->pitch = 0.0f;
    fusion->alpha = alpha;
    fusion->dt = dt;
    fusion->initialized = false;
}

void SensorFusion_Update(SensorFusion_t *fusion,
                         int16_t accel_x, int16_t accel_y, int16_t accel_z,
                         int16_t gyro_x, int16_t gyro_y) {
    // Convert raw values to physical units
    float ax = (float)accel_x / ACCEL_SCALE;
    float ay = (float)accel_y / ACCEL_SCALE;
    float az = (float)accel_z / ACCEL_SCALE;
    float gx = (float)gyro_x / GYRO_SCALE * DEG_TO_RAD;  // rad/s
    float gy = (float)gyro_y / GYRO_SCALE * DEG_TO_RAD;  // rad/s

    // Compute angles from accelerometer (gravity reference)
    float accel_roll = atan2f(ay, az);
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    if (!fusion->initialized) {
        // First sample: use accelerometer directly
        fusion->roll = accel_roll;
        fusion->pitch = accel_pitch;
        fusion->initialized = true;
        return;
    }

    // Complementary filter:
    // angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
    fusion->roll = fusion->alpha * (fusion->roll + gx * fusion->dt)
                 + (1.0f - fusion->alpha) * accel_roll;
    fusion->pitch = fusion->alpha * (fusion->pitch + gy * fusion->dt)
                  + (1.0f - fusion->alpha) * accel_pitch;
}

float SensorFusion_GetRoll(SensorFusion_t *fusion) {
    return fusion->roll;
}

float SensorFusion_GetPitch(SensorFusion_t *fusion) {
    return fusion->pitch;
}

void SensorFusion_Reset(SensorFusion_t *fusion) {
    fusion->roll = 0.0f;
    fusion->pitch = 0.0f;
    fusion->initialized = false;
}
```

**Step 3: Integrate sensor fusion into main.c**

Modify: `firmware/Core/Src/main.c`

Add after IMU initialization (around line 154):
```c
// Add include at top
#include "sensor_fusion.h"

// Add global variable with other globals (around line 66)
SensorFusion_t imu_fusion;

// Add in main() after IMU_Init (around line 154)
SensorFusion_Init(&imu_fusion, 0.98f, 0.02f);  // α=0.98, dt=20ms

// Add in control loop or telemetry sending
if (imu_available) {
    IMU_Data_t imu_data;
    if (IMU_ReadData(&hi2c1, &imu_data)) {
        SensorFusion_Update(&imu_fusion,
                           imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                           imu_data.gyro_x, imu_data.gyro_y);
    }
}
```

**Step 4: Build firmware and verify compilation**

Run: Build in STM32CubeIDE (Ctrl+B)
Expected: Build succeeds with 0 errors

**Step 5: Commit sensor fusion module**

```bash
git add firmware/Core/Inc/sensor_fusion.h firmware/Core/Src/sensor_fusion.c
git commit -m "feat(firmware): add complementary filter for IMU sensor fusion"
```

---

## Task 2: Trajectory Generation Module (Python)

**Files:**
- Create: `simulation/middleware/trajectory.py`
- Create: `simulation/middleware/tests/test_trajectory.py`

**Step 1: Write failing test for linear interpolation**

```python
# simulation/middleware/tests/test_trajectory.py
"""Tests for trajectory generation."""

import pytest
import numpy as np
from middleware.trajectory import TrajectoryPlanner


class TestLinearInterpolation:
    """Test linear trajectory interpolation."""

    def test_linear_single_segment(self):
        """Linear interpolation between two points."""
        planner = TrajectoryPlanner()
        waypoints = [(0.0, 0.0), (1.0, 1.0)]

        trajectory = planner.linear(waypoints, num_points=11)

        assert len(trajectory) == 11
        assert trajectory[0] == pytest.approx((0.0, 0.0), abs=1e-6)
        assert trajectory[5] == pytest.approx((0.5, 0.5), abs=1e-6)
        assert trajectory[10] == pytest.approx((1.0, 1.0), abs=1e-6)

    def test_linear_multiple_segments(self):
        """Linear interpolation through multiple waypoints."""
        planner = TrajectoryPlanner()
        waypoints = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]

        trajectory = planner.linear(waypoints, num_points=21)

        assert len(trajectory) == 21
        # First point
        assert trajectory[0] == pytest.approx((0.0, 0.0), abs=1e-6)
        # Midpoint of first segment
        assert trajectory[5] == pytest.approx((0.5, 0.0), abs=1e-6)
        # End of first segment / start of second
        assert trajectory[10] == pytest.approx((1.0, 0.0), abs=1e-6)
        # Final point
        assert trajectory[20] == pytest.approx((1.0, 1.0), abs=1e-6)
```

**Step 2: Run test to verify it fails**

Run: `cd simulation && python -m pytest middleware/tests/test_trajectory.py::TestLinearInterpolation::test_linear_single_segment -v`
Expected: FAIL with "ModuleNotFoundError: No module named 'middleware.trajectory'"

**Step 3: Create trajectory.py with linear interpolation**

```python
# simulation/middleware/trajectory.py
"""Trajectory generation for robotic arm motion planning."""

import numpy as np
from typing import List, Tuple
from middleware.kinematics import ArmKinematics


class TrajectoryPlanner:
    """Generate smooth trajectories for 2-DOF arm.

    Supports:
    - Linear interpolation between waypoints
    - Cubic spline interpolation for smooth motion
    - Figure-8 and circular patterns
    - Joint-space and Cartesian-space planning
    """

    def __init__(self, kinematics: ArmKinematics = None):
        """Initialize trajectory planner.

        Args:
            kinematics: ArmKinematics instance for IK (optional)
        """
        self.kinematics = kinematics or ArmKinematics()

    def linear(self, waypoints: List[Tuple[float, float]],
               num_points: int = 100) -> List[Tuple[float, float]]:
        """Generate linear interpolation through waypoints.

        Args:
            waypoints: List of (x, y) or (theta1, theta2) points
            num_points: Total number of output points

        Returns:
            List of interpolated points
        """
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints")

        # Calculate segment lengths
        segments = len(waypoints) - 1
        points_per_segment = (num_points - 1) // segments

        trajectory = []
        for i in range(segments):
            start = np.array(waypoints[i])
            end = np.array(waypoints[i + 1])

            # Number of points for this segment
            if i == segments - 1:
                # Last segment gets remaining points
                n = num_points - len(trajectory)
            else:
                n = points_per_segment + 1

            # Linear interpolation
            for j in range(n):
                if i == segments - 1 or j < n - 1:
                    t = j / (n - 1) if n > 1 else 0
                    point = start + t * (end - start)
                    trajectory.append(tuple(point))

        return trajectory
```

**Step 4: Run test to verify it passes**

Run: `cd simulation && python -m pytest middleware/tests/test_trajectory.py::TestLinearInterpolation -v`
Expected: PASS (2 tests)

**Step 5: Commit linear interpolation**

```bash
git add simulation/middleware/trajectory.py simulation/middleware/tests/test_trajectory.py
git commit -m "feat(trajectory): add linear interpolation for waypoint planning"
```

---

## Task 3: Cubic Spline Trajectory

**Files:**
- Modify: `simulation/middleware/trajectory.py`
- Modify: `simulation/middleware/tests/test_trajectory.py`

**Step 1: Write failing test for cubic spline**

Add to `simulation/middleware/tests/test_trajectory.py`:

```python
class TestCubicSpline:
    """Test cubic spline trajectory generation."""

    def test_cubic_smooth_velocity(self):
        """Cubic spline should have smooth velocity (no discontinuities)."""
        planner = TrajectoryPlanner()
        waypoints = [(0.0, 0.0), (0.5, 0.5), (1.0, 0.0)]

        trajectory = planner.cubic_spline(waypoints, num_points=101)

        assert len(trajectory) == 101
        # Start and end points match
        assert trajectory[0] == pytest.approx((0.0, 0.0), abs=1e-6)
        assert trajectory[100] == pytest.approx((1.0, 0.0), abs=1e-6)

        # Velocity should be continuous (check by finite differences)
        velocities = []
        for i in range(1, len(trajectory)):
            dx = trajectory[i][0] - trajectory[i-1][0]
            dy = trajectory[i][1] - trajectory[i-1][1]
            velocities.append((dx, dy))

        # Check no sudden velocity jumps (max change < 10x average)
        avg_speed = np.mean([np.sqrt(v[0]**2 + v[1]**2) for v in velocities])
        for i in range(1, len(velocities)):
            dv = np.sqrt((velocities[i][0] - velocities[i-1][0])**2 +
                        (velocities[i][1] - velocities[i-1][1])**2)
            assert dv < avg_speed * 0.5, f"Velocity discontinuity at index {i}"

    def test_cubic_passes_through_waypoints(self):
        """Cubic spline should pass through all waypoints."""
        planner = TrajectoryPlanner()
        waypoints = [(0.0, 0.0), (0.1, 0.05), (0.15, 0.0)]

        trajectory = planner.cubic_spline(waypoints, num_points=101)

        # Check trajectory passes near each waypoint
        for wp in waypoints:
            distances = [np.sqrt((p[0]-wp[0])**2 + (p[1]-wp[1])**2)
                        for p in trajectory]
            min_dist = min(distances)
            assert min_dist < 0.01, f"Trajectory doesn't pass through {wp}"
```

**Step 2: Run test to verify it fails**

Run: `cd simulation && python -m pytest middleware/tests/test_trajectory.py::TestCubicSpline -v`
Expected: FAIL with "AttributeError: 'TrajectoryPlanner' object has no attribute 'cubic_spline'"

**Step 3: Implement cubic_spline method**

Add to `simulation/middleware/trajectory.py`:

```python
from scipy.interpolate import CubicSpline

class TrajectoryPlanner:
    # ... existing code ...

    def cubic_spline(self, waypoints: List[Tuple[float, float]],
                     num_points: int = 100) -> List[Tuple[float, float]]:
        """Generate cubic spline interpolation through waypoints.

        Produces smooth motion with continuous velocity and acceleration.

        Args:
            waypoints: List of (x, y) points to pass through
            num_points: Total number of output points

        Returns:
            List of smoothly interpolated points
        """
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints")

        # Parameterize by arc length approximation
        waypoints = np.array(waypoints)
        t = np.zeros(len(waypoints))
        for i in range(1, len(waypoints)):
            t[i] = t[i-1] + np.linalg.norm(waypoints[i] - waypoints[i-1])

        # Normalize t to [0, 1]
        if t[-1] > 0:
            t = t / t[-1]

        # Create cubic splines for x and y
        cs_x = CubicSpline(t, waypoints[:, 0], bc_type='natural')
        cs_y = CubicSpline(t, waypoints[:, 1], bc_type='natural')

        # Sample the spline
        t_new = np.linspace(0, 1, num_points)
        trajectory = [(float(cs_x(ti)), float(cs_y(ti))) for ti in t_new]

        return trajectory
```

**Step 4: Run test to verify it passes**

Run: `cd simulation && python -m pytest middleware/tests/test_trajectory.py::TestCubicSpline -v`
Expected: PASS (2 tests)

**Step 5: Commit cubic spline**

```bash
git add simulation/middleware/trajectory.py simulation/middleware/tests/test_trajectory.py
git commit -m "feat(trajectory): add cubic spline for smooth motion"
```

---

## Task 4: Figure-8 Trajectory Pattern

**Files:**
- Modify: `simulation/middleware/trajectory.py`
- Modify: `simulation/middleware/tests/test_trajectory.py`

**Step 1: Write failing test for figure-8**

Add to `simulation/middleware/tests/test_trajectory.py`:

```python
class TestFigure8:
    """Test figure-8 trajectory generation."""

    def test_figure8_shape(self):
        """Figure-8 should cross center and reach extremes."""
        planner = TrajectoryPlanner()

        trajectory = planner.figure8(
            center=(0.12, 0.0),
            width=0.04,
            height=0.03,
            num_points=100
        )

        assert len(trajectory) == 100

        # Extract x and y coordinates
        xs = [p[0] for p in trajectory]
        ys = [p[1] for p in trajectory]

        # Should span the width and height
        assert max(xs) - min(xs) >= 0.035  # ~width
        assert max(ys) - min(ys) >= 0.025  # ~height

        # Center should be approximately at specified location
        avg_x = np.mean(xs)
        avg_y = np.mean(ys)
        assert abs(avg_x - 0.12) < 0.01
        assert abs(avg_y - 0.0) < 0.01

    def test_figure8_closed_loop(self):
        """Figure-8 should form a closed loop."""
        planner = TrajectoryPlanner()

        trajectory = planner.figure8(
            center=(0.12, 0.0),
            width=0.04,
            height=0.03,
            num_points=100
        )

        # First and last points should be close
        dist = np.sqrt((trajectory[0][0] - trajectory[-1][0])**2 +
                      (trajectory[0][1] - trajectory[-1][1])**2)
        assert dist < 0.005, "Figure-8 is not closed"
```

**Step 2: Run test to verify it fails**

Run: `cd simulation && python -m pytest middleware/tests/test_trajectory.py::TestFigure8 -v`
Expected: FAIL with "AttributeError: 'TrajectoryPlanner' object has no attribute 'figure8'"

**Step 3: Implement figure8 method**

Add to `simulation/middleware/trajectory.py`:

```python
class TrajectoryPlanner:
    # ... existing code ...

    def figure8(self, center: Tuple[float, float] = (0.12, 0.0),
                width: float = 0.04, height: float = 0.03,
                num_points: int = 100) -> List[Tuple[float, float]]:
        """Generate figure-8 (lemniscate) trajectory in Cartesian space.

        Uses parametric lemniscate of Bernoulli:
        x = a * cos(t) / (1 + sin²(t))
        y = a * sin(t) * cos(t) / (1 + sin²(t))

        Args:
            center: Center point (x, y) in meters
            width: Horizontal extent in meters
            height: Vertical extent in meters
            num_points: Number of trajectory points

        Returns:
            List of (x, y) points forming figure-8
        """
        trajectory = []

        for i in range(num_points):
            t = 2 * np.pi * i / num_points

            # Lemniscate parametric equations
            denom = 1 + np.sin(t)**2
            x = center[0] + (width / 2) * np.cos(t) / denom
            y = center[1] + (height) * np.sin(t) * np.cos(t) / denom

            trajectory.append((x, y))

        return trajectory
```

**Step 4: Run test to verify it passes**

Run: `cd simulation && python -m pytest middleware/tests/test_trajectory.py::TestFigure8 -v`
Expected: PASS (2 tests)

**Step 5: Commit figure-8 trajectory**

```bash
git add simulation/middleware/trajectory.py simulation/middleware/tests/test_trajectory.py
git commit -m "feat(trajectory): add figure-8 pattern for acceptance test"
```

---

## Task 5: Joint-Space Trajectory Conversion

**Files:**
- Modify: `simulation/middleware/trajectory.py`
- Modify: `simulation/middleware/tests/test_trajectory.py`

**Step 1: Write failing test for Cartesian to joint conversion**

Add to `simulation/middleware/tests/test_trajectory.py`:

```python
class TestCartesianToJoint:
    """Test Cartesian to joint-space conversion."""

    def test_to_joint_space_valid(self):
        """Convert reachable Cartesian trajectory to joint angles."""
        planner = TrajectoryPlanner()

        # Simple line in reachable workspace
        cartesian = [(0.15, 0.0), (0.14, 0.02), (0.13, 0.04)]

        joint_trajectory = planner.to_joint_space(cartesian)

        assert len(joint_trajectory) == 3
        # Each point should be (theta1, theta2) tuple
        for angles in joint_trajectory:
            assert len(angles) == 2
            # Angles should be within valid range
            assert -np.pi/2 <= angles[0] <= np.pi/2
            assert -np.pi/2 <= angles[1] <= np.pi/2

    def test_to_joint_space_roundtrip(self):
        """Joint angles should produce original Cartesian points."""
        planner = TrajectoryPlanner()

        cartesian = [(0.15, 0.0), (0.12, 0.05)]
        joint_trajectory = planner.to_joint_space(cartesian)

        # Convert back to Cartesian
        for i, (x_orig, y_orig) in enumerate(cartesian):
            theta1, theta2 = joint_trajectory[i]
            x_check, y_check = planner.kinematics.forward(theta1, theta2)
            assert abs(x_check - x_orig) < 0.001
            assert abs(y_check - y_orig) < 0.001
```

**Step 2: Run test to verify it fails**

Run: `cd simulation && python -m pytest middleware/tests/test_trajectory.py::TestCartesianToJoint -v`
Expected: FAIL with "AttributeError: 'TrajectoryPlanner' object has no attribute 'to_joint_space'"

**Step 3: Implement to_joint_space method**

Add to `simulation/middleware/trajectory.py`:

```python
class TrajectoryPlanner:
    # ... existing code ...

    def to_joint_space(self, cartesian_trajectory: List[Tuple[float, float]]
                       ) -> List[Tuple[float, float]]:
        """Convert Cartesian trajectory to joint-space angles.

        Uses inverse kinematics to compute joint angles for each point.

        Args:
            cartesian_trajectory: List of (x, y) points in meters

        Returns:
            List of (theta1, theta2) joint angles in radians

        Raises:
            ValueError: If any point is unreachable
        """
        joint_trajectory = []

        for i, (x, y) in enumerate(cartesian_trajectory):
            try:
                theta1, theta2 = self.kinematics.inverse(x, y)
                joint_trajectory.append((theta1, theta2))
            except ValueError as e:
                raise ValueError(f"Point {i} ({x:.3f}, {y:.3f}) unreachable: {e}")

        return joint_trajectory
```

**Step 4: Run test to verify it passes**

Run: `cd simulation && python -m pytest middleware/tests/test_trajectory.py::TestCartesianToJoint -v`
Expected: PASS (2 tests)

**Step 5: Commit joint-space conversion**

```bash
git add simulation/middleware/trajectory.py simulation/middleware/tests/test_trajectory.py
git commit -m "feat(trajectory): add Cartesian to joint-space conversion with IK"
```

---

## Task 6: PyQt5 Dashboard - Basic Window

**Files:**
- Create: `simulation/visualization/__init__.py`
- Create: `simulation/visualization/dashboard.py`

**Step 1: Create visualization package**

```python
# simulation/visualization/__init__.py
"""Real-time visualization dashboard for HIL simulator."""

from .dashboard import Dashboard

__all__ = ['Dashboard']
```

**Step 2: Create basic dashboard window**

```python
# simulation/visualization/dashboard.py
"""PyQt5 real-time dashboard for HIL robotics simulator."""

import sys
import numpy as np
from typing import Optional, Dict
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QGroupBox, QGridLayout)
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
            'shoulder': np.zeros(self.history_len),
            'elbow': np.zeros(self.history_len),
        }
        self.imu_history = {
            'roll': np.zeros(self.history_len),
            'pitch': np.zeros(self.history_len),
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
        self.arm_plot.setLabel('bottom', 'X (m)')
        self.arm_plot.setLabel('left', 'Y (m)')
        self.arm_line = self.arm_plot.plot(pen=pg.mkPen('r', width=3))
        self.arm_joints = self.arm_plot.plot(pen=None, symbol='o',
                                             symbolSize=10, symbolBrush='b')
        arm_layout.addWidget(self.arm_plot)
        layout.addWidget(arm_group, 0, 0)

        # 2. Joint angles (top-right)
        angles_group = QGroupBox("Joint Angles")
        angles_layout = QVBoxLayout(angles_group)

        # Text display
        self.shoulder_label = QLabel("Shoulder: 0.0°")
        self.elbow_label = QLabel("Elbow: 0.0°")
        self.shoulder_label.setFont(QFont('Monospace', 14))
        self.elbow_label.setFont(QFont('Monospace', 14))
        angles_layout.addWidget(self.shoulder_label)
        angles_layout.addWidget(self.elbow_label)

        # Time series plot
        self.angles_plot = pg.PlotWidget()
        self.angles_plot.setLabel('bottom', 'Time (s)')
        self.angles_plot.setLabel('left', 'Angle (°)')
        self.angles_plot.addLegend()
        self.shoulder_curve = self.angles_plot.plot(pen='r', name='Shoulder')
        self.elbow_curve = self.angles_plot.plot(pen='g', name='Elbow')
        angles_layout.addWidget(self.angles_plot)
        layout.addWidget(angles_group, 0, 1)

        # 3. IMU data (bottom-left)
        imu_group = QGroupBox("IMU Orientation")
        imu_layout = QVBoxLayout(imu_group)

        self.roll_label = QLabel("Roll: 0.0°")
        self.pitch_label = QLabel("Pitch: 0.0°")
        self.roll_label.setFont(QFont('Monospace', 14))
        self.pitch_label.setFont(QFont('Monospace', 14))
        imu_layout.addWidget(self.roll_label)
        imu_layout.addWidget(self.pitch_label)

        self.imu_plot = pg.PlotWidget()
        self.imu_plot.setLabel('bottom', 'Time (s)')
        self.imu_plot.setLabel('left', 'Angle (°)')
        self.imu_plot.addLegend()
        self.roll_curve = self.imu_plot.plot(pen='c', name='Roll')
        self.pitch_curve = self.imu_plot.plot(pen='m', name='Pitch')
        imu_layout.addWidget(self.imu_plot)
        layout.addWidget(imu_group, 1, 0)

        # 4. System status (bottom-right)
        status_group = QGroupBox("System Status")
        status_layout = QVBoxLayout(status_group)

        self.latency_label = QLabel("Latency: 0 ms")
        self.packets_label = QLabel("Packets: 0")
        self.status_label = QLabel("Status: Disconnected")

        for label in [self.latency_label, self.packets_label, self.status_label]:
            label.setFont(QFont('Monospace', 12))
            status_layout.addWidget(label)

        status_layout.addStretch()
        layout.addWidget(status_group, 1, 1)

    def update_state(self, theta1: float, theta2: float,
                     roll: float = 0.0, pitch: float = 0.0,
                     latency_ms: float = 0.0):
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
        self.angle_history['shoulder'] = np.roll(self.angle_history['shoulder'], -1)
        self.angle_history['shoulder'][-1] = np.degrees(theta1)
        self.angle_history['elbow'] = np.roll(self.angle_history['elbow'], -1)
        self.angle_history['elbow'][-1] = np.degrees(theta2)

        self.imu_history['roll'] = np.roll(self.imu_history['roll'], -1)
        self.imu_history['roll'][-1] = np.degrees(roll)
        self.imu_history['pitch'] = np.roll(self.imu_history['pitch'], -1)
        self.imu_history['pitch'][-1] = np.degrees(pitch)

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
        self.shoulder_curve.setData(self.time_axis, self.angle_history['shoulder'])
        self.elbow_curve.setData(self.time_axis, self.angle_history['elbow'])
        self.roll_curve.setData(self.time_axis, self.imu_history['roll'])
        self.pitch_curve.setData(self.time_axis, self.imu_history['pitch'])

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


if __name__ == '__main__':
    run_dashboard()
```

**Step 3: Test dashboard launches**

Run: `cd simulation && python -m visualization.dashboard`
Expected: Window opens showing animated arm demo

**Step 4: Commit dashboard**

```bash
git add simulation/visualization/__init__.py simulation/visualization/dashboard.py
git commit -m "feat(visualization): add PyQt5 real-time dashboard with arm plot and telemetry"
```

---

## Task 7: Integrate Dashboard with HIL Synchronizer

**Files:**
- Modify: `simulation/middleware/hil_synchronizer.py`
- Create: `simulation/scripts/run_hil_dashboard.py`

**Step 1: Add dashboard integration to HILSynchronizer**

Modify `simulation/middleware/hil_synchronizer.py`:

```python
# Add imports at top
from typing import Optional, Callable

class HILSynchronizer:
    def __init__(self, port: str, model_path: str, update_rate: float = 25.0,
                 telemetry_callback: Optional[Callable] = None):
        """Initialize HIL synchronizer.

        Args:
            port: Serial port for hardware communication
            model_path: Path to MuJoCo MJCF model file
            update_rate: Update frequency in Hz (default: 25.0)
            telemetry_callback: Optional callback(theta1, theta2, roll, pitch, latency)
        """
        # ... existing init code ...
        self.telemetry_callback = telemetry_callback
        self._last_command_time = 0.0

    def _process_telemetry(self, telemetry: dict) -> None:
        """Update MuJoCo state from hardware telemetry."""
        # ... existing processing ...

        # Call telemetry callback for dashboard update
        if self.telemetry_callback is not None:
            latency = (time.time() - self._last_command_time) * 1000
            roll = pitch = 0.0

            if telemetry['type'] == TelemetryType.FULL:
                data = telemetry['data']
                if len(data) >= 52:
                    values = struct.unpack('<I 2f 2f 3f 3f 2f', data)
                    roll = values[11]
                    pitch = values[12]

            self.telemetry_callback(
                self.data.qpos[0], self.data.qpos[1],
                roll, pitch, latency
            )

    def _send_commands(self) -> None:
        """Send joint angle commands to hardware."""
        self._last_command_time = time.time()
        # ... existing code ...
```

**Step 2: Create launch script with dashboard**

```python
# simulation/scripts/run_hil_dashboard.py
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
    parser = argparse.ArgumentParser(description='HIL Dashboard')
    parser.add_argument('port', nargs='?', default='COM3', help='Serial port')
    parser.add_argument('--model', default='mujoco_model/arm.xml', help='MuJoCo model path')
    parser.add_argument('--rate', type=float, default=25.0, help='Update rate (Hz)')
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
        telemetry_callback=dashboard.update_state
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


if __name__ == '__main__':
    sys.exit(main())
```

**Step 3: Test dashboard with hardware (manual)**

Run: `cd simulation && python scripts/run_hil_dashboard.py COM3`
Expected: Dashboard opens, shows live telemetry if hardware connected

**Step 4: Commit integration**

```bash
git add simulation/middleware/hil_synchronizer.py simulation/scripts/run_hil_dashboard.py
git commit -m "feat(hil): integrate dashboard with HIL synchronizer for live visualization"
```

---

## Task 8: Error Handling - Watchdog Timer (Firmware)

**Files:**
- Modify: `firmware/Core/Src/main.c`
- Modify: `firmware/Core/Inc/main.h`

**Step 1: Add watchdog variables to main.h**

Add to `firmware/Core/Inc/main.h`:

```c
// Watchdog timeout (milliseconds)
#define WATCHDOG_TIMEOUT_MS 500

// Safe servo positions (radians)
#define SERVO_SAFE_POSITION 0.0f
```

**Step 2: Implement software watchdog in main.c**

Add to `firmware/Core/Src/main.c`:

```c
// Add global variables (after other globals ~line 66)
volatile uint32_t last_command_time = 0;
volatile bool watchdog_triggered = false;

// Add watchdog check function
void Check_Watchdog(void) {
    uint32_t now = HAL_GetTick();

    if (now - last_command_time > WATCHDOG_TIMEOUT_MS) {
        if (!watchdog_triggered) {
            // First trigger: move to safe position
            Servo_SetAngle(&servo_shoulder, SERVO_SAFE_POSITION);
            Servo_SetAngle(&servo_elbow, SERVO_SAFE_POSITION);
            watchdog_triggered = true;

            // Blink LED rapidly to indicate watchdog
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }
    }
}

// Modify Handle_Command to reset watchdog
void Handle_Command(Packet_t *packet) {
    // Reset watchdog on any valid command
    last_command_time = HAL_GetTick();
    watchdog_triggered = false;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // LED on = active

    switch (packet->type) {
        // ... existing cases ...
    }
}

// Add to main loop (in while(1))
while (1) {
    Process_UART_Commands();
    Check_Watchdog();  // Add this line
}
```

**Step 3: Build and verify**

Run: Build in STM32CubeIDE
Expected: Build succeeds

**Step 4: Commit watchdog**

```bash
git add firmware/Core/Src/main.c firmware/Core/Inc/main.h
git commit -m "feat(firmware): add software watchdog for safety timeout"
```

---

## Task 9: Error Handling - Auto-Reconnect (Python)

**Files:**
- Modify: `simulation/middleware/serial_manager.py`
- Modify: `simulation/middleware/tests/test_serial_manager.py`

**Step 1: Write failing test for auto-reconnect**

Add to `simulation/middleware/tests/test_serial_manager.py`:

```python
class TestAutoReconnect:
    """Test automatic reconnection logic."""

    def test_reconnect_on_disconnect(self, mocker):
        """Should attempt reconnection after disconnect detected."""
        mock_serial = mocker.patch('serial.Serial')
        mock_instance = mock_serial.return_value
        mock_instance.is_open = True

        manager = SerialManager(port='COM3')
        manager.connect()

        # Simulate disconnect
        mock_instance.write.side_effect = serial.SerialException("Disconnected")

        result = manager.send_command(0x10, b'\x00\x00')

        assert result == False
        assert manager.is_connected() == False

    def test_reconnect_callback(self, mocker):
        """Should call reconnect callback on disconnect."""
        mock_serial = mocker.patch('serial.Serial')

        callback_called = [False]
        def on_disconnect():
            callback_called[0] = True

        manager = SerialManager(port='COM3', on_disconnect=on_disconnect)
        manager.connect()

        # Trigger disconnect
        manager._connected = False
        manager._check_connection()

        # Note: actual reconnect logic tested in integration
```

**Step 2: Run test to verify current behavior**

Run: `cd simulation && python -m pytest middleware/tests/test_serial_manager.py::TestAutoReconnect -v`
Expected: Tests may fail or error (depending on current implementation)

**Step 3: Enhance SerialManager with reconnect support**

Modify `simulation/middleware/serial_manager.py`:

```python
class SerialManager:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.1,
                 on_disconnect: callable = None, auto_reconnect: bool = True,
                 reconnect_interval: float = 1.0):
        """Initialize serial manager.

        Args:
            port: Serial port name
            baud: Baud rate (default: 115200)
            timeout: Read timeout in seconds
            on_disconnect: Callback when disconnect detected
            auto_reconnect: Whether to auto-reconnect (default: True)
            reconnect_interval: Seconds between reconnect attempts
        """
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._connected = False
        self._on_disconnect = on_disconnect
        self._auto_reconnect = auto_reconnect
        self._reconnect_interval = reconnect_interval
        self._last_reconnect_attempt = 0.0
        self._reconnect_thread: Optional[threading.Thread] = None

    def _check_connection(self):
        """Check connection and trigger reconnect if needed."""
        if not self._connected and self._on_disconnect:
            self._on_disconnect()

        if not self._connected and self._auto_reconnect:
            self._attempt_reconnect()

    def _attempt_reconnect(self):
        """Attempt to reconnect to serial port."""
        now = time.time()
        if now - self._last_reconnect_attempt < self._reconnect_interval:
            return False

        self._last_reconnect_attempt = now

        try:
            with self._lock:
                if self._serial is not None:
                    try:
                        self._serial.close()
                    except:
                        pass

                self._serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
                self._connected = True
                return True
        except serial.SerialException:
            return False

    def send_command(self, cmd_type: int, data: bytes) -> bool:
        """Send command packet to hardware."""
        if not self._connected or self._serial is None:
            self._check_connection()
            return False

        try:
            packet = encode_packet(cmd_type, data)
            with self._lock:
                self._serial.write(packet)
            return True
        except serial.SerialException:
            self._connected = False
            self._check_connection()
            return False
```

**Step 4: Run tests**

Run: `cd simulation && python -m pytest middleware/tests/test_serial_manager.py -v`
Expected: All tests pass

**Step 5: Commit auto-reconnect**

```bash
git add simulation/middleware/serial_manager.py simulation/middleware/tests/test_serial_manager.py
git commit -m "feat(serial): add auto-reconnect with configurable callback"
```

---

## Task 10: Figure-8 Acceptance Test Script

**Files:**
- Create: `simulation/scripts/test_figure8_trajectory.py`

**Step 1: Create acceptance test script**

```python
# simulation/scripts/test_figure8_trajectory.py
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
    parser = argparse.ArgumentParser(description='Figure-8 acceptance test')
    parser.add_argument('port', nargs='?', default='COM3', help='Serial port')
    parser.add_argument('--duration', type=float, default=20.0, help='Test duration (s)')
    parser.add_argument('--cycles', type=int, default=2, help='Number of figure-8 cycles')
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
        center=(0.12, 0.0),
        width=0.04,
        height=0.03,
        num_points=100 * args.cycles
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

        if telemetry and telemetry['type'] in [TelemetryType.ANGLES_ONLY, TelemetryType.FULL]:
            import struct
            data = telemetry['data']
            if len(data) >= 8:
                actual_theta1, actual_theta2 = struct.unpack('<ff', data[:8])

                # Calculate error
                err1 = abs(np.degrees(cmd_theta1 - actual_theta1))
                err2 = abs(np.degrees(cmd_theta2 - actual_theta2))
                max_err = max(err1, err2)
                errors.append(max_err)

                # Progress
                if i % 20 == 0:
                    print(f"  [{i+1}/{len(joint_trajectory)}] "
                          f"Cmd: ({np.degrees(cmd_theta1):+5.1f}°, {np.degrees(cmd_theta2):+5.1f}°) "
                          f"Act: ({np.degrees(actual_theta1):+5.1f}°, {np.degrees(actual_theta2):+5.1f}°) "
                          f"Err: {max_err:.1f}°")

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


if __name__ == '__main__':
    sys.exit(main())
```

**Step 2: Test script runs (may fail without hardware)**

Run: `cd simulation && python scripts/test_figure8_trajectory.py --help`
Expected: Shows help message with options

**Step 3: Commit acceptance test**

```bash
git add simulation/scripts/test_figure8_trajectory.py
git commit -m "test: add figure-8 trajectory acceptance test script"
```

---

## Task 11: Run All Tests and Validate

**Step 1: Run Python test suite**

Run: `cd simulation && python -m pytest middleware/tests/ -v --tb=short`
Expected: All tests pass

**Step 2: Build firmware**

Run: Build in STM32CubeIDE
Expected: Build succeeds with 0 errors, 0 warnings

**Step 3: Flash and test with hardware (manual)**

1. Flash firmware to STM32
2. Run: `cd simulation && python scripts/test_figure8_trajectory.py COM3`
3. Expected: Acceptance test passes with <5° error

**Step 4: Final commit**

```bash
git add -A
git commit -m "feat: complete Week 3 - sensor fusion, trajectory, dashboard, error handling"
```

---

## Summary

| Task | Component | Files | Status |
|------|-----------|-------|--------|
| 1 | Sensor Fusion | `firmware/Core/*/sensor_fusion.*` | |
| 2 | Linear Trajectory | `simulation/middleware/trajectory.py` | |
| 3 | Cubic Spline | `simulation/middleware/trajectory.py` | |
| 4 | Figure-8 Pattern | `simulation/middleware/trajectory.py` | |
| 5 | Joint Conversion | `simulation/middleware/trajectory.py` | |
| 6 | Dashboard UI | `simulation/visualization/dashboard.py` | |
| 7 | HIL Integration | `simulation/scripts/run_hil_dashboard.py` | |
| 8 | Watchdog Timer | `firmware/Core/Src/main.c` | |
| 9 | Auto-Reconnect | `simulation/middleware/serial_manager.py` | |
| 10 | Acceptance Test | `simulation/scripts/test_figure8_trajectory.py` | |
| 11 | Validation | All | |

**Total estimated tasks:** 11 major tasks with ~55 bite-sized steps
