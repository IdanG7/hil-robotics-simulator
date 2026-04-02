"""Hardware-in-the-Loop synchronizer for MuJoCo simulation."""

import mujoco
import time
import struct
import numpy as np
from typing import Optional, Callable
from middleware.serial_manager import SerialManager
from middleware.protocol import CommandType, TelemetryType, encode_set_joint_angles


class HILSynchronizer:
    """Synchronizes MuJoCo simulation with physical hardware.

    Commands hardware at specified rate (default 25Hz), receives telemetry,
    and updates simulation state accordingly.
    """

    def __init__(
        self,
        port: str,
        model_path: str,
        update_rate: float = 25.0,
        telemetry_callback: Optional[Callable] = None,
    ):
        """Initialize HIL synchronizer.

        Args:
            port: Serial port for hardware communication
            model_path: Path to MuJoCo MJCF model file
            update_rate: Update frequency in Hz (default: 25.0)
            telemetry_callback: Optional callback(theta1, theta2, roll, pitch, latency)
        """
        self.port = port
        self.update_rate = update_rate
        self.period = 1.0 / update_rate

        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Initialize serial manager
        self.serial = SerialManager(port=port)

        self.telemetry_callback = telemetry_callback
        self._last_command_time = 0.0
        self._running = False

    def start(self) -> None:
        """Start HIL loop (blocking).

        Connects to hardware, enters control loop, and runs until stopped.
        """
        # Connect to hardware
        if not self.serial.connect():
            raise RuntimeError(f"Failed to connect to {self.port}")

        print(f"Connected to {self.port}")
        print(f"HIL Loop: {self.update_rate}Hz ({self.period*1000:.1f}ms period)")
        print("Press Ctrl+C to stop")

        self._running = True

        try:
            while self._running:
                loop_start = time.time()

                # Command hardware
                self._send_commands()

                # Receive telemetry
                telemetry = self.serial.receive_telemetry(timeout=0.02)
                if telemetry is not None:
                    self._process_telemetry(telemetry)

                # Step physics
                mujoco.mj_step(self.model, self.data)

                # Maintain update rate
                elapsed = time.time() - loop_start
                if elapsed < self.period:
                    time.sleep(self.period - elapsed)
                else:
                    print(
                        f"Warning: Loop overrun: {elapsed*1000:.1f}ms > {self.period*1000:.1f}ms"
                    )

        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            self.stop()

    def stop(self) -> None:
        """Stop HIL loop and disconnect."""
        self._running = False
        self.serial.disconnect()
        print("Disconnected")

    def _send_commands(self) -> None:
        """Send joint angle commands to hardware."""
        self._last_command_time = time.time()

        # Get commanded angles from MuJoCo actuators
        shoulder = self.data.ctrl[0]  # Actuator 0: shoulder
        elbow = self.data.ctrl[1]  # Actuator 1: elbow

        # Encode and send
        packet = encode_set_joint_angles(shoulder, elbow)
        self.serial._serial.write(packet)

    def _process_telemetry(self, telemetry: dict) -> None:
        """Update MuJoCo state from hardware telemetry."""
        roll = pitch = 0.0

        if telemetry["type"] == TelemetryType.ANGLES_ONLY:
            # Update joint positions in MuJoCo
            angles = telemetry["data"]
            if len(angles) >= 8:
                shoulder, elbow = struct.unpack("<ff", angles[:8])
                self.data.qpos[0] = shoulder
                self.data.qpos[1] = elbow

        elif telemetry["type"] == TelemetryType.FULL:
            # Process full telemetry (IMU data, velocities)
            data = telemetry["data"]
            if len(data) >= 52:
                values = struct.unpack("<I 2f 2f 3f 3f 2f", data)
                # Update joint positions
                self.data.qpos[0] = values[1]  # shoulder angle
                self.data.qpos[1] = values[2]  # elbow angle
                # Velocities could be used for better physics simulation
                # IMU data available in values[5:11]
                roll = values[11]  # IMU roll
                pitch = values[12]  # IMU pitch

        # Call telemetry callback for dashboard update
        if self.telemetry_callback is not None:
            latency = (time.time() - self._last_command_time) * 1000
            self.telemetry_callback(
                self.data.qpos[0], self.data.qpos[1], roll, pitch, latency
            )
