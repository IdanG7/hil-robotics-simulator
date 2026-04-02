"""Serial communication manager for hardware interface."""

import serial
import threading
import time
from typing import Optional, Dict, Callable
from middleware.protocol import encode_packet, decode_packet, CommandType, TelemetryType


class SerialManager:
    """Manages serial communication with STM32 hardware.

    Features:
    - Automatic reconnection on disconnect
    - Thread-safe send/receive
    - Timeout detection
    """

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        timeout: float = 0.1,
        on_disconnect: Optional[Callable] = None,
        auto_reconnect: bool = True,
        reconnect_interval: float = 1.0,
    ):
        """Initialize serial manager.

        Args:
            port: Serial port name (e.g., 'COM3', '/dev/ttyUSB0')
            baud: Baud rate (default: 115200)
            timeout: Read timeout in seconds (default: 0.1)
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

    def is_connected(self) -> bool:
        """Check if serial port is connected."""
        return self._connected

    def connect(self) -> bool:
        """Connect to serial port.

        Returns:
            True if connected successfully, False otherwise
        """
        try:
            with self._lock:
                self._serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
                self._connected = True
            return True
        except serial.SerialException as e:
            self._connected = False
            return False

    def disconnect(self) -> None:
        """Disconnect from serial port."""
        with self._lock:
            if self._serial is not None:
                self._serial.close()
                self._serial = None
            self._connected = False

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
        """Send command packet to hardware.

        Args:
            cmd_type: Command type (from CommandType enum)
            data: Command data bytes

        Returns:
            True if sent successfully, False otherwise
        """
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

    def receive_telemetry(self, timeout: float = 0.05) -> Optional[Dict]:
        """Receive and decode telemetry packet.

        Args:
            timeout: Timeout in seconds

        Returns:
            Decoded telemetry dict or None if no packet
        """
        if not self._connected or self._serial is None:
            return None

        try:
            # Read until we get a valid packet
            start_time = time.time()
            buffer = bytearray()

            while time.time() - start_time < timeout:
                # if self._serial.in_waiting > 0:
                byte = self._serial.read(1)
                if len(byte) == 0:
                    break

                # Look for header
                if len(buffer) == 0 and byte[0] != 0xAA:
                    continue

                buffer.extend(byte)

                # Check if we have enough for length field
                if len(buffer) >= 3:
                    expected_len = 4 + buffer[2]
                    if len(buffer) >= expected_len:
                        # Try to decode
                        packet = decode_packet(bytes(buffer))
                        if packet is not None:
                            return packet
                        buffer.clear()

            return None
        except serial.SerialException:
            self._connected = False
            return None
