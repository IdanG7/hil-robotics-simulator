"""Debug serial: Send raw bytes and see what comes back."""

import sys
import serial
import time
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM3'

    print(f"Debug Serial: {port}")
    print("=" * 50)

    ser = serial.Serial(port, 115200, timeout=0.5)
    time.sleep(0.1)  # Let connection stabilize

    # Clear any pending data
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Send a valid SET_JOINT_ANGLES packet manually
    # [0xAA][0x10][0x08][8 bytes of zeros][CRC]
    # CRC of [0x10, 0x08, 0,0,0,0,0,0,0,0]
    from middleware.protocol import encode_packet, CommandType, crc8

    packet = encode_packet(CommandType.SET_JOINT_ANGLES, bytes(8))
    print(f"Sending packet: {packet.hex()}")
    print(f"Packet bytes: {list(packet)}")

    ser.write(packet)
    ser.flush()

    print("Waiting for response...")
    time.sleep(0.2)

    # Read any response
    response = ser.read(ser.in_waiting or 100)
    if response:
        print(f"Received {len(response)} bytes: {response.hex()}")
        print(f"Response bytes: {list(response)}")
    else:
        print("No response received")

    # Try reading for a bit longer
    print("\nListening for 2 more seconds...")
    start = time.time()
    while time.time() - start < 2:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            print(f"Got: {data.hex()} = {list(data)}")
        time.sleep(0.1)

    ser.close()
    print("Done")

if __name__ == '__main__':
    main()
