#!/usr/bin/env python3
"""
Interactive motor test - test one thing at a time with clear observation.

Run: python3 interactive_test.py --port /dev/ttyUSB0
"""

import serial
import struct
import time
import argparse
import sys

# Protocol constants
SOF = 0xAA55
PROTOCOL_VERSION = 0x01
MSG_CMD_VELOCITY = 0x01
MSG_CMD_ESTOP = 0x02
MSG_TEL_STATUS = 0x81


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
    return crc & 0xFFFF


def build_velocity_command(linear: float, angular: float, seq: int = 0) -> bytes:
    payload = struct.pack("<ff", linear, angular)
    header = bytes([PROTOCOL_VERSION, MSG_CMD_VELOCITY, seq, len(payload)]) + payload
    crc = crc16_ccitt(header)
    return struct.pack("<H", SOF) + header + struct.pack("<H", crc)


def build_corrupted_packet(linear: float, angular: float, seq: int = 0) -> bytes:
    """Build packet with intentionally wrong CRC."""
    packet = build_velocity_command(linear, angular, seq)
    pkt = bytearray(packet)
    if len(pkt) > 6:
        pkt[6] ^= 0xFF  # Flip one payload byte to force CRC mismatch
    return bytes(pkt)


class MotorTester:
    def __init__(self, port: str):
        print(f"Connecting to {port}...")
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.seq = 0
        time.sleep(2)  # Wait for Arduino reset
        self.ser.reset_input_buffer()
        print("Connected!\n")

    def send_velocity(self, linear: float, angular: float):
        """Send velocity and return response."""
        packet = build_velocity_command(linear, angular, self.seq)
        self.seq = (self.seq + 1) % 256
        self.ser.write(packet)
        time.sleep(0.03)
        return self.ser.read(64)

    def send_corrupted(self, linear: float, angular: float):
        """Send corrupted packet."""
        packet = build_corrupted_packet(linear, angular, self.seq)
        self.ser.write(packet)
        time.sleep(0.03)
        return self.ser.read(64)

    def keep_alive(self, duration: float, linear: float = 0.0, angular: float = 0.0):
        """Send commands continuously to keep motors running."""
        end_time = time.time() + duration
        while time.time() < end_time:
            self.send_velocity(linear, angular)
            time.sleep(0.02)  # 50Hz

    def stop(self):
        """Stop motors."""
        for _ in range(5):
            self.send_velocity(0.0, 0.0)
            time.sleep(0.02)

    def close(self):
        self.stop()
        self.ser.close()


def wait_for_enter(msg="Press ENTER to continue..."):
    input(msg)


def main():
    parser = argparse.ArgumentParser(description="Interactive motor test")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    args = parser.parse_args()

    tester = MotorTester(args.port)

    try:
        while True:
            print("\n" + "=" * 50)
            print("INTERACTIVE MOTOR TEST")
            print("=" * 50)
            print("""
Choose a test:

  1. Forward motion (both wheels same direction)
  2. Backward motion
  3. Turn LEFT (right wheel faster)
  4. Turn RIGHT (left wheel faster)
  5. Spin in place (wheels opposite directions)
  6. WATCHDOG TEST - motors should STOP after 300ms
  7. CRC TEST - send bad packet (motors should NOT change)
  8. STOP motors
  9. Custom velocity
  0. Exit
""")
            choice = input("Enter choice (0-9): ").strip()

            if choice == "1":
                print("\n>>> FORWARD: linear=0.5, angular=0.0")
                print("    Both wheels should spin FORWARD at ~50% speed")
                wait_for_enter("Press ENTER to start (runs for 3 seconds)...")
                tester.keep_alive(3.0, linear=0.5, angular=0.0)
                tester.stop()
                print("    Stopped.")

            elif choice == "2":
                print("\n>>> BACKWARD: linear=-0.5, angular=0.0")
                print("    Both wheels should spin BACKWARD at ~50% speed")
                wait_for_enter("Press ENTER to start (runs for 3 seconds)...")
                tester.keep_alive(3.0, linear=-0.5, angular=0.0)
                tester.stop()
                print("    Stopped.")

            elif choice == "3":
                print("\n>>> TURN LEFT: linear=0.3, angular=0.5")
                print("    Right wheel should be FASTER than left")
                wait_for_enter("Press ENTER to start (runs for 3 seconds)...")
                tester.keep_alive(3.0, linear=0.3, angular=0.5)
                tester.stop()
                print("    Stopped.")

            elif choice == "4":
                print("\n>>> TURN RIGHT: linear=0.3, angular=-0.5")
                print("    Left wheel should be FASTER than right")
                wait_for_enter("Press ENTER to start (runs for 3 seconds)...")
                tester.keep_alive(3.0, linear=0.3, angular=-0.5)
                tester.stop()
                print("    Stopped.")

            elif choice == "5":
                print("\n>>> SPIN IN PLACE: linear=0.0, angular=1.0")
                print("    Wheels should spin OPPOSITE directions")
                wait_for_enter("Press ENTER to start (runs for 3 seconds)...")
                tester.keep_alive(3.0, linear=0.0, angular=1.0)
                tester.stop()
                print("    Stopped.")

            elif choice == "6":
                print("\n>>> WATCHDOG TEST")
                print("    Step 1: Motors will run for 2 seconds")
                print("    Step 2: Commands STOP for 500ms")
                print("    Step 3: Motors should STOP by themselves (watchdog)")
                wait_for_enter("Press ENTER to start...")
                
                print("    Running motors...")
                tester.keep_alive(2.0, linear=0.5, angular=0.0)
                
                print("    Stopping commands (waiting 500ms for watchdog)...")
                print("    >>> WATCH THE MOTORS - they should STOP! <<<")
                time.sleep(0.5)
                
                wait_for_enter("Did the motors stop? Press ENTER...")
                tester.stop()

            elif choice == "7":
                print("\n>>> CRC VALIDATION TEST")
                print("    Step 1: Motors run at 50% speed")
                print("    Step 2: Send CORRUPTED packet")
                print("    Step 3: Motors should CONTINUE (bad packet ignored)")
                wait_for_enter("Press ENTER to start...")
                
                print("    Running motors at 50%...")
                tester.keep_alive(2.0, linear=0.5, angular=0.0)
                
                print("    Sending CORRUPTED packet...")
                tester.send_corrupted(0.0, 0.0)  # Try to stop with bad CRC
                
                print("    Motors should still be running!")
                print("    (keeping alive for 2 more seconds)")
                tester.keep_alive(2.0, linear=0.5, angular=0.0)
                
                tester.stop()
                print("    Stopped.")

            elif choice == "8":
                print("\n>>> STOPPING MOTORS")
                tester.stop()
                print("    Motors stopped.")

            elif choice == "9":
                print("\n>>> CUSTOM VELOCITY")
                try:
                    linear = float(input("    Enter linear velocity (-1.0 to 1.0): "))
                    angular = float(input("    Enter angular velocity (-1.0 to 1.0): "))
                    duration = float(input("    Enter duration (seconds): "))
                    
                    print(f"    Running: linear={linear}, angular={angular} for {duration}s")
                    wait_for_enter("Press ENTER to start...")
                    tester.keep_alive(duration, linear=linear, angular=angular)
                    tester.stop()
                    print("    Stopped.")
                except ValueError:
                    print("    Invalid input!")

            elif choice == "0":
                print("\nExiting...")
                break

            else:
                print("Invalid choice!")

    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    finally:
        tester.close()
        print("Done.")


if __name__ == "__main__":
    main()
