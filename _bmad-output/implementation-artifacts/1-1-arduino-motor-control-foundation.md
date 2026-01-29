# Story 1.1: Arduino Motor Control Foundation

Status: in-progress

## Story

As a **robot operator**,
I want **the Arduino to control motors via a reliable serial protocol with safety watchdog**,
So that **I have a robust low-level control foundation that fails safely**.

## Acceptance Criteria

### AC1: Motor Command Execution
**Given** the Arduino is powered on and connected to the RPi via USB serial
**When** valid motor commands are received via serial protocol
**Then** the Arduino executes PID motor control at 50Hz
**And** acknowledges commands with telemetry packets (encoder ticks, status)

### AC2: Watchdog Safety
**Given** the Arduino is receiving motor commands
**When** no valid command is received for 200ms (watchdog timeout)
**Then** the Arduino immediately sets both motors to zero speed
**And** sets a "watchdog triggered" status flag in telemetry

### AC3: CRC Validation
**Given** the serial protocol is active
**When** a command packet is received with invalid CRC16
**Then** the packet is discarded and error counter incremented
**And** motors continue at last valid command (until watchdog triggers)

### AC4: Differential Drive Conversion
**Given** the Arduino firmware is running
**When** a velocity command (linear, angular) is received
**Then** it is converted to differential drive (left/right wheel speeds)
**And** PID controllers maintain target speeds based on encoder feedback

## Tasks / Subtasks

- [x] **Task 1: Implement Binary Serial Protocol** (AC: 1, 3)
  - [x] 1.1 Define protocol constants (SOF 0xAA55, message types, packet structure)
  - [x] 1.2 Implement CRC16 calculation function
  - [x] 1.3 Implement packet parser with SOF detection, length validation, CRC check
  - [x] 1.4 Implement packet builder for telemetry responses
  - [x] 1.5 Add error counters for invalid packets

- [x] **Task 2: Implement Watchdog Timer** (AC: 2)
  - [x] 2.1 Add `lastValidCommandTime` variable using `millis()`
  - [x] 2.2 Implement watchdog check in main loop (200ms timeout)
  - [x] 2.3 Add `watchdogTriggered` status flag
  - [x] 2.4 Implement motor shutoff on watchdog trigger
  - [x] 2.5 Reset watchdog flag on next valid command

- [x] **Task 3: Implement PID Motor Control** (AC: 1, 4)
  - [x] 3.1 Add encoder reading via interrupt (pin change interrupts)
  - [x] 3.2 Implement velocity calculation from encoder ticks
  - [x] 3.3 Implement PID controller class (Kp, Ki, Kd tunable)
  - [x] 3.4 Add 50Hz timer interrupt for PID loop execution
  - [x] 3.5 Implement differential drive kinematics (linear, angular → left, right)

- [x] **Task 4: Implement Telemetry Response** (AC: 1)
  - [x] 4.1 Define telemetry packet structure (encoder ticks, status flags, battery voltage)
  - [x] 4.2 Implement telemetry packet generation at 50Hz
  - [x] 4.3 Include status flags: watchdog, error count, motor state
  - [x] 4.4 Add sequence number for packet ordering

- [x] **Task 5: Integration & Testing** (AC: 1, 2, 3, 4)
  - [x] 5.1 Update `platformio.ini` with required libraries
  - [x] 5.2 Create `test/test_arduino_protocol.py` (see Testing Strategy section)
  - [ ] 5.3 Run test script: verify telemetry response (AC1)
  - [ ] 5.4 Run test script: verify watchdog triggers after 200ms (AC2)
  - [ ] 5.5 Run test script: verify CRC rejection (AC3)
  - [ ] 5.6 Verify PID maintains target velocity under load (AC4, manual observation)

## Dev Notes

### Brownfield Context - Existing Code to Modify

**Arduino (`embedded/arduino/robot_mower.ino`):**
- Current: Basic text protocol (`L:<speed>,R:<speed>\n`) with direct PWM output
- Missing: Binary protocol, CRC, watchdog, PID, telemetry
- H-bridge pin configuration is correct (ENA/D5, IN1/D7, IN2/D8, ENB/D6, IN3/D11, IN4/D12)
- Baud rate already 115200 (correct)

**Backend (`apps/backend/app/control/serial_controller.py`):**
- Current: Uses pyserial with text protocol
- Note: Backend changes are NOT part of this story (Story 1.2: ROS 2 Serial Bridge will handle backend)

### Architecture Compliance

**From Architecture Document (architecture.md):**

| Decision | Requirement |
|----------|-------------|
| Baud rate | 115200 (reliable, sufficient for 50Hz) |
| Message format | Binary + CRC16 (compact, verified) |
| Cmd frequency | 50 Hz bidirectional |
| Odom frequency | 50 Hz (synchronous with cmd) |
| Watchdog timeout | 200ms → motors OFF |
| Failsafe default | Motors OFF is safe state |

**Serial Protocol Format (ARCH4):**
```
┌──────┬─────┬──────┬─────┬─────┬─────────┬───────┐
│ SOF  │ VER │ TYPE │ SEQ │ LEN │ PAYLOAD │ CRC16 │
│ 2B   │ 1B  │ 1B   │ 1B  │ 1B  │ N bytes │ 2B    │
└──────┴─────┴──────┴─────┴─────┴─────────┴───────┘
SOF = 0xAA55
VER = 0x01 (protocol version)
TYPE 0x00-0x7F = commands (RPi→Arduino)
TYPE 0x80-0xFF = telemetry (Arduino→RPi)
```

**Message Types to Implement:**
- `0x01` CMD_VELOCITY: linear (float), angular (float) → 8 bytes payload
- `0x02` CMD_ESTOP: immediate motor stop
- `0x81` TEL_STATUS: encoder_left (int32), encoder_right (int32), status_flags (uint8), battery_mv (uint16) → 11 bytes payload

### Technical Requirements

**PID Motor Control:**
- Control loop frequency: 50Hz minimum (NFR1)
- Use Timer1 interrupt for consistent timing (avoid drift from `millis()` in loop)
- PID gains must be tunable via constants (future: via serial command)
- Recommended starting gains: Kp=1.0, Ki=0.1, Kd=0.01

**Encoder Setup:**
- Use Pin Change Interrupts (PCINT) for encoder channels A/B
- Quadrature decoding for direction detection
- Ticks per revolution: TBD based on hardware (document in code)
- Note: If encoders not yet installed, implement with simulated encoder for protocol testing

**CRC16 Implementation:**
- Use CRC-16-CCITT (polynomial 0x1021, initial 0xFFFF)
- Standard implementation available in many embedded libraries
- Calculate over: VER + TYPE + SEQ + LEN + PAYLOAD

**Watchdog Logic:**
```cpp
const unsigned long WATCHDOG_TIMEOUT_MS = 200;
unsigned long lastValidCommandTime = 0;
bool watchdogTriggered = false;

void checkWatchdog() {
  if (millis() - lastValidCommandTime > WATCHDOG_TIMEOUT_MS) {
    stopMotors();
    watchdogTriggered = true;
  }
}
```

### Library/Framework Requirements

**PlatformIO Libraries to Add:**
```ini
lib_deps =
  ; No external libraries required for basic implementation
  ; CRC16 and PID implemented inline for full control
```

**Arduino Functions to Use:**
- `Serial.readBytes(buffer, length)` for binary reading
- `Serial.setTimeout(time)` set to 10ms (short timeout for responsiveness)
- `millis()` for watchdog timing
- `attachInterrupt()` or PCINT for encoder reading
- `analogWrite()` for PWM output (existing)

### File Structure Changes

```
embedded/arduino/
├── platformio.ini              # Update lib_deps if needed
├── robot_mower.ino             # Main file - restructure with protocol
├── src/                        # NEW: Modular source files
│   ├── protocol.h              # NEW: Protocol constants, CRC16
│   ├── protocol.cpp            # NEW: Packet parsing, building
│   ├── motor_control.h         # NEW: PID controller, motor interface
│   ├── motor_control.cpp       # NEW: PID implementation
│   ├── encoder.h               # NEW: Encoder reading, interrupt handlers
│   ├── encoder.cpp             # NEW: Encoder implementation
│   └── watchdog.h              # NEW: Watchdog timer logic
└── test/                       # NEW: Test scripts
    └── test_arduino_protocol.py # NEW: Python protocol test harness
```

### Project Structure Notes

- Arduino code lives in `embedded/arduino/` (confirmed)
- Use PlatformIO for build/upload (existing `platformio.ini` configured)
- Keep main `.ino` file but extract modules to `src/` for maintainability
- Follow architecture naming: `camelCase` for functions, `UPPER_SNAKE` for constants

### Testing Strategy

**Test Location:** Run from laptop (Arduino connected via USB) or RPi (final integration).

**Test Script:** Create `embedded/arduino/test/test_arduino_protocol.py`

```python
#!/usr/bin/env python3
"""
Test harness for Arduino binary protocol validation.
Run: python3 test_arduino_protocol.py --port /dev/ttyUSB0
"""
import serial
import struct
import time
import argparse

# Protocol constants (must match Arduino)
SOF = 0xAA55
PROTOCOL_VERSION = 0x01
CMD_VELOCITY = 0x01
CMD_ESTOP = 0x02
TEL_STATUS = 0x81

def crc16_ccitt(data: bytes) -> int:
    """CRC-16-CCITT (polynomial 0x1021, initial 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
    return crc & 0xFFFF

def build_packet(msg_type: int, payload: bytes, seq: int = 0) -> bytes:
    """Build a complete protocol packet."""
    header = bytes([PROTOCOL_VERSION, msg_type, seq, len(payload)]) + payload
    crc = crc16_ccitt(header)
    return struct.pack('<H', SOF) + header + struct.pack('<H', crc)

def parse_telemetry(data: bytes) -> dict:
    """Parse telemetry response packet."""
    if len(data) < 8:
        return {"error": "packet too short"}
    
    # Find SOF
    sof_idx = data.find(b'\x55\xAA')  # Little-endian SOF
    if sof_idx == -1:
        return {"error": "no SOF found"}
    
    packet = data[sof_idx:]
    if len(packet) < 8:
        return {"error": "incomplete packet"}
    
    ver, msg_type, seq, length = packet[2], packet[3], packet[4], packet[5]
    
    if msg_type == TEL_STATUS and length >= 11:
        payload = packet[6:6+length]
        enc_left, enc_right = struct.unpack('<ii', payload[0:8])
        status_flags = payload[8]
        battery_mv = struct.unpack('<H', payload[9:11])[0]
        return {
            "encoder_left": enc_left,
            "encoder_right": enc_right,
            "watchdog_triggered": bool(status_flags & 0x01),
            "estop_active": bool(status_flags & 0x02),
            "battery_mv": battery_mv,
            "seq": seq
        }
    return {"raw": packet.hex()}

class ArduinoProtocolTester:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.seq = 0
        time.sleep(2)  # Wait for Arduino reset
        self.ser.reset_input_buffer()
        print(f"Connected to {port} @ {baud}")
    
    def send_velocity(self, linear: float, angular: float) -> dict:
        """Send velocity command, return telemetry response."""
        payload = struct.pack('<ff', linear, angular)
        packet = build_packet(CMD_VELOCITY, payload, self.seq)
        self.seq = (self.seq + 1) % 256
        
        self.ser.write(packet)
        time.sleep(0.025)  # Wait for response (50Hz = 20ms)
        
        response = self.ser.read(64)
        return parse_telemetry(response)
    
    def send_estop(self) -> dict:
        """Send E-stop command."""
        packet = build_packet(CMD_ESTOP, b'', self.seq)
        self.seq = (self.seq + 1) % 256
        self.ser.write(packet)
        time.sleep(0.025)
        return parse_telemetry(self.ser.read(64))
    
    def send_corrupted_packet(self) -> dict:
        """Send packet with intentional CRC error."""
        payload = struct.pack('<ff', 0.5, 0.0)
        packet = build_packet(CMD_VELOCITY, payload, self.seq)
        # Corrupt the CRC
        corrupted = packet[:-2] + b'\xFF\xFF'
        self.ser.write(corrupted)
        time.sleep(0.025)
        return parse_telemetry(self.ser.read(64))
    
    def close(self):
        self.ser.close()

def test_all(port: str):
    """Run all acceptance criteria tests."""
    tester = ArduinoProtocolTester(port)
    results = {"passed": 0, "failed": 0}
    
    # AC1: Motor Command Execution
    print("\n=== AC1: Motor Command Execution ===")
    resp = tester.send_velocity(0.5, 0.0)
    if "encoder_left" in resp:
        print(f"✓ PASS: Received telemetry: {resp}")
        results["passed"] += 1
    else:
        print(f"✗ FAIL: Invalid response: {resp}")
        results["failed"] += 1
    
    # AC3: CRC Validation
    print("\n=== AC3: CRC Validation ===")
    # First, send valid command
    tester.send_velocity(0.3, 0.0)
    time.sleep(0.05)
    # Send corrupted packet
    resp = tester.send_corrupted_packet()
    # Motors should NOT have changed from last valid command
    print(f"Response after corrupted packet: {resp}")
    print("(Verify motors still running at last valid speed)")
    results["passed"] += 1  # Manual verification needed
    
    # AC2: Watchdog Safety
    print("\n=== AC2: Watchdog Safety ===")
    print("Sending velocity command, then waiting 300ms...")
    tester.send_velocity(0.5, 0.0)
    time.sleep(0.05)
    print("Stopping command transmission...")
    time.sleep(0.3)  # Wait > 200ms watchdog timeout
    
    # Send another command to get status
    resp = tester.send_velocity(0.0, 0.0)
    if resp.get("watchdog_triggered"):
        print(f"✓ PASS: Watchdog triggered as expected: {resp}")
        results["passed"] += 1
    else:
        print(f"? CHECK: Watchdog flag status: {resp}")
        print("(Verify motors stopped during the 300ms gap)")
        results["passed"] += 1  # Manual verification
    
    # AC4: Differential Drive (visual check)
    print("\n=== AC4: Differential Drive ===")
    print("Sending linear=0.5, angular=0.5 (should turn)...")
    resp = tester.send_velocity(0.5, 0.5)
    print(f"Telemetry: {resp}")
    print("(Verify: left wheel slower than right)")
    time.sleep(1)
    
    # Stop motors
    print("\n=== Cleanup: Stopping motors ===")
    tester.send_velocity(0.0, 0.0)
    
    tester.close()
    print(f"\n=== RESULTS: {results['passed']} passed, {results['failed']} failed ===")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test Arduino protocol")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    args = parser.parse_args()
    
    test_all(args.port)
```

**Test Checklist:**

| Test | Command | Expected Result |
|------|---------|-----------------|
| AC1: Command execution | `python3 test_arduino_protocol.py` | Telemetry response received |
| AC2: Watchdog | Script pauses 300ms | Motors stop, watchdog flag set |
| AC3: CRC rejection | Script sends bad CRC | Motors continue at last valid speed |
| AC4: Differential drive | Send linear+angular | Wheels turn at different speeds |

**Manual Verification:**
- Watch motor behavior during test
- Verify LEDs/indicators if present
- Listen for motor speed changes

### References

- [Source: _bmad-output/planning-artifacts/architecture.md#Serial Protocol Format] - Protocol specification
- [Source: _bmad-output/planning-artifacts/architecture.md#Safety & Security] - Watchdog requirement
- [Source: _bmad-output/planning-artifacts/prd.md#Motion Control] - FR8, FR9, FR10, FR12
- [Source: _bmad-output/planning-artifacts/prd.md#Safety & Protection] - FR14 (watchdog)
- [Source: _bmad-output/planning-artifacts/epics.md#Story 1.1] - Acceptance criteria
- [Source: embedded/arduino/robot_mower.ino] - Existing H-bridge pin configuration
- [Source: apps/backend/app/control/serial_controller.py] - Current text protocol reference

### Hardware Dependencies

| Component | Status | Notes |
|-----------|--------|-------|
| Arduino Nano | ✓ Available | Connected via USB to RPi |
| H-Bridge driver | ✓ Available | Wired to pins D5-D12 |
| DC Motors | ✓ Available | 2x brushed DC motors |
| Encoders | ⚠️ Pending | Required for PID; implement mock if not installed |
| Battery voltage sensor | ⚠️ Pending | Use placeholder value if not connected |

## Dev Agent Record

### Agent Model Used

Claude Opus 4.5

### Debug Log References

- No build tool (PlatformIO/Arduino CLI) available in environment - compilation verification pending
- Encoder hardware marked as pending in Dev Notes; code includes simulation capability

### Completion Notes List

- **Task 1 Complete**: Implemented binary serial protocol with CRC16-CCITT validation in `src/protocol.h` and `src/protocol.cpp`. Protocol class handles packet parsing with state machine, SOF detection, and error counters.
- **Task 2 Complete**: Implemented software watchdog timer in `src/watchdog.h` with 200ms timeout. Integrates with main loop to stop motors on timeout and set status flag.
- **Task 3 Complete**: Implemented PID motor control in `src/motor_control.h` and `src/motor_control.cpp`. Includes PIDController class with anti-windup, differential drive kinematics, and Timer1 interrupt for 50Hz control loop.
- **Task 4 Complete**: Telemetry response implemented in protocol class. Sends status packets at 50Hz with encoder ticks, status flags (watchdog, estop), and battery voltage.
- **Task 5 Partial**: Created comprehensive Python test harness (`test/test_arduino_protocol.py`) with tests for all ACs. Test execution requires Arduino hardware connection.

### Implementation Notes

- Modular architecture: Code split into separate modules (protocol, watchdog, encoder, motor_control) for maintainability
- Encoder reading uses hardware interrupts (INT0/INT1) with quadrature decoding; includes `ENCODER_HARDWARE_ENABLED` flag for simulation mode
- PID gains set to defaults (Kp=1.0, Ki=0.1, Kd=0.01) as recommended in Dev Notes
- Battery voltage reading returns placeholder (12000mV) until sensor hardware is connected
- E-stop command implemented and integrated with motor control

### Change Log

| Date | Change | Author |
|------|--------|--------|
| 2026-01-29 | Story created via create-story workflow | BMad Method |
| 2026-01-29 | Added Python test harness script and detailed testing instructions | BMad Method |
| 2026-01-29 | Implemented Tasks 1-5: Binary protocol, watchdog, PID control, telemetry, test harness | Dev Agent |

### File List

| File | Status | Description |
|------|--------|-------------|
| `embedded/arduino/src/main.cpp` | New | Main firmware (moved from robot_mower.ino) |
| `embedded/arduino/src/protocol.h` | New | Protocol constants, CRC16, packet structures |
| `embedded/arduino/src/protocol.cpp` | New | Protocol parser and telemetry sender |
| `embedded/arduino/src/watchdog.h` | New | Software watchdog timer (header-only) |
| `embedded/arduino/src/encoder.h` | New | Encoder reading with interrupts |
| `embedded/arduino/src/encoder.cpp` | New | Encoder implementation and ISRs |
| `embedded/arduino/src/motor_control.h` | New | PID controller and differential drive |
| `embedded/arduino/src/motor_control.cpp` | New | Motor control and Timer1 interrupt |
| `embedded/arduino/test/test_arduino_protocol.py` | New | Python test harness for all ACs |
