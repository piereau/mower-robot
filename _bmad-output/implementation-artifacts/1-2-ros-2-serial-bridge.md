# Story 1.2: ROS 2 Serial Bridge

Status: review

## Story

As a **robot operator**,
I want **a ROS 2 node that bridges serial communication to standard ROS topics**,
So that **I can use standard ROS 2 tools and integrate with Nav2 later**.

## Acceptance Criteria

### AC1: Serial Connection & Status Publishing
**Given** the serial bridge node is launched
**When** the node starts
**Then** it connects to Arduino via /dev/ttyUSB0 at 115200 baud
**And** publishes connection status to /mower/serial_status

### AC2: Velocity Command Subscription
**Given** a geometry_msgs/Twist message is published to /cmd_vel
**When** the serial bridge receives it
**Then** it encodes the command using the binary protocol (SOF, TYPE, SEQ, PAYLOAD, CRC16)
**And** sends it to Arduino at 50Hz rate

### AC3: Telemetry Publishing
**Given** the Arduino sends telemetry packets
**When** the serial bridge receives valid packets
**Then** it publishes nav_msgs/Odometry to /odom (wheel odometry)
**And** publishes mower_msgs/MowerStatus to /mower/status

### AC4: Reconnection Handling
**Given** serial communication is interrupted
**When** no valid packets received for 1 second
**Then** the node publishes "disconnected" status
**And** logs error with reconnection attempts

## Tasks / Subtasks

- [x] **Task 1: Create mower_msgs Package** (AC: 3)
  - [x] 1.1 Create `apps/ros2/src/mower_msgs/` package structure
  - [x] 1.2 Define `MowerStatus.msg` (encoder_left, encoder_right, watchdog_triggered, estop_active, battery_mv)
  - [x] 1.3 Define `SerialStatus.msg` (connected, last_packet_time, error_count)
  - [x] 1.4 Add CMakeLists.txt and package.xml for message generation
  - [x] 1.5 Build and verify messages generate correctly

- [x] **Task 2: Create mower_hardware Package** (AC: 1, 2, 3, 4)
  - [x] 2.1 Create `apps/ros2/src/mower_hardware/` package structure
  - [x] 2.2 Add `mower_hardware/serial_bridge.py` main node file
  - [x] 2.3 Add `mower_hardware/protocol.py` (protocol constants, CRC16, packet builder/parser)
  - [x] 2.4 Add package.xml with dependencies (rclpy, std_msgs, geometry_msgs, nav_msgs, mower_msgs, pyserial)
  - [x] 2.5 Add setup.py with console_scripts entry point

- [x] **Task 3: Implement Protocol Layer** (AC: 2, 3)
  - [x] 3.1 Port CRC16-CCITT from Arduino implementation (polynomial 0x1021, initial 0xFFFF)
  - [x] 3.2 Implement `build_velocity_packet(linear, angular, seq)` function
  - [x] 3.3 Implement `build_estop_packet(seq)` function
  - [x] 3.4 Implement `parse_telemetry_packet(data)` function with SOF detection and CRC validation

- [x] **Task 4: Implement Serial Bridge Node** (AC: 1, 2, 3, 4)
  - [x] 4.1 Implement SerialBridge class extending rclpy.node.Node
  - [x] 4.2 Add /cmd_vel subscriber (geometry_msgs/Twist) with rate limiting to 50Hz
  - [x] 4.3 Add /odom publisher (nav_msgs/Odometry) for wheel odometry
  - [x] 4.4 Add /mower/status publisher (MowerStatus)
  - [x] 4.5 Add /mower/serial_status publisher (SerialStatus)
  - [x] 4.6 Implement threaded serial reader for non-blocking I/O
  - [x] 4.7 Implement 50Hz timer for command transmission
  - [x] 4.8 Implement connection monitoring with 1-second timeout
  - [x] 4.9 Implement reconnection logic with exponential backoff

- [x] **Task 5: Add Launch Configuration** (AC: 1)
  - [x] 5.1 Create `apps/ros2/src/mower_bringup/` package (if not exists)
  - [x] 5.2 Add `launch/serial_bridge.launch.py` with serial port parameter
  - [x] 5.3 Add `config/serial_bridge_params.yaml` for configurable parameters

- [ ] **Task 6: Integration Testing** (AC: 1, 2, 3, 4)
  - [ ] 6.1 Test serial connection with Arduino (verify /mower/serial_status shows connected)
  - [ ] 6.2 Test velocity command transmission (`ros2 topic pub /cmd_vel geometry_msgs/Twist`)
  - [ ] 6.3 Verify telemetry publishing (`ros2 topic echo /mower/status`)
  - [ ] 6.4 Verify odometry publishing (`ros2 topic echo /odom`)
  - [ ] 6.5 Test reconnection (unplug/replug Arduino, verify recovery)

## Dev Notes

### Previous Story Intelligence (Story 1.1)

**Key Learnings from Story 1.1 Implementation:**

| Aspect | What Was Done | Impact on This Story |
|--------|---------------|---------------------|
| Protocol format | Binary: SOF(0xAA55) + VER + TYPE + SEQ + LEN + PAYLOAD + CRC16 | MUST match exactly in Python |
| CRC implementation | CRC-16-CCITT (poly 0x1021, init 0xFFFF) | Port identical algorithm |
| Message types | CMD_VELOCITY=0x01, CMD_ESTOP=0x02, TEL_STATUS=0x81 | Use same constants |
| Velocity payload | 8 bytes: linear(float) + angular(float), little-endian | Use `struct.pack('<ff', ...)` |
| Telemetry payload | 11 bytes: enc_left(i32) + enc_right(i32) + flags(u8) + battery(u16) | Parse with `struct.unpack` |
| Status flags | Bit 0=watchdog, Bit 1=estop, Bit 4=CRC error seen | Extract and publish |
| Baud rate | 115200 | Match in pyserial config |
| Timing | 50Hz bidirectional | Timer callback at 20ms |

**Arduino Protocol Header Reference** (from `embedded/arduino/src/protocol.h`):
```cpp
#define PROTOCOL_SOF          0xAA55
#define PROTOCOL_VERSION      0x01
#define MSG_CMD_VELOCITY      0x01
#define MSG_CMD_ESTOP         0x02
#define MSG_TEL_STATUS        0x81
#define PAYLOAD_SIZE_VELOCITY 8
#define PAYLOAD_SIZE_STATUS   11
#define STATUS_FLAG_WATCHDOG_TRIGGERED  0x01
#define STATUS_FLAG_ESTOP_ACTIVE        0x02
#define STATUS_FLAG_CRC_ERROR_SEEN      0x10
```

### Architecture Compliance

**From Architecture Document:**

| Decision | Requirement | Implementation |
|----------|-------------|----------------|
| Bridge node | Custom Python (pyserial + rclpy) | `serial_bridge.py` with rclpy |
| Baud rate | 115200 | pyserial configuration |
| Message format | Binary custom + CRC16 | protocol.py module |
| Cmd frequency | 50 Hz | rclpy Timer at 20ms |
| Odom frequency | 50 Hz | Publish on each valid telemetry |
| Naming | `/mower/snake_case` for topics | /mower/status, /mower/serial_status |

**ROS 2 Workspace Structure** (ARCH8-ARCH14):
```
ros2_ws/
└── src/
    ├── mower_bringup/        # Launch files, configs
    │   ├── launch/
    │   │   └── serial_bridge.launch.py
    │   └── config/
    │       └── serial_bridge_params.yaml
    ├── mower_hardware/       # Serial bridge, motor interface
    │   ├── mower_hardware/
    │   │   ├── __init__.py
    │   │   ├── serial_bridge.py
    │   │   └── protocol.py
    │   ├── setup.py
    │   └── package.xml
    └── mower_msgs/           # Custom messages
        ├── msg/
        │   ├── MowerStatus.msg
        │   └── SerialStatus.msg
        ├── CMakeLists.txt
        └── package.xml
```

### Technical Requirements

**ROS 2 Best Practices (2024/2025):**
- Use threading/asyncio for serial I/O to avoid blocking ROS 2 executor
- Implement robust framing with SOF detection, checksums
- Use exponential backoff for reconnection (1s, 2s, 4s, max 30s)
- Handle serial port disconnection gracefully
- Install pyserial via: `sudo apt install python3-serial` or `pip install pyserial`

**Serial Bridge Node Architecture:**
```
┌─────────────────────────────────────────────────────────┐
│                   SerialBridgeNode                       │
├─────────────────────────────────────────────────────────┤
│  Subscribers:                                            │
│    /cmd_vel (geometry_msgs/Twist)                       │
│                                                          │
│  Publishers:                                             │
│    /odom (nav_msgs/Odometry)                            │
│    /mower/status (mower_msgs/MowerStatus)               │
│    /mower/serial_status (mower_msgs/SerialStatus)       │
│                                                          │
│  Timers:                                                 │
│    50Hz command transmission timer                      │
│    1Hz status publish timer                             │
│                                                          │
│  Threads:                                                │
│    Serial reader thread (non-blocking)                  │
└─────────────────────────────────────────────────────────┘
```

**Odometry Calculation:**
```python
# Wheel parameters (must match physical robot)
WHEEL_SEPARATION = 0.3  # meters (track width)
TICKS_PER_METER = 1000  # encoder ticks per meter traveled (calibrate)

# Calculate velocities from encoder delta
def calculate_odometry(delta_left_ticks, delta_right_ticks, dt):
    left_dist = delta_left_ticks / TICKS_PER_METER
    right_dist = delta_right_ticks / TICKS_PER_METER
    linear = (left_dist + right_dist) / 2.0 / dt
    angular = (right_dist - left_dist) / WHEEL_SEPARATION / dt
    return linear, angular
```

> **⚠️ IMPORTANT:** Wheel parameters (WHEEL_SEPARATION, TICKS_PER_METER) are placeholders. 
> Calibrate with actual hardware before autonomous navigation.

### Library/Framework Requirements

**Python Dependencies:**
```python
# package.xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>mower_msgs</depend>
<exec_depend>python3-serial</exec_depend>  # pyserial
```

**setup.py:**
```python
entry_points={
    'console_scripts': [
        'serial_bridge = mower_hardware.serial_bridge:main',
    ],
},
```

### Message Definitions

**mower_msgs/msg/MowerStatus.msg:**
```
# Motor and sensor status from Arduino
int32 encoder_left          # Cumulative left encoder ticks
int32 encoder_right         # Cumulative right encoder ticks
bool watchdog_triggered     # True if watchdog fired (comms timeout)
bool estop_active           # True if E-stop is active
bool crc_error_seen         # True if Arduino saw CRC errors
uint16 battery_mv           # Battery voltage in millivolts
uint8 sequence              # Packet sequence number
```

**mower_msgs/msg/SerialStatus.msg:**
```
# Serial connection status
bool connected              # True if serial port is open and responding
float64 last_packet_time    # ROS time of last valid packet
uint32 packets_received     # Total valid packets received
uint32 packets_sent         # Total packets sent
uint32 crc_errors           # CRC validation failures
```

### Project Structure Notes

- ROS 2 workspace lives in `ros2_ws/` at project root (to be created)
- Follow ROS 2 Humble conventions for package structure
- Use `colcon build` for workspace compilation
- Serial port typically `/dev/ttyUSB0` on RPi (may vary: `/dev/ttyACM0`)

### Testing Strategy

**Test Commands:**

```bash
# Terminal 1: Launch serial bridge
cd ~/ros2_ws && source install/setup.bash
ros2 run mower_hardware serial_bridge --ros-args -p serial_port:=/dev/ttyUSB0

# Terminal 2: Check connection status
ros2 topic echo /mower/serial_status

# Terminal 3: Send velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" -r 10

# Terminal 4: Monitor telemetry
ros2 topic echo /mower/status

# Terminal 5: Monitor odometry
ros2 topic echo /odom
```

**Acceptance Criteria Verification:**

| AC | Test | Expected Result |
|----|------|-----------------|
| AC1 | Launch node, check /mower/serial_status | connected=True |
| AC2 | Pub to /cmd_vel, verify motor movement | Arduino receives velocity, motors move |
| AC3 | Echo /odom and /mower/status | Valid encoder/status data published |
| AC4 | Unplug Arduino, wait 2s, replug | Disconnected→reconnected automatically |

### References

- [Source: _bmad-output/planning-artifacts/architecture.md#Communication ROS ↔ Arduino] - Protocol decisions
- [Source: _bmad-output/planning-artifacts/architecture.md#Workspace ROS 2 Structure] - Package organization
- [Source: _bmad-output/planning-artifacts/epics.md#Story 1.2] - Acceptance criteria
- [Source: embedded/arduino/src/protocol.h] - Protocol constants (MUST MATCH)
- [Source: _bmad-output/implementation-artifacts/1-1-arduino-motor-control-foundation.md] - Previous story learnings
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/) - rclpy best practices

### Hardware Dependencies

| Component | Status | Notes |
|-----------|--------|-------|
| Arduino Nano | ✓ Available | Running Story 1.1 firmware |
| USB Serial | ✓ Available | /dev/ttyUSB0 or /dev/ttyACM0 |
| RPi 4B | ✓ Available | ROS 2 Humble installed |

## Dev Agent Record

### Agent Model Used

Gemini 2.5 (Antigravity)

### Debug Log References

- Protocol tests: All 5 core tests passed (CRC16, velocity packet, e-stop packet, telemetry parsing, CRC validation)

### Completion Notes List

- **Task 1-5 Complete**: All ROS 2 packages created and structured properly
- **Protocol Layer**: CRC-16-CCITT implementation matches Arduino exactly (verified with test vector "123456789" = 0x29B1)
- **Serial Bridge Node**: Full implementation with threaded I/O, reconnection logic, and TF broadcasting
- **Integration Testing (Task 6)**: Requires hardware - left unchecked for on-robot testing
- **Note**: Wheel parameters (WHEEL_SEPARATION, TICKS_PER_METER) are placeholders - calibrate with actual hardware

### Change Log

| Date | Change | Author |
|------|--------|--------|
| 2026-02-01 | Story created via create-story workflow | SM Agent (Bob) |
| 2026-02-01 | Tasks 1-5 implemented: mower_msgs, mower_hardware, mower_bringup packages | Dev Agent (Amelia) |

### File List

| File | Status | Description |
|------|--------|-------------|
| `apps/ros2/src/mower_msgs/package.xml` | Created | Message package manifest |
| `apps/ros2/src/mower_msgs/CMakeLists.txt` | Created | Message generation build config |
| `apps/ros2/src/mower_msgs/msg/MowerStatus.msg` | Created | Custom message for motor status |
| `apps/ros2/src/mower_msgs/msg/SerialStatus.msg` | Created | Custom message for serial status |
| `apps/ros2/src/mower_hardware/package.xml` | Created | Hardware package manifest |
| `apps/ros2/src/mower_hardware/setup.py` | Created | Python package setup |
| `apps/ros2/src/mower_hardware/setup.cfg` | Created | Setup configuration |
| `apps/ros2/src/mower_hardware/mower_hardware/__init__.py` | Created | Package init |
| `apps/ros2/src/mower_hardware/mower_hardware/protocol.py` | Created | Protocol implementation |
| `apps/ros2/src/mower_hardware/mower_hardware/serial_bridge.py` | Created | Main ROS 2 node |
| `apps/ros2/src/mower_hardware/test/test_protocol.py` | Created | Protocol unit tests |
| `apps/ros2/src/mower_hardware/resource/mower_hardware` | Created | Ament resource marker |
| `apps/ros2/src/mower_bringup/package.xml` | Created | Bringup package manifest |
| `apps/ros2/src/mower_bringup/CMakeLists.txt` | Created | Bringup build config |
| `apps/ros2/src/mower_bringup/launch/serial_bridge.launch.py` | Created | Launch file |
| `apps/ros2/src/mower_bringup/config/serial_bridge_params.yaml` | Created | Parameter config |

