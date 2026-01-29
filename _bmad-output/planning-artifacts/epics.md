---
stepsCompleted:
  - step-01-validate-prerequisites
  - step-02-design-epics
  - step-03-create-stories
  - step-04-final-validation
status: complete
inputDocuments:
  - _bmad-output/planning-artifacts/prd.md
  - _bmad-output/planning-artifacts/architecture.md
---

# mower-roboto - Epic Breakdown

## Overview

This document provides the complete epic and story breakdown for mower-roboto, decomposing the requirements from the PRD and Architecture into implementable stories.

## Requirements Inventory

### Functional Requirements

**Navigation & Localization**
- FR1: Robot can build 2D occupancy map of operating area using SLAM
- FR2: Robot can localize within mapped area with <30cm accuracy
- FR3: Robot can navigate to specified waypoints avoiding obstacles
- FR4: Robot can follow vine row centerline maintaining ±20cm lateral deviation
- FR5: Robot can execute 180° headland turns at row ends
- FR6: Robot can resume navigation from current position after pause/stop
- FR7: Robot can fall back to SLAM-only localization when GPS is unavailable

**Motion Control**
- FR8: Operator can control robot motion via joystick commands (teleoperation)
- FR9: Robot can execute velocity commands (linear + angular) at 50Hz
- FR10: Robot can stop within 0.5m from full speed when commanded
- FR11: Robot can traverse slopes up to 25° without slipping backward
- FR12: Robot can rotate in place for tight maneuvering

**Safety & Protection**
- FR13: Operator can trigger immediate motor stop via dashboard E-stop button
- FR14: Robot automatically stops if RPi-Arduino communication fails for 200ms
- FR15: Robot automatically stops if tilt exceeds 25° (pitch or roll)
- FR16: Robot respects geofence boundaries defined in map
- FR17: Robot detects obstacles within 1m and stops or replans path
- FR18: Robot reports safety events (E-stop, watchdog, tilt) to supervision dashboard

**Supervision & Monitoring**
- FR19: User can view real-time robot state (idle, navigating, mowing, error) on dashboard
- FR20: User can view battery level with low-battery warning
- FR21: User can view robot position on map overlay
- FR22: User can view LiDAR scan visualization
- FR23: User can view mission progress (% complete, current row)
- FR24: User receives push notification on mission complete or alert
- FR25: Dashboard shows connection status with automatic reconnection

**Mission Management**
- FR26: User can define mowing zones on map
- FR27: User can create missions (sequence of rows/waypoints)
- FR28: User can start, pause, resume, and abort missions
- FR29: User can save and recall mission patterns
- FR30: Robot can execute multi-row missions autonomously
- FR31: User can send "skip to next row" command during obstacle situations

**Diagnostics & Maintenance**
- FR32: User can view system diagnostics (sensor status, firmware versions)
- FR33: User can run self-test sequence to verify all subsystems
- FR34: User can update Arduino firmware via RPi
- FR35: System logs operational data for post-session review
- FR36: User can export logs for debugging

**Configuration**
- FR37: User can configure safety thresholds (tilt limit, speed limits)
- FR38: User can configure notification preferences
- FR39: User can define keepout zones in map
- FR40: User can calibrate sensors (IMU, encoders)

### NonFunctional Requirements

**Performance**
- NFR1: Motor commands execute at 50Hz minimum (control loop frequency)
- NFR2: Dashboard updates within 500ms of state change (telemetry latency)
- NFR3: Motor shutoff within 200ms of E-stop command (E-stop response)
- NFR4: New path computed within 2 seconds (navigation replanning)
- NFR5: Robot operational within 60 seconds of power-on (boot time)

**Reliability**
- NFR6: 2+ hours continuous operation without requiring restart
- NFR7: Auto-reconnect within 10 seconds after WiFi drop (communication recovery)
- NFR8: Mission state survives brief (<5s) communication interruption (mission persistence)
- NFR9: >50 operating hours between manual interventions (MTBF)

**Scalability**
- NFR10: Support maps up to 10 ha (academic scope)
- NFR11: Support missions with up to 50 waypoints
- NFR12: Single supervisor (multi-user future phase)

**Maintainability**
- NFR13: ROS 2 packages independently deployable (code modularity)
- NFR14: All parameters in YAML files, no hardcoded values (configuration management)
- NFR15: Structured logs with severity levels for debugging (logging)
- NFR16: README for each major component (documentation)

**Security**
- NFR17: Dashboard accessible only on local network (no cloud exposure for MVP)
- NFR18: WebSocket commands require session token (command authentication)
- NFR19: Firmware updates only via authenticated SSH session (firmware integrity)

**Usability**
- NFR20: Dashboard functional on smartphone screen (mobile-first UI)
- NFR21: High-contrast UI elements for sunlight visibility (outdoor readability)
- NFR22: User-friendly error descriptions with suggested actions (error messages)
- NFR23: New user completes first teleoperation session in <15 minutes (learning curve)

### Additional Requirements

**From Architecture - Starter Template:**
- ARCH1: Use Nav2 + slam_toolbox standard configuration, adapted for tracked robot
- ARCH2: Base reference: Turtlebot3 navigation stack (differential drive model)
- ARCH3: Adaptations required: rectangular footprint, slip-aware odometry model, agricultural terrain costmap parameters

**From Architecture - Communication Protocol:**
- ARCH4: Serial protocol: binary format with CRC16 (SOF 0xAA55, VER, TYPE, SEQ, LEN, PAYLOAD, CRC16)
- ARCH5: Baud rate: 115200 (reliable, sufficient for 50Hz)
- ARCH6: Command frequency: 50Hz bidirectional
- ARCH7: Message types: 0x00-0x7F for commands (RPi→Arduino), 0x80-0xFF for telemetry (Arduino→RPi)

**From Architecture - ROS 2 Workspace Structure:**
- ARCH8: Create mower_bringup package (launch files, configs)
- ARCH9: Create mower_description package (URDF, TF)
- ARCH10: Create mower_navigation package (Nav2 params, behaviors)
- ARCH11: Create mower_perception package (LiDAR, camera nodes)
- ARCH12: Create mower_localization package (EKF config, SLAM params)
- ARCH13: Create mower_hardware package (serial bridge, motor interface)
- ARCH14: Create mower_msgs package (custom messages: MowerStatus.msg, SafetyStatus.msg)

**From Architecture - Safety Implementation:**
- ARCH15: E-stop: Software only for MVP (no hardware button for prototype)
- ARCH16: Watchdog: Arduino 200ms timeout - stops motors if RPi silent
- ARCH17: Geofencing: Nav2 costmap keepout zones
- ARCH18: Tilt detection: IMU thresholds (pitch/roll > 25°)
- ARCH19: Failsafe default: Motors OFF is safe state

**From Architecture - Deployment:**
- ARCH20: Service manager: systemd for auto-start
- ARCH21: ROS 2 launch: Via systemd service at boot
- ARCH22: Updates: Git pull + systemctl restart
- ARCH23: Arduino flash: USB + platformio CLI from RPi

**From Architecture - Implementation Dependencies:**
- ARCH24: Serial bridge depends on Arduino protocol implementation
- ARCH25: EKF depends on encoders + IMU data
- ARCH26: Nav2 depends on SLAM + EKF
- ARCH27: UI telemetry depends on all subsystems

**From PRD - Project Context:**
- CTX1: Brownfield project - existing WebSocket telemetry dashboard (FastAPI + React)
- CTX2: Existing code to integrate: FastAPI backend, React + Vite + Tailwind frontend

**From PRD - Domain Constraints:**
- DOM1: Handle slopes up to 25°, soft soil, uneven surfaces
- DOM2: IP54 minimum protection (dust/splash resistant)
- DOM3: Operating temperature: 0°C to 40°C
- DOM4: Vegetation height: up to 30cm
- DOM5: Row width: 1.5m to 2.5m
- DOM6: 2+ hour continuous operation between charges

### FR Coverage Map

| FR | Epic | Description |
|----|------|-------------|
| FR1 | Epic 2 | SLAM mapping |
| FR2 | Epic 2 | Localization <30cm accuracy |
| FR3 | Epic 2 | Waypoint navigation with obstacle avoidance |
| FR4 | Epic 3 | Row centerline following |
| FR5 | Epic 3 | Headland turns |
| FR6 | Epic 3 | Resume navigation after pause |
| FR7 | Epic 2 | SLAM fallback when GPS unavailable |
| FR8 | Epic 1 | Joystick teleoperation |
| FR9 | Epic 1 | 50Hz velocity command execution |
| FR10 | Epic 1 | Stop within 0.5m |
| FR11 | Epic 3 | Slope traversal up to 25° |
| FR12 | Epic 1 | Rotate in place |
| FR13 | Epic 1 | Dashboard E-stop |
| FR14 | Epic 1 | Watchdog (200ms communication timeout) |
| FR15 | Epic 1 | Tilt detection (25° threshold) |
| FR16 | Epic 3 | Geofence boundary enforcement |
| FR17 | Epic 2 | Obstacle detection within 1m |
| FR18 | Epic 5 | Safety event reporting |
| FR19 | Epic 1 | Real-time robot state display |
| FR20 | Epic 1 | Battery level with low-battery warning |
| FR21 | Epic 2 | Robot position on map overlay |
| FR22 | Epic 2 | LiDAR scan visualization |
| FR23 | Epic 4 | Mission progress display |
| FR24 | Epic 4 | Push notifications |
| FR25 | Epic 1 | Connection status with auto-reconnect |
| FR26 | Epic 4 | Define mowing zones |
| FR27 | Epic 4 | Create missions |
| FR28 | Epic 4 | Start/pause/resume/abort missions |
| FR29 | Epic 4 | Save and recall mission patterns |
| FR30 | Epic 4 | Multi-row autonomous execution |
| FR31 | Epic 4 | Skip to next row command |
| FR32 | Epic 5 | System diagnostics |
| FR33 | Epic 5 | Self-test sequence |
| FR34 | Epic 5 | Arduino firmware update |
| FR35 | Epic 5 | Operational logging |
| FR36 | Epic 5 | Log export |
| FR37 | Epic 5 | Safety threshold configuration |
| FR38 | Epic 5 | Notification preferences |
| FR39 | Epic 5 | Keepout zone definition |
| FR40 | Epic 5 | Sensor calibration |

## Epic List

### Epic 1: Safe Manual Control
Pierrot can safely drive the robot remotely via the dashboard, see real-time telemetry, and trust that safety systems will protect the robot.

**FRs covered:** FR8, FR9, FR10, FR12, FR13, FR14, FR15, FR19, FR20, FR25

### Epic 2: Autonomous Mapping & Navigation
Pierrot can have the robot build a map of the vineyard and navigate autonomously to specified waypoints while avoiding obstacles.

**FRs covered:** FR1, FR2, FR3, FR7, FR17, FR21, FR22

### Epic 3: Vineyard Row Operations
Pierrot can have the robot follow vine row centerlines, execute headland turns, and operate on sloped terrain—the core vineyard mowing behavior.

**FRs covered:** FR4, FR5, FR6, FR11, FR16

### Epic 4: Mission Autonomy
Pierrot can create multi-row mowing missions, start them, monitor progress, and receive notifications—enabling hands-off autonomous operation.

**FRs covered:** FR23, FR24, FR26, FR27, FR28, FR29, FR30, FR31

### Epic 5: System Administration
Pierrot (in admin mode) can diagnose issues, update firmware, configure safety parameters, and maintain the robot between seasons.

**FRs covered:** FR18, FR32, FR33, FR34, FR35, FR36, FR37, FR38, FR39, FR40

---

## Epic 1: Safe Manual Control

Pierrot can safely drive the robot remotely via the dashboard, see real-time telemetry, and trust that safety systems will protect the robot.

### Story 1.1: Arduino Motor Control Foundation

As a **robot operator**,
I want **the Arduino to control motors via a reliable serial protocol with safety watchdog**,
So that **I have a robust low-level control foundation that fails safely**.

**Acceptance Criteria:**

**Given** the Arduino is powered on and connected to the RPi via USB serial
**When** valid motor commands are received via serial protocol
**Then** the Arduino executes PID motor control at 50Hz
**And** acknowledges commands with telemetry packets (encoder ticks, status)

**Given** the Arduino is receiving motor commands
**When** no valid command is received for 200ms (watchdog timeout)
**Then** the Arduino immediately sets both motors to zero speed
**And** sets a "watchdog triggered" status flag in telemetry

**Given** the serial protocol is active
**When** a command packet is received with invalid CRC16
**Then** the packet is discarded and error counter incremented
**And** motors continue at last valid command (until watchdog triggers)

**Given** the Arduino firmware is running
**When** a velocity command (linear, angular) is received
**Then** it is converted to differential drive (left/right wheel speeds)
**And** PID controllers maintain target speeds based on encoder feedback

### Story 1.2: ROS 2 Serial Bridge

As a **robot operator**,
I want **a ROS 2 node that bridges serial communication to standard ROS topics**,
So that **I can use standard ROS 2 tools and integrate with Nav2 later**.

**Acceptance Criteria:**

**Given** the serial bridge node is launched
**When** the node starts
**Then** it connects to Arduino via /dev/ttyUSB0 at 115200 baud
**And** publishes connection status to /mower/serial_status

**Given** a geometry_msgs/Twist message is published to /cmd_vel
**When** the serial bridge receives it
**Then** it encodes the command using the binary protocol (SOF, TYPE, SEQ, PAYLOAD, CRC16)
**And** sends it to Arduino at 50Hz rate

**Given** the Arduino sends telemetry packets
**When** the serial bridge receives valid packets
**Then** it publishes nav_msgs/Odometry to /odom (wheel odometry)
**And** publishes mower_msgs/MowerStatus to /mower/status

**Given** serial communication is interrupted
**When** no valid packets received for 1 second
**Then** the node publishes "disconnected" status
**And** logs error with reconnection attempts

### Story 1.3: Teleoperation via Dashboard

As a **robot operator**,
I want **to control robot motion using a virtual joystick on the dashboard**,
So that **I can manually drive the robot for positioning and testing**.

**Acceptance Criteria:**

**Given** the dashboard is connected via WebSocket
**When** I interact with the joystick component
**Then** joystick position is translated to linear velocity (-1.0 to 1.0 m/s) and angular velocity (-1.0 to 1.0 rad/s)
**And** commands are sent via WebSocket at 10Hz minimum

**Given** the FastAPI backend receives joystick commands
**When** valid velocity commands arrive
**Then** they are forwarded to ROS 2 /cmd_vel topic
**And** command acknowledgment is sent back to dashboard

**Given** I am using the joystick
**When** I release the joystick (return to center)
**Then** zero velocity command is sent immediately
**And** robot stops moving within 0.5m (FR10)

**Given** the robot is in teleoperation mode
**When** I push the joystick fully forward
**Then** the robot moves forward at maximum configured speed
**And** speed is displayed on dashboard

### Story 1.4: Real-time Telemetry Display

As a **robot operator**,
I want **to see real-time robot state, battery level, and connection status on the dashboard**,
So that **I can monitor the robot's health while operating**.

**Acceptance Criteria:**

**Given** the dashboard is connected
**When** telemetry is received from the robot
**Then** robot state is displayed (idle, moving, error)
**And** telemetry updates within 500ms of state change (NFR2)

**Given** the robot is operational
**When** battery voltage is read by Arduino
**Then** battery percentage is calculated and displayed on dashboard
**And** low-battery warning appears when below 20%

**Given** the dashboard WebSocket connection
**When** connection is lost
**Then** "Disconnected" indicator is shown prominently
**And** automatic reconnection attempts occur with exponential backoff
**And** "Reconnecting..." status is displayed

**Given** the dashboard is displaying telemetry
**When** connection is restored after interruption
**Then** "Connected" indicator is shown
**And** telemetry display resumes immediately

### Story 1.5: Emergency Stop System

As a **robot operator**,
I want **to immediately stop the robot via E-stop button, with automatic tilt protection**,
So that **the robot is safe in emergency situations and on slopes**.

**Acceptance Criteria:**

**Given** the dashboard is displaying robot controls
**When** I press the E-stop button
**Then** an E-stop command is sent immediately via WebSocket
**And** motors stop within 200ms (NFR3)
**And** robot enters "E-STOP" state displayed on dashboard

**Given** the robot is in E-stop state
**When** I press "Release E-stop" button
**Then** E-stop is released
**And** robot returns to "idle" state
**And** teleoperation is re-enabled

**Given** the robot has IMU connected (or simulated for initial testing)
**When** pitch or roll exceeds 25° threshold
**Then** motors automatically stop (tilt protection - FR15)
**And** "TILT DETECTED" alert is shown on dashboard
**And** robot enters protected state until manually cleared

**Given** the watchdog is active
**When** RPi-Arduino communication fails for 200ms
**Then** Arduino stops motors automatically (FR14)
**And** when communication resumes, status shows "watchdog was triggered"

---

## Epic 2: Autonomous Mapping & Navigation

Pierrot can have the robot build a map of the vineyard and navigate autonomously to specified waypoints while avoiding obstacles.

### Story 2.1: LiDAR Integration

As a **robot operator**,
I want **the LD19 LiDAR to publish scan data to ROS 2**,
So that **the robot can perceive its environment for mapping and obstacle detection**.

**Acceptance Criteria:**

**Given** the LD19 LiDAR is connected to the RPi
**When** the LiDAR driver node is launched
**Then** it publishes sensor_msgs/LaserScan to /scan topic
**And** scan data is published at 8-15Hz (native LD19 rate)

**Given** the LiDAR is publishing scan data
**When** I run `ros2 topic echo /scan`
**Then** I see valid range data with 12m maximum range
**And** angular resolution matches LD19 specifications

**Given** the LiDAR driver is running
**When** the LiDAR is disconnected or fails
**Then** the driver logs an error and attempts reconnection
**And** publishes diagnostic status indicating sensor failure

### Story 2.2: Robot Description & TF

As a **robot operator**,
I want **a proper URDF model with correct TF transforms**,
So that **sensor data is correctly positioned for navigation algorithms**.

**Acceptance Criteria:**

**Given** the mower_description package is built
**When** the robot_state_publisher is launched
**Then** TF transforms are published for base_link, laser_frame, imu_link
**And** transforms match physical robot dimensions

**Given** the TF tree is active
**When** I run `ros2 run tf2_tools view_frames`
**Then** the complete transform tree is visualized
**And** base_link is the root frame with proper child relationships

**Given** the URDF is loaded
**When** visualized in RViz
**Then** the robot model matches the physical tracked robot footprint
**And** sensor positions are correctly placed

### Story 2.3: SLAM Mapping

As a **robot operator**,
I want **to build a 2D occupancy map of the vineyard using SLAM**,
So that **the robot knows its environment for autonomous navigation**.

**Acceptance Criteria:**

**Given** the robot is in an unmapped area
**When** I launch slam_toolbox in mapping mode
**Then** an occupancy grid is built from LiDAR scans
**And** the map is published to /map topic

**Given** SLAM is running
**When** I drive the robot around using teleoperation
**Then** the map grows as new areas are explored
**And** robot position is tracked on the map with <30cm accuracy (FR2)

**Given** a mapping session is complete
**When** I run the map save service
**Then** the map is saved as PGM + YAML files
**And** can be loaded for future navigation sessions

**Given** GPS signal is unavailable (under vine canopy)
**When** the robot is navigating
**Then** SLAM-only localization continues to function (FR7)
**And** position accuracy remains within acceptable limits

### Story 2.4: Map Visualization on Dashboard

As a **robot operator**,
I want **to see the map and robot position on the dashboard**,
So that **I can monitor where the robot is in the vineyard**.

**Acceptance Criteria:**

**Given** a map has been created via SLAM
**When** the dashboard loads
**Then** the occupancy grid map is displayed
**And** the map can be panned and zoomed

**Given** the robot is localized on the map
**When** telemetry updates are received
**Then** robot position is shown as an icon on the map (FR21)
**And** position updates in real-time (<500ms latency)

**Given** the LiDAR is active
**When** I enable LiDAR visualization
**Then** current scan points are overlaid on the map (FR22)
**And** obstacles are visible as point clusters

### Story 2.5: Waypoint Navigation

As a **robot operator**,
I want **to send the robot to specific waypoints autonomously**,
So that **the robot can navigate without manual control**.

**Acceptance Criteria:**

**Given** the robot is localized on a saved map
**When** I click a destination on the dashboard map
**Then** a navigation goal is sent to Nav2
**And** the robot plans a path to the destination

**Given** Nav2 receives a navigation goal
**When** planning completes
**Then** the planned path is displayed on the dashboard
**And** the robot begins following the path

**Given** the robot is navigating to a waypoint
**When** it reaches the destination
**Then** navigation completes successfully
**And** robot state returns to "idle"

**Given** a navigation goal is active
**When** I send a new goal
**Then** the previous goal is cancelled
**And** navigation to the new goal begins

### Story 2.6: Obstacle Detection & Avoidance

As a **robot operator**,
I want **the robot to detect and avoid obstacles during navigation**,
So that **the robot safely navigates around unexpected objects**.

**Acceptance Criteria:**

**Given** the robot is navigating autonomously
**When** an obstacle is detected within 1m (FR17)
**Then** the robot stops or replans its path
**And** does not collide with the obstacle

**Given** a new obstacle appears on the planned path
**When** Nav2 detects the obstruction
**Then** a new path is computed within 2 seconds (NFR4)
**And** the robot follows the new path around the obstacle

**Given** the obstacle blocks all possible paths
**When** replanning fails
**Then** the robot stops safely
**And** reports "path blocked" status to dashboard

**Given** the obstacle is removed
**When** the path becomes clear
**Then** navigation can resume
**And** robot continues to original destination

---

## Epic 3: Vineyard Row Operations

Pierrot can have the robot follow vine row centerlines, execute headland turns, and operate on sloped terrain—the core vineyard mowing behavior.

### Story 3.1: Row Centerline Detection

As a **robot operator**,
I want **the robot to detect vine row boundaries using LiDAR**,
So that **it can calculate and follow the row centerline**.

**Acceptance Criteria:**

**Given** the robot is positioned in a vine row
**When** the row detection algorithm processes LiDAR data
**Then** it identifies left and right row boundaries (vine posts/wires)
**And** calculates the centerline between them

**Given** row boundaries are detected
**When** the row width is between 1.5m and 2.5m (DOM5)
**Then** a valid centerline is published
**And** confidence score indicates detection quality

**Given** the robot is in a row
**When** one boundary is partially obscured
**Then** the algorithm uses the visible boundary to estimate centerline
**And** flags reduced confidence

### Story 3.2: Row Following Controller

As a **robot operator**,
I want **the robot to follow the row centerline accurately**,
So that **mowing coverage is consistent across the row**.

**Acceptance Criteria:**

**Given** a valid centerline is detected
**When** row following mode is active
**Then** the robot maintains ±20cm lateral deviation from centerline (FR4)
**And** deviation is displayed on dashboard

**Given** the robot is following a row
**When** the centerline curves slightly
**Then** the robot adjusts its heading smoothly
**And** continues tracking the centerline

**Given** the robot is on a slope up to 25°
**When** row following is active
**Then** the robot maintains traction without slipping backward (FR11)
**And** lateral deviation remains within tolerance

### Story 3.3: Headland Turn Execution

As a **robot operator**,
I want **the robot to execute 180° turns at row ends**,
So that **it can proceed to the next row automatically**.

**Acceptance Criteria:**

**Given** the robot is following a row
**When** it reaches the row end (headland)
**Then** it executes a 180° turn (FR5)
**And** aligns with the adjacent row entry

**Given** the headland turn is initiated
**When** the turn completes
**Then** the robot is facing the opposite direction
**And** positioned to enter the next row

**Given** the headland has limited space
**When** a standard turn cannot complete
**Then** the robot performs a multi-point turn
**And** completes the maneuver safely

### Story 3.4: Navigation Pause & Resume

As a **robot operator**,
I want **to pause and resume navigation without losing progress**,
So that **I can intervene when needed and continue where I left off**.

**Acceptance Criteria:**

**Given** the robot is navigating (row following or waypoint)
**When** I press "Pause" on the dashboard
**Then** the robot stops at its current position
**And** navigation state is preserved

**Given** navigation is paused
**When** I press "Resume"
**Then** the robot continues from its current position (FR6)
**And** resumes the previous navigation goal

**Given** the robot is paused
**When** I do not resume within 5 minutes
**Then** the robot remains stationary (does not timeout)
**And** can still be resumed

### Story 3.5: Geofence Boundaries

As a **robot operator**,
I want **the robot to respect defined boundaries**,
So that **it stays within the designated operating area**.

**Acceptance Criteria:**

**Given** geofence boundaries are defined in the map
**When** the robot approaches a boundary
**Then** it does not cross the boundary (FR16)
**And** replans path to stay within allowed area

**Given** a navigation goal is outside the geofence
**When** Nav2 attempts to plan
**Then** the goal is rejected
**And** user is notified that destination is out of bounds

**Given** the robot is near a geofence boundary
**When** attempting to replan around an obstacle
**Then** the planner respects the geofence
**And** does not create paths through forbidden areas

---

## Epic 4: Mission Autonomy

Pierrot can create multi-row mowing missions, start them, monitor progress, and receive notifications—enabling hands-off autonomous operation.

### Story 4.1: Zone Definition Interface

As a **robot operator**,
I want **to draw mowing zones on the map**,
So that **I can define which areas the robot should cover**.

**Acceptance Criteria:**

**Given** the map is displayed on the dashboard
**When** I enter zone editing mode
**Then** I can draw polygon boundaries on the map (FR26)
**And** the zone is highlighted visually

**Given** I have drawn a zone polygon
**When** I save the zone
**Then** it is stored with a name and color
**And** appears in the zone list

**Given** zones are defined
**When** I view the map
**Then** all zones are displayed with their boundaries
**And** I can edit or delete existing zones

### Story 4.2: Mission Creation

As a **robot operator**,
I want **to create missions by selecting zones and row sequences**,
So that **I can plan the robot's work for the day**.

**Acceptance Criteria:**

**Given** zones are defined on the map
**When** I create a new mission (FR27)
**Then** I can select which zones to include
**And** define the row sequence (order of rows to mow)

**Given** I am creating a mission
**When** I add waypoints for row entry/exit
**Then** the mission shows the planned path
**And** estimated coverage time is displayed

**Given** a mission is defined
**When** I finalize the mission
**Then** it is validated for feasibility
**And** any issues are reported (unreachable areas, etc.)

### Story 4.3: Mission Execution Engine

As a **robot operator**,
I want **the robot to execute multi-row missions autonomously**,
So that **it can mow multiple rows without my intervention**.

**Acceptance Criteria:**

**Given** a valid mission is defined
**When** I start the mission
**Then** the robot begins executing the first row (FR30)
**And** proceeds through all rows in sequence

**Given** the robot completes a row
**When** it reaches the headland
**Then** it executes the headland turn
**And** begins the next row in the mission

**Given** the mission is executing
**When** an obstacle blocks progress
**Then** the robot attempts to navigate around it
**And** if blocked, enters "obstacle blocked" state

**Given** an obstacle cannot be avoided
**When** the robot is blocked
**Then** "skip to next row" command is available (FR31)
**And** operator can skip the blocked section

### Story 4.4: Mission Control (Start/Pause/Resume/Abort)

As a **robot operator**,
I want **full control over mission execution**,
So that **I can manage the robot's work as conditions change**.

**Acceptance Criteria:**

**Given** a mission is defined
**When** I press "Start Mission"
**Then** the robot begins mission execution (FR28)
**And** dashboard shows "Mission Active" state

**Given** a mission is active
**When** I press "Pause"
**Then** the robot stops at current position
**And** mission state is preserved

**Given** a mission is paused
**When** I press "Resume"
**Then** the robot continues from current position
**And** completes remaining mission tasks

**Given** a mission is active or paused
**When** I press "Abort"
**Then** the mission is cancelled
**And** robot returns to idle state

### Story 4.5: Mission Progress Display

As a **robot operator**,
I want **to see real-time mission progress on the dashboard**,
So that **I know how much work is completed and remaining**.

**Acceptance Criteria:**

**Given** a mission is active
**When** the robot is working
**Then** progress percentage is displayed (FR23)
**And** current row number is shown

**Given** the mission is progressing
**When** viewing the map
**Then** completed rows are visually marked
**And** current position and remaining path are shown

**Given** the robot encounters an issue
**When** progress is blocked
**Then** the blocked location is highlighted
**And** estimated completion time is updated

### Story 4.6: Push Notifications

As a **robot operator**,
I want **to receive notifications for mission events**,
So that **I can monitor progress without watching the dashboard constantly**.

**Acceptance Criteria:**

**Given** notifications are enabled
**When** a mission completes successfully
**Then** a push notification is sent (FR24)
**And** notification includes completion summary

**Given** the robot encounters an error
**When** intervention is required
**Then** an alert notification is sent immediately
**And** includes error details and suggested action

**Given** battery is low during mission
**When** charge drops below threshold
**Then** low-battery notification is sent
**And** estimated remaining runtime is included

### Story 4.7: Mission Persistence

As a **robot operator**,
I want **to save and recall mission patterns**,
So that **I can reuse missions for recurring mowing schedules**.

**Acceptance Criteria:**

**Given** I have created a mission
**When** I select "Save Mission"
**Then** the mission is stored with a name (FR29)
**And** appears in the saved missions list

**Given** saved missions exist
**When** I view the missions list
**Then** all saved missions are displayed
**And** I can select one to load

**Given** I load a saved mission
**When** the mission loads
**Then** all zones, rows, and waypoints are restored
**And** I can start the mission or modify it

---

## Epic 5: System Administration

Pierrot (in admin mode) can diagnose issues, update firmware, configure safety parameters, and maintain the robot between seasons.

### Story 5.1: Safety Event Reporting

As a **system administrator**,
I want **to see all safety events on the dashboard**,
So that **I can understand what triggered safety systems**.

**Acceptance Criteria:**

**Given** a safety event occurs (E-stop, watchdog, tilt)
**When** the event is triggered
**Then** it is logged with timestamp and details (FR18)
**And** displayed in the safety events panel

**Given** safety events are logged
**When** I view the safety history
**Then** I can see all past events
**And** filter by event type and date range

**Given** a tilt event occurred
**When** viewing event details
**Then** pitch and roll angles are shown
**And** location at time of event is displayed

### Story 5.2: System Diagnostics

As a **system administrator**,
I want **to view comprehensive system diagnostics**,
So that **I can identify issues and verify system health**.

**Acceptance Criteria:**

**Given** I open the diagnostics panel
**When** the panel loads
**Then** sensor status is displayed for all sensors (FR32)
**And** each sensor shows online/offline/error state

**Given** diagnostics are displayed
**When** viewing firmware information
**Then** Arduino firmware version is shown
**And** ROS 2 package versions are listed

**Given** a sensor is in error state
**When** viewing its details
**Then** error description is provided
**And** suggested troubleshooting steps are shown

### Story 5.3: Self-Test Sequence

As a **system administrator**,
I want **to run automated self-tests**,
So that **I can verify all subsystems before operation**.

**Acceptance Criteria:**

**Given** the robot is powered on and idle
**When** I initiate self-test (FR33)
**Then** a sequence of subsystem tests begins
**And** progress is displayed for each test

**Given** self-test is running
**When** each subsystem is tested
**Then** motors, sensors, communication are verified
**And** results (pass/fail) are recorded

**Given** self-test completes
**When** viewing results
**Then** overall status (pass/fail) is displayed
**And** any failed tests are highlighted with details

### Story 5.4: Firmware Update Interface

As a **system administrator**,
I want **to update Arduino firmware from the dashboard**,
So that **I can deploy improvements without physical access to the Arduino**.

**Acceptance Criteria:**

**Given** I have a new firmware file
**When** I upload it via the dashboard (FR34)
**Then** the file is transferred to RPi
**And** validation confirms it's a valid firmware image

**Given** firmware is uploaded
**When** I initiate the update
**Then** platformio CLI flashes the Arduino via USB
**And** progress is displayed

**Given** firmware update completes
**When** the Arduino restarts
**Then** new version is verified
**And** confirmation is displayed on dashboard

**Given** firmware update fails
**When** an error occurs
**Then** rollback instructions are provided
**And** current (old) firmware continues to function

### Story 5.5: Operational Logging

As a **system administrator**,
I want **comprehensive operational logs**,
So that **I can review sessions and debug issues**.

**Acceptance Criteria:**

**Given** the robot is operating
**When** events occur
**Then** they are logged with timestamps and severity (FR35)
**And** logs include telemetry, commands, and state changes

**Given** I want to export logs
**When** I select a date range
**Then** logs are exported to downloadable file (FR36)
**And** format is human-readable (JSON or CSV)

**Given** logs are accumulating
**When** storage approaches limits
**Then** old logs are rotated automatically
**And** user is notified of log rotation

### Story 5.6: Safety Configuration

As a **system administrator**,
I want **to configure safety thresholds**,
So that **I can tune safety parameters for different conditions**.

**Acceptance Criteria:**

**Given** I open safety configuration
**When** viewing current settings
**Then** tilt threshold is displayed (default 25°) (FR37)
**And** speed limits are shown

**Given** I want to adjust tilt threshold
**When** I enter a new value (e.g., 20°)
**Then** the value is validated (range 15°-30°)
**And** saved to configuration

**Given** speed limits need adjustment
**When** I set new max linear/angular speeds
**Then** values are validated against hardware limits
**And** applied to motor control

**Given** configuration is changed
**When** changes are saved
**Then** they take effect immediately
**And** are persisted across restarts

### Story 5.7: Sensor Calibration

As a **system administrator**,
I want **to calibrate IMU and encoders**,
So that **sensor readings are accurate**.

**Acceptance Criteria:**

**Given** I initiate IMU calibration (FR40)
**When** the calibration wizard starts
**Then** instructions guide me through the process
**And** calibration data is collected

**Given** IMU calibration data is collected
**When** calibration completes
**Then** offsets are calculated and saved
**And** improved accuracy is verified

**Given** encoder calibration is needed
**When** I run encoder calibration
**Then** the robot moves a known distance
**And** encoder ticks-per-meter is calculated

**Given** calibration is complete
**When** returning to normal operation
**Then** calibrated values are used
**And** can be reset to defaults if needed
