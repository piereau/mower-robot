---
stepsCompleted:
  - step-01-init
  - step-02-discovery
  - step-03-success
  - step-04-journeys
  - step-05-domain
  - step-06-innovation
  - step-07-project-type
  - step-08-scoping
  - step-09-functional
  - step-10-nonfunctional
  - step-11-polish
  - step-12-complete
status: complete
inputDocuments:
  - _bmad-output/planning-artifacts/product-brief-mower-roboto-2026-01-28.md
  - _bmad-output/brainstorming/brainstorming-session-2026-01-28.md
  - _bmad-output/planning-artifacts/architecture.md
  - README.md
classification:
  projectType: iot_embedded
  domain: agricultural-robotics
  complexity: medium
  projectContext: brownfield
documentCounts:
  briefs: 1
  research: 0
  brainstorming: 1
  architecture: 1
  projectDocs: 1
workflowType: 'prd'
project_name: 'mower-roboto'
user_name: 'Pierrot'
date: '2026-01-29'
---

# Product Requirements Document - mower-roboto

**Author:** Pierrot  
**Date:** 2026-01-29  
**Version:** 1.0

---

## Executive Summary

**mower-roboto** is an autonomous vineyard row mower — a tracked robot designed to autonomously mow weeds between vine rows without human intervention.

### Problem Statement

Vineyard inter-row maintenance (mechanical weeding, mowing) is repetitive, time-consuming, and physically demanding. Current solutions are either manual (expensive labor), herbicide-based (environmental/regulatory pressure), or commercial robots that are costly and poorly adapted to uneven agricultural terrain.

### Proposed Solution

A tracked autonomous robot capable of:
- Navigating between vine rows without human intervention
- Mapping the vineyard domain (SLAM + RTK-GPS)
- Avoiding obstacles (stakes, wires, vegetation)
- Performing headland turns at row ends
- Remote supervision via web/mobile interface

### Key Differentiators

| Differentiator | Value |
|----------------|-------|
| **Tracked locomotion** | Superior traction on soft/sloped terrain vs. wheeled robots |
| **Hybrid ROS 2 + Arduino architecture** | Industrial robustness + real-time safety |
| **Dual localization (SLAM + RTK)** | Reliability even under vine canopy |
| **Cost-controlled** | Accessible components (RPi, Arduino, LiDAR LD19) |
| **Modular architecture** | Ready for evolution from prototype to commercial product |

### Project Context

This is a **brownfield project** with an existing POC:
- **Current state:** WebSocket telemetry dashboard (FastAPI + React) displaying robot state and battery
- **Target state:** Full autonomous navigation and control for vineyard row mowing

---

## Success Criteria

### User Success

| Criterion | Measure |
|-----------|---------|
| **Autonomous row completion** | Robot completes one full vine row pass without manual intervention |
| **Reliable operation** | Robot operates for 2+ hours without requiring restart or recovery |
| **Easy supervision** | User can monitor robot status from smartphone while doing other tasks |
| **Minimal setup** | Initial vineyard mapping completed in < 1 day |
| **Trust building** | User comfortable leaving robot unsupervised after 5 successful sessions |

### Business Success (Academic → Demo)

| Phase | Success Measure |
|-------|-----------------|
| **Prototype validation** | Successful autonomous operation in personal vineyard |
| **Demo readiness** | 3 external viticulteurs witness successful demonstration |
| **Technical credibility** | Architecture documented sufficiently for academic evaluation |

### Technical Success

| Criterion | Measure |
|-----------|---------|
| **Control loop reliability** | Motor commands execute at 50Hz without drops |
| **Safety response** | E-stop triggers motor shutoff within 200ms |
| **Localization accuracy** | Position error < 30cm in mapped areas |
| **Navigation robustness** | Robot recovers from GPS dropout using SLAM fallback |

### Measurable Outcomes

**MVP Success Moment:** Robot autonomously mows one complete vine row (entry → mowing → headland turn → next row entry) while user monitors from supervision dashboard.

**Demo Success Moment:** External viticulteur says "I would use this on my vineyard."

---

## Product Scope

### MVP - Phase 1: Foundation & Teleoperation

**Goal:** Reliable manual control with real-time monitoring

| Capability | Description |
|------------|-------------|
| Motor control | Teleoperation via joystick commands through WebSocket |
| Real-time telemetry | Battery, motor status, safety state visible on dashboard |
| Safety system | Watchdog, E-stop, tilt detection operational |
| Basic sensing | LiDAR data streaming, obstacle proximity alerts |

### Phase 2: Autonomous Navigation

**Goal:** Robot navigates autonomously within mapped area

| Capability | Description |
|------------|-------------|
| SLAM mapping | Build 2D occupancy map of vineyard section |
| Waypoint navigation | Navigate to user-defined waypoints via Nav2 |
| Obstacle avoidance | Dynamic obstacle detection and path replanning |
| Row following | Follow vine row centerline using LiDAR or camera |

### Phase 3: Mission Autonomy

**Goal:** Complete multi-row mowing missions autonomously

| Capability | Description |
|------------|-------------|
| Mission planning | Define mowing zones, row sequences, coverage patterns |
| Headland maneuvers | Automated 180° turns at row ends |
| Progress tracking | Real-time mission progress on supervision dashboard |
| Resume capability | Resume interrupted missions from last known position |

### Vision - Future Phases

| Capability | Description |
|------------|-------------|
| RTK-GPS integration | Centimeter-level positioning for precision agriculture |
| Multi-robot coordination | Fleet management for large vineyards |
| Camera-based row detection | Visual guidance for row centering |
| Cloud analytics | Operational data, maintenance predictions, usage reports |

---

## User Journeys

### Journey 1: Pierrot — First Autonomous Row (Primary User, Success Path)

**Persona:** Pierrot, viticulteur indépendant, 15 ha vineyard, technically capable, building this robot for his own use.

**Opening Scene:** It's 7 AM on a June morning. Pierrot walks to the edge of his vineyard with the mower-roboto on a small trailer. The inter-rows are overgrown after spring rains. Normally, this means 3 days of manual brush-cutting in the heat.

**Rising Action:**
1. Pierrot unloads the robot at row 1 entry point
2. Opens supervision app on smartphone, sees "Connected" indicator
3. Confirms battery is 95%, all systems green
4. Selects "Row 1 → Row 2" mission from saved patterns
5. Taps "Start Mission" — robot begins moving
6. Walks alongside for first 20 meters, watching LiDAR scan on dashboard
7. Gains confidence, returns to farmhouse to prepare coffee

**Climax:** From kitchen window, Pierrot checks phone. Dashboard shows robot at 60% through Row 1. No alerts. He smiles — the first time he's not physically present during mowing.

**Resolution:** Robot completes Row 1, executes headland turn, enters Row 2. Mission completes 45 minutes later. Pierrot receives "Mission Complete" notification. He retrieves the robot — battery at 40%, rows cleanly mowed. The work that would take a full morning is done while he handled administrative tasks.

**Journey reveals:** Mission control, real-time progress, notification system, battery monitoring, confidence-building through progressive autonomy.

---

### Journey 2: Pierrot — Obstacle Recovery (Primary User, Edge Case)

**Opening Scene:** Day 15 of using mower-roboto. Pierrot has grown confident, often letting the robot work while he's in another part of the vineyard.

**Rising Action:**
1. Robot is mid-mission in Row 7 when it encounters an unexpected obstacle — a fallen branch from recent wind
2. Robot stops, enters "Obstacle Blocked" state
3. Pierrot's phone vibrates with alert: "Mission paused — obstacle detected"
4. Opens app, sees camera feed showing the branch
5. Robot has already attempted replanning but branch blocks entire row width

**Climax:** Pierrot has options:
- Walk to robot and remove branch (robot auto-resumes)
- Send "Skip to next row" command
- Abort mission entirely

He walks over (5 minutes), removes branch, robot automatically resumes. No data loss, no restart needed.

**Resolution:** Robot completes remaining rows. Pierrot notes this was the first real-world obstacle. System handled it gracefully — he was informed, given options, and recovery was seamless.

**Journey reveals:** Alert system, obstacle handling, manual intervention workflow, mission resumption, camera/sensor feed for remote diagnosis.

---

### Journey 3: Demo Day — Visiting Viticulteur (Secondary User)

**Persona:** Marc, 50-year-old viticulteur from neighboring village, skeptical of technology, heard about Pierrot's robot project.

**Opening Scene:** Marc arrives at Pierrot's vineyard, arms crossed. "Show me this robot of yours. I've seen expensive gadgets that don't work in real conditions."

**Rising Action:**
1. Pierrot hands Marc the smartphone showing supervision dashboard
2. "See? This is the robot's current view — LiDAR scan of the row, battery status, speed."
3. Pierrot starts a short demo mission — just 2 rows
4. Robot begins moving. Marc watches the physical robot while checking the screen
5. Robot encounters slight slope, adjusts trajectory
6. "How does it know where to go?" — Pierrot shows the map overlay

**Climax:** Robot completes headland turn. Marc leans in. "It turned by itself? No wires buried?" 

Pierrot: "No wires. It builds a map from the LiDAR and remembers where it's been."

Marc watches the second row. Robot finishes, returns to start point.

**Resolution:** Marc is quiet for a moment. "How much would something like this cost? My son is leaving for the city — I can't keep doing this manually."

This is the demo success moment. Technical proof becomes human interest.

**Journey reveals:** Demo mode, simple UI for non-technical observers, visual feedback (map overlay), explanation touchpoints, compelling value demonstration.

---

### Journey 4: System Administrator — Firmware Update (Operations)

**Persona:** Pierrot in "admin mode" — maintaining the system between seasons.

**Opening Scene:** Winter maintenance. Pierrot has developed a new safety feature (tilt threshold adjustment) and needs to update Arduino firmware.

**Rising Action:**
1. Connects laptop to RPi via SSH
2. Runs diagnostic to confirm current firmware version
3. Uploads new Arduino sketch via platformio CLI
4. Runs self-test sequence — motors, sensors, watchdog
5. Checks logs for any warnings

**Climax:** Self-test passes. Tilt detection now triggers at 25° instead of 30° — safer on steep slopes.

**Resolution:** Robot is ready for next season with improved safety parameters. All changes logged.

**Journey reveals:** Firmware update workflow, diagnostic tools, self-test capability, configuration management, change logging.

---

### Journey Requirements Summary

| Journey | Key Capabilities Required |
|---------|---------------------------|
| First Autonomous Row | Mission control, progress tracking, notifications, battery monitoring |
| Obstacle Recovery | Alerts, obstacle detection, remote diagnosis, mission resumption |
| Demo Day | Demo mode, simple UI, map visualization, explanation features |
| Firmware Update | SSH access, platformio integration, self-test, diagnostics |

---

## Domain-Specific Requirements

### Agricultural Robotics Context

This product operates in outdoor agricultural environments with specific challenges:

| Challenge | Requirement |
|-----------|-------------|
| **Terrain variability** | Handle slopes up to 25°, soft soil, uneven surfaces |
| **Weather exposure** | IP54 minimum protection (dust/splash resistant) |
| **GPS coverage** | Fallback to SLAM when canopy blocks satellite signals |
| **Obstacle diversity** | Detect stakes, wires, vegetation, animals, humans |
| **Long operation cycles** | 2+ hour continuous operation between charges |

### Safety Requirements (Academic Scope)

No formal CE certification required, but practical safety measures are essential:

| Safety Measure | Implementation |
|----------------|----------------|
| **E-stop software** | Immediate motor shutoff via dashboard command |
| **Watchdog timeout** | Arduino stops motors if RPi silent for 200ms |
| **Tilt detection** | IMU triggers stop if pitch/roll exceeds 25° |
| **Geofencing** | Nav2 keepout zones prevent leaving mapped area |
| **Failsafe default** | Motors OFF is the safe default state |

### Operating Constraints

| Constraint | Specification |
|------------|---------------|
| **Operating temperature** | 0°C to 40°C |
| **Vegetation height** | Up to 30cm (mower blade clearance) |
| **Row width** | 1.5m to 2.5m (typical vine inter-row) |
| **Maximum slope** | 25° (tracked locomotion limit) |

---

## Innovation Analysis

### Innovation Areas

| Area | Innovation |
|------|------------|
| **Hybrid architecture** | ROS 2 high-level planning + Arduino real-time safety — uncommon in DIY/academic robots |
| **Dual localization** | SLAM + RTK-GPS fusion for agricultural reliability under canopy |
| **Tracked platform for viticulture** | Most vineyard robots use wheels; tracks offer superior traction on slopes |
| **Cost-accessible design** | Full autonomous capability at fraction of commercial robot cost |

### Validation Approach

| Innovation | Validation Method |
|------------|-------------------|
| Hybrid architecture | Demonstrate failsafe operation when RPi crashes |
| Dual localization | Test navigation continuity during GPS blackout |
| Tracked performance | Compare slippage on 20° slope vs. wheeled prototype |
| Cost model | Document BOM and compare to commercial alternatives |

### Risk Mitigation

| Risk | Mitigation |
|------|------------|
| ROS 2 complexity on RPi | Start with minimal nodes, optimize incrementally |
| SLAM drift in repetitive rows | Use RTK-GPS anchoring when available |
| Arduino-RPi communication failure | Binary protocol with CRC, automatic reconnection |

---

## IoT/Embedded Specific Requirements

### Hardware Architecture

| Component | Specification |
|-----------|---------------|
| **Compute** | Raspberry Pi 4B (4GB RAM) |
| **Real-time control** | Arduino Nano (motor PID, safety watchdog) |
| **LiDAR** | LD19 (8-15 Hz scan rate, 12m range) |
| **IMU** | 9-axis (ICM-20948 or equivalent) |
| **Motors** | 2x DC brushed with H-bridge, encoder feedback |
| **Connectivity** | WiFi (802.11n) for supervision link |

### Communication Protocols

| Interface | Protocol | Frequency |
|-----------|----------|-----------|
| RPi ↔ Arduino | Serial binary + CRC16 | 50 Hz bidirectional |
| RPi ↔ Dashboard | WebSocket (JSON) | 10 Hz telemetry |
| RPi ↔ Sensors | ROS 2 topics | Per-sensor native rate |

### Power Requirements

| Aspect | Specification |
|--------|---------------|
| **Battery type** | LiPo or LiFePO4, 24V nominal |
| **Capacity target** | 2+ hours operation at moderate load |
| **Charging** | Manual plug-in (future: docking station) |
| **Power monitoring** | Voltage sensing with 10% low-battery warning |

### Connectivity Requirements

| Requirement | Specification |
|-------------|---------------|
| **Range** | 100m+ WiFi coverage (vineyard scale) |
| **Latency tolerance** | Telemetry: 500ms acceptable; E-stop: <200ms |
| **Offline operation** | Robot continues mission if supervision link drops |
| **Reconnection** | Automatic WebSocket reconnection with exponential backoff |

---

## Functional Requirements

### Navigation & Localization

- FR1: Robot can build 2D occupancy map of operating area using SLAM
- FR2: Robot can localize within mapped area with <30cm accuracy
- FR3: Robot can navigate to specified waypoints avoiding obstacles
- FR4: Robot can follow vine row centerline maintaining ±20cm lateral deviation
- FR5: Robot can execute 180° headland turns at row ends
- FR6: Robot can resume navigation from current position after pause/stop
- FR7: Robot can fall back to SLAM-only localization when GPS is unavailable

### Motion Control

- FR8: Operator can control robot motion via joystick commands (teleoperation)
- FR9: Robot can execute velocity commands (linear + angular) at 50Hz
- FR10: Robot can stop within 0.5m from full speed when commanded
- FR11: Robot can traverse slopes up to 25° without slipping backward
- FR12: Robot can rotate in place for tight maneuvering

### Safety & Protection

- FR13: Operator can trigger immediate motor stop via dashboard E-stop button
- FR14: Robot automatically stops if RPi-Arduino communication fails for 200ms
- FR15: Robot automatically stops if tilt exceeds 25° (pitch or roll)
- FR16: Robot respects geofence boundaries defined in map
- FR17: Robot detects obstacles within 1m and stops or replans path
- FR18: Robot reports safety events (E-stop, watchdog, tilt) to supervision dashboard

### Supervision & Monitoring

- FR19: User can view real-time robot state (idle, navigating, mowing, error) on dashboard
- FR20: User can view battery level with low-battery warning
- FR21: User can view robot position on map overlay
- FR22: User can view LiDAR scan visualization
- FR23: User can view mission progress (% complete, current row)
- FR24: User receives push notification on mission complete or alert
- FR25: Dashboard shows connection status with automatic reconnection

### Mission Management

- FR26: User can define mowing zones on map
- FR27: User can create missions (sequence of rows/waypoints)
- FR28: User can start, pause, resume, and abort missions
- FR29: User can save and recall mission patterns
- FR30: Robot can execute multi-row missions autonomously
- FR31: User can send "skip to next row" command during obstacle situations

### Diagnostics & Maintenance

- FR32: User can view system diagnostics (sensor status, firmware versions)
- FR33: User can run self-test sequence to verify all subsystems
- FR34: User can update Arduino firmware via RPi
- FR35: System logs operational data for post-session review
- FR36: User can export logs for debugging

### Configuration

- FR37: User can configure safety thresholds (tilt limit, speed limits)
- FR38: User can configure notification preferences
- FR39: User can define keepout zones in map
- FR40: User can calibrate sensors (IMU, encoders)

---

## Non-Functional Requirements

### Performance

| Metric | Requirement |
|--------|-------------|
| **Control loop frequency** | Motor commands at 50Hz minimum |
| **Telemetry latency** | Dashboard update within 500ms of state change |
| **E-stop response** | Motor shutoff within 200ms of command |
| **Navigation replanning** | New path computed within 2 seconds |
| **Boot time** | Robot operational within 60 seconds of power-on |

### Reliability

| Metric | Requirement |
|--------|-------------|
| **Continuous operation** | 2+ hours without requiring restart |
| **Communication recovery** | Auto-reconnect within 10 seconds after WiFi drop |
| **Mission persistence** | Mission state survives brief (<5s) communication interruption |
| **Mean time between failures** | >50 operating hours between manual interventions |

### Scalability

| Metric | Requirement |
|--------|-------------|
| **Map size** | Support maps up to 10 ha (academic scope) |
| **Mission complexity** | Support missions with up to 50 waypoints |
| **Concurrent users** | Single supervisor (multi-user future phase) |

### Maintainability

| Metric | Requirement |
|--------|-------------|
| **Code modularity** | ROS 2 packages independently deployable |
| **Configuration management** | All parameters in YAML files, no hardcoded values |
| **Logging** | Structured logs with severity levels for debugging |
| **Documentation** | README for each major component |

### Security

| Metric | Requirement |
|--------|-------------|
| **Network access** | Dashboard accessible only on local network (no cloud exposure for MVP) |
| **Command authentication** | WebSocket commands require session token |
| **Firmware integrity** | Firmware updates only via authenticated SSH session |

### Usability

| Metric | Requirement |
|--------|-------------|
| **Mobile-first UI** | Dashboard functional on smartphone screen |
| **Outdoor readability** | High-contrast UI elements for sunlight visibility |
| **Error messages** | User-friendly error descriptions with suggested actions |
| **Learning curve** | New user completes first teleoperation session in <15 minutes |

---

## Technical Architecture Reference

The architecture document (`architecture.md`) defines the complete technical implementation. Key architectural decisions:

| Decision | Choice |
|----------|--------|
| **Middleware** | ROS 2 Humble on RPi 4B |
| **SLAM** | slam_toolbox |
| **Navigation** | Nav2 |
| **Sensor fusion** | robot_localization (EKF) |
| **Real-time control** | Arduino Nano with binary serial protocol |
| **Supervision backend** | FastAPI (existing) |
| **Supervision frontend** | React + Vite + Tailwind (existing) |

### Implementation Phases

| Phase | Focus | Deliverables |
|-------|-------|--------------|
| **Phase 1** | Foundation | Serial protocol, motor control, teleoperation, telemetry |
| **Phase 2** | Perception | LiDAR integration, SLAM mapping |
| **Phase 3** | Navigation | Nav2 configuration, waypoint following, obstacle avoidance |
| **Phase 4** | Autonomy | Mission manager, row-by-row execution, headland maneuvers |

---

## Appendix: Glossary

| Term | Definition |
|------|------------|
| **SLAM** | Simultaneous Localization and Mapping — building a map while tracking position |
| **Nav2** | ROS 2 navigation stack for autonomous mobile robots |
| **EKF** | Extended Kalman Filter — sensor fusion algorithm |
| **RTK-GPS** | Real-Time Kinematic GPS — centimeter-level positioning |
| **Headland** | Area at row ends where robot turns around |
| **Teleoperation** | Manual remote control of robot motion |
| **Watchdog** | Timer that triggers safety action if not periodically reset |
| **Footprint** | Robot's physical dimensions used for collision checking |

---

## Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-01-29 | Pierrot | Initial PRD created via BMAD workflow |
