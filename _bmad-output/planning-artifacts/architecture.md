---
stepsCompleted: [1, 2, 3, 4, 5, 6, 7, 8]
status: complete
inputDocuments:
  - _bmad-output/planning-artifacts/product-brief-mower-roboto-2026-01-28.md
  - _bmad-output/brainstorming/brainstorming-session-2026-01-28.md
workflowType: 'architecture'
project_name: 'mower-roboto'
user_name: 'Pierrot'
date: '2026-01-28'
---

# Architecture Decision Document

_This document builds collaboratively through step-by-step discovery. Sections are appended as we work through each architectural decision together._

---

## Project Context Analysis

### Requirements Overview

**Functional Requirements:**
- Navigation autonome entre rangs de vigne (suivi de trajectoire, demi-tours)
- Cartographie du domaine (construction et utilisation de carte)
- Évitement d'obstacles (statiques et dynamiques)
- Supervision à distance via interface web/mobile
- Tonte/désherbage mécanique des inter-rangs

**Non-Functional Requirements:**
- **Temps réel**: latence commande moteur < 20ms, boucle contrôle 50-100Hz
- **Sécurité**: arrêt garanti même si RPi plante (watchdog Arduino, E-stop matériel)
- **Robustesse**: fonctionnement avec GPS dégradé (fallback SLAM sous feuillage)
- **Low power**: optimisation autonomie batterie
- **Maintenabilité**: architecture modulaire, séparation des concerns

**Scale & Complexity:**
- Primary domain: Robotique embarquée (ROS 2) + Web (supervision)
- Complexity level: Élevée
- Estimated architectural components: 15-20 modules

### Technical Constraints & Dependencies

- **Compute**: Raspberry Pi 4B (ressources limitées, pas de GPU)
- **Communication**: Série USB/UART vers Arduino (protocole binaire custom)
- **Environnement**: Extérieur agricole (poussière, humidité, vibrations, pentes)
- **Capteurs**: LiDAR LD19, IMU, encodeurs, RPiCam, HC-SR04, (futur) RTK-GPS
- **Locomotion**: Chenilles (glissement important → odométrie imprécise)

### Cross-Cutting Concerns Identified

| Concern | Impact |
|---------|--------|
| **Sécurité** | E-stop HW/SW, watchdog, geofencing, détection basculement |
| **Logging** | Enregistrement trajectoires, replay debug, bags ROS |
| **Configuration** | Paramétrage zones, vitesse, seuils capteurs |
| **Mise à jour** | OTA firmware Arduino + software RPi |
| **Monitoring** | Télémétrie temps réel, alertes, état batterie |

---

## Technology Stack Selection

### Primary Technology Domain

**Robotique embarquée autonome** avec interface de supervision web.

### Stack Sélectionnée

| Couche | Technologie | Version | Justification |
|--------|-------------|---------|---------------|
| **OS** | Raspberry Pi OS 64-bit | Bookworm | Stable, support ROS 2 |
| **Middleware Robot** | ROS 2 | Humble LTS | Standard industrie, Nav2/SLAM |
| **SLAM** | slam_toolbox | 2.6+ | Maintenu, intégré Nav2 |
| **Navigation** | Nav2 | 1.0+ | Comportements, planification |
| **Fusion capteurs** | robot_localization | Humble | EKF odométrie+IMU+GPS |
| **LiDAR Driver** | ldlidar_ros2 | Latest | Support LD19 |
| **Backend UI** | FastAPI | 0.100+ | Async, WebSocket, existant |
| **Frontend** | React + Vite + Tailwind | Latest | Mobile-first, existant |
| **Microcontrôleur** | Arduino (C++) | - | Temps réel, PID |
| **Protocol série** | Custom binaire + CRC16 | - | Robuste, faible latence |

### Starter/Template Approach

**Approche:** Configuration Nav2 + slam_toolbox standard, adaptée pour robot à chenilles.

**Base de référence:** Turtlebot3 navigation stack (robot différentiel similaire)

**Adaptations requises:**
- Footprint rectangulaire (chenilles)
- Modèle odométrique pour glissement
- Paramètres costmap terrain agricole
- Intégration protocole série custom

### Workspace ROS 2 Structure

```
mower_ws/
├── src/
│   ├── mower_bringup/        # Launch files, configs
│   ├── mower_description/    # URDF, TF
│   ├── mower_navigation/     # Nav2 params, behaviors
│   ├── mower_perception/     # LiDAR, camera nodes
│   ├── mower_localization/   # EKF config, SLAM params
│   ├── mower_hardware/       # Serial bridge, motor interface
│   └── mower_msgs/           # Custom messages
└── ...
```

---

## Core Architectural Decisions

### Decision Priority Analysis

**Critical Decisions (Block Implementation):**
- Protocole série RPi↔Arduino
- Structure messages ROS 2
- Safety/watchdog architecture

**Important Decisions (Shape Architecture):**
- Stratégie de logging
- Gestion configuration
- Déploiement et mise à jour

**Deferred Decisions (Post-MVP):**
- RTK-GPS integration
- Multi-robot coordination
- Cloud analytics

### Safety & Security

| Decision | Choice | Rationale |
|----------|--------|-----------|
| **E-stop** | Software only (MVP) | Pas de bouton HW pour prototype |
| **Watchdog** | Arduino 200ms timeout | Stop moteurs si RPi muette |
| **Geofencing** | Nav2 costmap (keepout zones) | Flexible, configurable |
| **Tilt detection** | IMU seuils (pitch/roll > 30°) | Arrêt si basculement |
| **Failsafe default** | Motors OFF | État sûr par défaut |

### Communication ROS ↔ Arduino

| Decision | Choice | Rationale |
|----------|--------|-----------|
| **Bridge node** | Custom Python (pyserial + rclpy) | Contrôle total, debug facile |
| **Baud rate** | 115200 | Fiable, suffisant pour 50Hz |
| **Message format** | Binaire custom + CRC16 | Compact, vérifié |
| **Cmd frequency** | 50 Hz | Suffisant pour chenilles |
| **Odom frequency** | 50 Hz | Synchrone avec cmd |

### Data & Persistence

| Decision | Choice | Rationale |
|----------|--------|-----------|
| **Map storage** | PGM + YAML (standard Nav2) | Compatible slam_toolbox |
| **Robot logs** | ROS 2 bags (rosbag2) | Replay, debug, standard |
| **Config user** | YAML files | Simple, versionnable |
| **Telemetry** | In-memory + WebSocket | Temps réel, pas de DB |

### Deployment & Operations

| Decision | Choice | Rationale |
|----------|--------|-----------|
| **Service manager** | systemd | Standard Linux, robuste |
| **ROS 2 launch** | Via systemd service | Auto-start at boot |
| **Updates** | Git pull + systemctl restart | Simple pour prototype |
| **Monitoring** | journald + WebSocket UI | Pas d'infra supplémentaire |
| **Arduino flash** | USB + platformio CLI | Depuis RPi si besoin |

### Decision Impact Analysis

**Implementation Sequence:**
1. Protocole série + Arduino firmware (base)
2. ROS 2 serial bridge node
3. URDF + TF setup
4. LiDAR integration + SLAM
5. Nav2 configuration
6. UI integration (WebSocket telemetry)

**Cross-Component Dependencies:**
- Serial bridge ← dépend du protocole Arduino
- EKF ← dépend des encodeurs + IMU
- Nav2 ← dépend de SLAM + EKF
- UI ← dépend de tous (télémétrie)

---

## Implementation Patterns

### Naming Conventions

| Context | Convention | Example |
|---------|------------|---------|
| ROS 2 package | `snake_case` | `mower_navigation` |
| ROS 2 node | `snake_case` | `serial_bridge_node` |
| ROS 2 topic | `/mower/snake_case` | `/mower/cmd_vel` |
| ROS 2 message | `PascalCase` | `MowerStatus.msg` |
| Python file | `snake_case.py` | `serial_bridge.py` |
| Arduino function | `camelCase` | `setMotorSpeed()` |
| Arduino constant | `UPPER_SNAKE` | `WATCHDOG_TIMEOUT_MS` |

### Serial Protocol Format

```
┌──────┬─────┬──────┬─────┬─────┬─────────┬───────┐
│ SOF  │ VER │ TYPE │ SEQ │ LEN │ PAYLOAD │ CRC16 │
│ 2B   │ 1B  │ 1B   │ 1B  │ 1B  │ N bytes │ 2B    │
└──────┴─────┴──────┴─────┴─────┴─────────┴───────┘
SOF = 0xAA55
TYPE 0x00-0x7F = commands (RPi→Arduino)
TYPE 0x80-0xFF = telemetry (Arduino→RPi)
```

### Error Handling

- **Arduino**: Fail-safe (motors OFF), set status bit, never crash
- **ROS 2**: Log via rclpy, nodes survive transient errors
- **Safety node**: Separate process, independent failure domain

---

## Project Structure

```
mower-roboto/
├── backend/                    # FastAPI backend (existing)
│   └── app/
├── src/                        # React frontend (existing)
├── embedded/
│   ├── Arduino/
│   │   └── robot_mower/        # Arduino firmware
│   │       ├── robot_mower.ino
│   │       ├── motor_control.h
│   │       ├── protocol.h
│   │       └── safety.h
│   └── RPI/
│       └── mower-backend.service
├── ros2_ws/                    # NEW: ROS 2 workspace
│   └── src/
│       ├── mower_bringup/
│       │   ├── launch/
│       │   │   ├── mower.launch.py
│       │   │   ├── slam.launch.py
│       │   │   └── navigation.launch.py
│       │   └── config/
│       │       ├── nav2_params.yaml
│       │       ├── slam_params.yaml
│       │       └── ekf_params.yaml
│       ├── mower_description/
│       │   └── urdf/
│       │       └── mower.urdf.xacro
│       ├── mower_hardware/
│       │   ├── mower_hardware/
│       │   │   ├── serial_bridge.py
│       │   │   └── hardware_interface.py
│       │   └── launch/
│       ├── mower_msgs/
│       │   └── msg/
│       │       ├── MowerStatus.msg
│       │       └── SafetyStatus.msg
│       └── mower_navigation/
│           └── config/
└── _bmad-output/               # Planning artifacts
```

---

## Component Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         Web UI (React)                          │
│                    WebSocket + REST API                         │
└─────────────────────────┬───────────────────────────────────────┘
                          │
┌─────────────────────────┴───────────────────────────────────────┐
│                    FastAPI Backend                              │
│              Telemetry bridge, REST API                         │
└─────────────────────────┬───────────────────────────────────────┘
                          │ ROS 2 topics
┌─────────────────────────┴───────────────────────────────────────┐
│                      ROS 2 (RPi 4B)                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │ slam_toolbox│  │    Nav2     │  │   robot_localization    │  │
│  │   (SLAM)    │  │ (Planning)  │  │        (EKF)            │  │
│  └──────┬──────┘  └──────┬──────┘  └───────────┬─────────────┘  │
│         │                │                     │                │
│  ┌──────┴────────────────┴─────────────────────┴──────────┐     │
│  │                    /scan, /odom, /cmd_vel              │     │
│  └────────────────────────────┬───────────────────────────┘     │
│                               │                                 │
│  ┌────────────────┐    ┌──────┴──────┐    ┌────────────────┐   │
│  │  ldlidar_ros2  │    │serial_bridge│    │   imu_node     │   │
│  │   (LD19)       │    │  (custom)   │    │   (future)     │   │
│  └────────────────┘    └──────┬──────┘    └────────────────┘   │
└───────────────────────────────┼─────────────────────────────────┘
                                │ Serial 115200 baud
┌───────────────────────────────┴─────────────────────────────────┐
│                      Arduino Nano                               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │  Motor PID  │  │  Encoders   │  │    Safety/Watchdog      │  │
│  │  (50 Hz)    │  │  (ticks)    │  │    (200ms timeout)      │  │
│  └──────┬──────┘  └──────┬──────┘  └───────────┬─────────────┘  │
└─────────┼────────────────┼─────────────────────┼────────────────┘
          │                │                     │
    ┌─────┴─────┐    ┌─────┴─────┐         ┌─────┴─────┐
    │  Motors   │    │ Encoders  │         │  E-stop   │
    │ (H-bridge)│    │ (future)  │         │  (future) │
    └───────────┘    └───────────┘         └───────────┘
```

---

## Implementation Roadmap

### Phase 1: Foundation (MVP)
1. Arduino firmware (protocol, PID, watchdog)
2. ROS 2 serial bridge node
3. URDF + TF frames
4. Basic teleoperation via `/cmd_vel`

### Phase 2: Perception
5. LiDAR driver integration (LD19)
6. slam_toolbox configuration
7. Map building and saving

### Phase 3: Navigation
8. Nav2 configuration
9. Waypoint following
10. Obstacle avoidance

### Phase 4: Autonomy
11. Mission manager (row-by-row)
12. Headland maneuvers (demi-tours)
13. UI supervision

### Future
- IMU integration + EKF
- RTK-GPS
- Camera-based row detection

---

## Validation Checklist

- [x] Architecture supports all functional requirements
- [x] Safety requirements addressed (watchdog, failsafe)
- [x] Technology stack appropriate for constraints (RPi 4B)
- [x] Clear separation: real-time (Arduino) vs high-level (ROS 2)
- [x] Modular design for incremental development
- [x] Existing code (FastAPI, React) integrated
- [ ] Hardware not yet validated (encoders, IMU pending)
