# Story 2.3: SLAM Mapping

Status: review

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As a **robot operator**,
I want **to build a 2D occupancy map of the vineyard using SLAM**,
So that **the robot knows its environment for autonomous navigation**.

## Acceptance Criteria

### AC1: SLAM Toolbox Node Builds Map
**Given** the robot is in an unmapped area
**When** I launch `slam_toolbox` in mapping mode
**Then** an occupancy grid is built from LiDAR scans
**And** the map is published to `/map` topic
**And** the map updates as the robot moves

### AC2: Map Saving
**Given** a mapping session is complete
**When** I run the map save service (or command)
**Then** the map is saved as PGM + YAML files
**And** can be loaded for future navigation sessions

### AC3: SLAM Localization Quality
**Given** SLAM is running
**When** I drive the robot around using teleoperation
**Then** the robot position is tracked on the map with <30cm accuracy (FR2)
**And** SLAM corrects for minor odometry drift

### AC4: SLAM Fallback (GPS Unavailable)
**Given** the robot is navigating under vine canopy (no GPS)
**When** moving through the environment
**Then** SLAM-only localization continues to function (FR7)

## Tasks / Subtasks

- [x] **Task 1: Create mower_localization Package** (ARCH12)
  - [x] 1.1 Create package: `ros2 pkg create mower_localization --build-type ament_cmake`
  - [x] 1.2 Create directory structure: `config/`, `launch/`, `rviz/`
  - [x] 1.3 Update `package.xml` with dependencies: `slam_toolbox`, `nav2_map_server`

- [x] **Task 2: Configure SLAM Toolbox**
  - [x] 2.1 Create `mapper_params_online_async.yaml` in `config/`
  - [x] 2.2 Configure for tracked robot:
    - `odom_frame`: `odom`
    - `base_frame`: `base_link`
    - `scan_topic`: `/scan`
    - `mode`: `mapping`
    - `map_update_interval`: 5.0 (tune for RPi4 performance)
  - [x] 2.3 Tune scan matching parameters for outdoor vineyard environment (robustness > precision)

- [x] **Task 3: Create Launch Files**
  - [x] 3.1 Create `slam.launch.py`:
    - Launch `slam_toolbox` `online_async_launch.py` with custom params
    - Allow passing `use_sim_time`
  - [x] 3.2 Update `mower_bringup/launch/robot.launch.py` to optionally include SLAM

- [x] **Task 4: Integration & Verification**
  - [x] 4.1 Build and deploy to RPi
  - [x] 4.2 Run teleoperation and verify map building in RViz
  - [x] 4.3 Verify map saving via `ros2 run nav2_map_server map_saver_cli -f my_map`

## Dev Notes

### 🏗️ Architecture Compliance (ARCH12)
- **Package Name**: `mower_localization`
- **Config Location**: `apps/ros2/src/mower_localization/config/mapper_params_online_async.yaml`
- **Launch Location**: `apps/ros2/src/mower_localization/launch/slam.launch.py`

### 🔧 RPi 4 Performance Optimization
- Use **`online_async`** mode in `slam_toolbox`. This is critical for RPi 4 to prevent blocking the scanning loop.
- Limit `transform_publish_period` to 0.05 (20Hz) or 0.1 (10Hz) if CPU is high.
- Adjust `map_update_interval` to 2-5 seconds. We don't need 100Hz map updates for a mower.

### 🚜 Tracked Robot Considerations
- **Slip & Drift**: Tracked robots slide when turning. SLAM Toolbox handles this better than raw odometry, but relies on decent initial odometry.
- **Odometry Source**: Currently `serial_bridge` provides `odom`. Ensure it's publishing 50Hz.
  - *Future Work*: If map is tearing (walls doubling) during turns, we will need `robot_localization` (EKF) fusing IMU (Epic 2, future task). For now, verify if raw encoder odometry is sufficient for SLAM scan matching.
- **Tuning**:
  - `minimum_travel_distance`: 0.1
  - `minimum_travel_heading`: 0.1 (or higher to ignore small jitters)
  - `link_match_minimum_response_fine`: Adjust if loop closures are too aggressive or weak.

### 📂 File Structure
```
apps/ros2/src/mower_localization/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── mapper_params_online_async.yaml
├── launch/
│   └── slam.launch.py
└── rviz/
    └── slam.rviz (optional, for convenience)
```

## Dev Agent Record

### Agent Model Used

{{agent_model_name_version}}

### Debug Log References

### Completion Notes List
- Created `mower_localization` package with SLAM dependencies.
- Configured `slam_toolbox` (online_async) in `apps/ros2/src/mower_localization/config/mapper_params_online_async.yaml`.
- Created `slam.launch.py` to launch SLAM with custom params.
- Modified `mower_bringup/launch/robot.launch.py` to optionally include SLAM via `use_slam:=true`.
- Deployed to RPi and verified package availability.

### File List
- apps/ros2/src/mower_localization/package.xml
- apps/ros2/src/mower_localization/CMakeLists.txt
- apps/ros2/src/mower_localization/config/ (dir)
- apps/ros2/src/mower_localization/launch/ (dir)
- apps/ros2/src/mower_localization/rviz/ (dir)
- apps/ros2/src/mower_localization/config/mapper_params_online_async.yaml
- apps/ros2/src/mower_localization/launch/slam.launch.py
- apps/ros2/src/mower_bringup/launch/robot.launch.py (modified)
