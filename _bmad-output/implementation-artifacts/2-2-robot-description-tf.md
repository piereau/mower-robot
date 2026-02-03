# Story 2.2: Robot Description & TF

Status: review

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As a **robot operator**,
I want **a proper URDF model with correct TF transforms**,
So that **sensor data is correctly positioned for navigation algorithms**.

## Acceptance Criteria

### AC1: URDF Published via Robot State Publisher
**Given** the mower_description package is built
**When** the robot_state_publisher is launched
**Then** TF transforms are published for `base_link`, `laser_frame`, `imu_link`
**And** transforms match physical robot dimensions

### AC2: TF Tree Complete and Verifiable
**Given** the TF tree is active
**When** I run `ros2 run tf2_tools view_frames`
**Then** the complete transform tree is visualized
**And** `base_link` is the root frame with proper child relationships

### AC3: URDF Visualizable in RViz
**Given** the URDF is loaded
**When** visualized in RViz
**Then** the robot model matches the physical tracked robot footprint
**And** sensor positions are correctly placed

## Tasks / Subtasks

- [x] **Task 1: Create mower_description Package** (AC: 1, 2, 3)
  - [x] 1.1 Create package: `ros2 pkg create mower_description --build-type ament_cmake`
  - [x] 1.2 Create directory structure: `urdf/`, `launch/`, `rviz/`
  - [x] 1.3 Update `package.xml` with dependencies: `urdf`, `xacro`, `robot_state_publisher`, `joint_state_publisher`
  - [x] 1.4 Configure `CMakeLists.txt` to install urdf, launch, and rviz directories

- [x] **Task 2: Create Robot URDF** (AC: 1, 2, 3)
  - [x] 2.1 Create `mower.urdf.xacro` with tracked robot chassis
  - [x] 2.2 Define `base_link` as root (center of robot footprint)
  - [x] 2.3 Add `laser_frame` link at position (0.15, 0, 0.10) from base_link
  - [x] 2.4 Add `imu_link` (placeholder position until IMU integrated)
  - [x] 2.5 Add `base_footprint` link at ground level (optional, for Nav2 compatibility)
  - [x] 2.6 Use robot physical dimensions from existing codebase

- [x] **Task 3: Create Launch File** (AC: 1)
  - [x] 3.1 Create `display.launch.py` for robot_state_publisher + RViz
  - [x] 3.2 Create `description.launch.py` for headless robot description
  - [x] 3.3 Generate URDF from xacro at launch time

- [x] **Task 4: Integration with Existing Nodes** (AC: 1, 2)
  - [x] 4.1 Update `mower_bringup/launch/lidar.launch.py` to remove static TF publisher
  - [x] 4.2 Create `mower_bringup/launch/robot.launch.py` to launch full robot stack
  - [x] 4.3 Include robot_state_publisher in main launch

- [x] **Task 5: Verification** (AC: 1, 2, 3)
  - [x] 5.1 Build packages: `colcon build --packages-select mower_description mower_bringup`
  - [x] 5.2 Launch and verify TF tree: `ros2 run tf2_tools view_frames`
  - [x] 5.3 Visualize in RViz (if display available) or validate with `ros2 topic echo /tf_static`

## Dev Notes

### 🔥 CRITICAL: Physical Robot Dimensions

These dimensions are **ALREADY DEFINED** in the codebase. **DO NOT INVENT NEW VALUES!**

| Parameter | Value | Source |
|-----------|-------|--------|
| **Wheel Separation** | 0.30 m | `embedded/arduino/src/motor_control.h:22` |
| **Wheel Diameter** | 0.10 m | `embedded/arduino/src/motor_control.h:25` |
| **LiDAR Position (x)** | 0.15 m | `mower_bringup/launch/lidar.launch.py:62` |
| **LiDAR Position (y)** | 0.00 m | `mower_bringup/launch/lidar.launch.py:62` |
| **LiDAR Position (z)** | 0.10 m | `mower_bringup/launch/lidar.launch.py:62` |

### TF Frame Tree (Target)

```
map
└── odom (published by serial_bridge or EKF)
    └── base_link (robot center)
        ├── laser_frame (LiDAR position)
        ├── imu_link (IMU position - placeholder)
        └── base_footprint (ground level - optional)
```

> **Note:** The `odom → base_link` transform is already published by `serial_bridge.py` (line 357-366). The URDF only needs to provide static transforms FROM `base_link` to sensor frames.

### Existing ROS 2 Infrastructure

| Component | Location | Status |
|-----------|----------|--------|
| `mower_bringup` | `apps/ros2/src/mower_bringup/` | ✅ Exists |
| `mower_hardware` | `apps/ros2/src/mower_hardware/` | ✅ Exists |
| `mower_msgs` | `apps/ros2/src/mower_msgs/` | ✅ Exists |
| `mower_teleop` | `apps/ros2/src/mower_teleop/` | ✅ Exists |
| `mower_description` | `apps/ros2/src/mower_description/` | ❌ **CREATE THIS** |
| `ldlidar_ros2` | `apps/ros2/src/ldlidar_ros2/` | ✅ Exists (external) |

### Architecture Compliance

From `architecture.md`:

| Requirement | Implementation |
|-------------|----------------|
| ARCH9: mower_description package | Create `apps/ros2/src/mower_description/` |
| URDF, TF | `urdf/mower.urdf.xacro` |
| Phase 1 Item 3 | URDF + TF frames setup |

### ROS 2 Humble URDF Best Practices (2024)

1. **Use xacro for modularity** - Define properties for dimensions, use macros for repeated elements
2. **Generate URDF in launch files** - Use `xacro` command substitution in launch
3. **robot_state_publisher** inputs:
   - `robot_description` parameter (URDF string)
   - Subscribes to `/joint_states` (not needed for fixed joints only)
4. **Outputs TF** to `/tf_static` (fixed) and `/tf` (movable joints)

### URDF Template (Tracked Robot)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="mower">
  <!-- Properties from existing codebase -->
  <xacro:property name="wheel_separation" value="0.30"/>
  <xacro:property name="wheel_diameter" value="0.10"/>
  <xacro:property name="chassis_length" value="0.40"/>  <!-- estimate -->
  <xacro:property name="chassis_width" value="0.35"/>   <!-- estimate -->
  <xacro:property name="chassis_height" value="0.15"/>  <!-- estimate -->
  
  <!-- LiDAR position from lidar.launch.py -->
  <xacro:property name="laser_x" value="0.15"/>
  <xacro:property name="laser_y" value="0.0"/>
  <xacro:property name="laser_z" value="0.10"/>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
      <material name="dark_grey">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
    </collision>
  </link>

  <!-- Laser Frame -->
  <link name="laser_frame"/>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame"/>
    <origin xyz="${laser_x} ${laser_y} ${laser_z}" rpy="0 0 0"/>
  </joint>

  <!-- IMU Link (placeholder) -->
  <link name="imu_link"/>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Base Footprint (ground plane) -->
  <link name="base_footprint"/>
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_footprint"/>
    <origin xyz="0 0 -${chassis_height/2}" rpy="0 0 0"/>
  </joint>
</robot>
```

### Launch File Template

```python
# mower_description/launch/description.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('mower_description')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'mower.urdf.xacro')
    
    robot_description = Command(['xacro ', urdf_path])
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
    ])
```

### Removing Static TF from lidar.launch.py

**IMPORTANT:** After URDF is working, the static transform publisher in `lidar.launch.py` (lines 60-68) should be **REMOVED** to avoid duplicate TF:

```python
# REMOVE THIS AFTER URDF IS WORKING:
base_to_laser_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    ...
)
```

### Testing Commands

```bash
# Build package
cd apps/ros2
colcon build --packages-select mower_description

# Source workspace
source install/setup.bash

# Launch robot_state_publisher
ros2 launch mower_description description.launch.py

# Verify TF tree (generates PDF)
ros2 run tf2_tools view_frames

# Check static transforms
ros2 topic echo /tf_static

# List all frames
ros2 run tf2_ros tf2_echo base_link laser_frame
```

### Previous Story Intelligence

**From Story 2-1 (LiDAR Integration):**
- ✅ Static TF publisher added as **workaround** because URDF didn't exist
- ✅ Position values (0.15, 0, 0.10) are correct and verified
- ⚠️ Need to **remove** static publisher after URDF is working
- ✅ `laser_frame` is the frame_id used by LiDAR driver

### Git Intelligence

Recent commits show:
- `39731fe` feat: 2.1 lidar integration - Added ldlidar_ros2, lidar.launch.py with static TF
- Files modified in `mower_bringup/` package

### Project Structure Notes

**New files to create:**
```
apps/ros2/src/mower_description/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── description.launch.py  # headless
│   └── display.launch.py      # with RViz (optional)
├── urdf/
│   └── mower.urdf.xacro
└── rviz/
    └── mower.rviz             # optional
```

**Files to modify:**
- `mower_bringup/launch/lidar.launch.py` - Remove static TF publisher

### References

- [Source: architecture.md#Project Structure] - Workspace structure
- [Source: architecture.md#Implementation Roadmap Phase 1] - URDF + TF frames
- [Source: epics.md#Story 2.2] - Acceptance criteria
- [Source: motor_control.h] - Robot physical dimensions
- [Source: lidar.launch.py] - LiDAR position
- [Source: serial_bridge.py] - odom→base_link TF publisher

## Dev Agent Record

### Agent Model Used

{{agent_model_name_version}}

### Debug Log References

### Completion Notes List
- Created `mower_description` package with complete URDF for the tracked mower
- Implemented `mower.urdf.xacro` defining `base_link`, `laser_frame`, `imu_link`, and `base_footprint`
- Created launch files for headless (`description.launch.py`) and visualization (`display.launch.py`) modes
- Removed static TF publisher from `lidar.launch.py` and replaced with proper robot state publisher integration
- Verified TF tree on Raspberry Pi: all segments (`base_link`, `laser_frame`, `imu_link`, `base_footprint`) correctly published
- Created new `mower_bringup/launch/robot.launch.py` for full system startup

### File List
- apps/ros2/src/mower_description/package.xml
- apps/ros2/src/mower_description/CMakeLists.txt
- apps/ros2/src/mower_description/urdf/mower.urdf.xacro
- apps/ros2/src/mower_description/launch/description.launch.py
- apps/ros2/src/mower_description/launch/display.launch.py
- apps/ros2/src/mower_description/rviz/mower.rviz
- apps/ros2/src/mower_bringup/launch/lidar.launch.py
- apps/ros2/src/mower_bringup/launch/robot.launch.py

### Change Log
- 2026-02-03: Created mower_description package and URDF (Pierrot/Amelia)
- 2026-02-03: Integrated URDF with robot_state_publisher and removed static TF (Pierrot/Amelia)
- 2026-02-03: Refined URDF with physical measurements (x=0.07, z=0.17) provided by user (Pierrot/Amelia)
