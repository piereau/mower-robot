# Story 2.4: Map Visualization on Dashboard

Status: done

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As a **robot operator**,
I want **to see the map and robot position on the dashboard**,
So that **I can monitor where the robot is in the vineyard**.

## Acceptance Criteria

### AC1: Map Visualization (FR21)
**Given** a map has been created via SLAM/Nav2 and published to `/map`
**When** the dashboard loads
**Then** the occupancy grid map is displayed efficiently on the UI
**And** the map visualization renders occupied, free, and unknown spaces correctly
**And** the map can be panned and zoomed by the user

### AC2: Real-Time Robot Tracking (FR21)
**Given** the robot is localized on the map (TF `map -> base_link` exists)
**When** the robot moves
**Then** an icon representing the robot updates its position and orientation on the map
**And** the visual update latency is <500ms (NFR2)
**And** the robot icon scale matches real-world dimensions relative to the map

### AC3: LiDAR Overlay (FR22)
**Given** the LiDAR is active and publishing to `/scan`
**When** I enable "Show LiDAR" toggle on the dashboard
**Then** current laser scan points are overlaid on the map in real-time
**And** points fade or update at >5Hz to show dynamic obstacles

### AC4: Performance & Bandwidth
**Given** the dashboard is connected via WiFi
**When** map and scan data are streaming
**Then** the browser tab memory usage does not constantly increase (no leaks)
**And** handling large map updates does not freeze the UI thread

## Tasks / Subtasks

- [ ] **Task 1: Backend ROS 2 Bridging**
  - [ ] 1.1 Update `mower_hardware/serial_bridge.py` or create new `map_bridge` node to subscribe to `/map` (OccupancyGrid) and `/scan`.
  - [ ] 1.2 Implement efficient WebSocket serialization for Map (only send on change/latch) and Scan (decimate if needed).
  - [ ] 1.3 Ensure TF transform (`map` -> `base_link`) is broadcast to frontend or pre-calculated as Pose.

- [ ] **Task 2: Frontend Map Component**
  - [ ] 2.1 Create `<MapCanvas />` component using HTML5 Canvas or `react-konva`.
  - [ ] 2.2 Implement occupancy grid rendering (convert 0-100 probability to grayscale/color).
  - [ ] 2.3 Implement coordinate transform helper (ROS World Coords <-> Canvas Pixel Coords).
  - [ ] 2.4 Add Pan/Zoom interaction (mouse/touch events).

- [ ] **Task 3: Overlays & Integration**
  - [ ] 3.1 Draw Robot Icon at current Pose (rotate and scale correctly).
  - [ ] 3.2 Render LiDAR points relative to Robot Pose (if scan data is in `laser_frame`, transform to `map` frame using TF or locally in JS if limits known).
  - [ ] 3.3 Add "Layer Control" UI to toggle Map/LiDAR visibility.

- [ ] **Task 4: Verification**
  - [ ] 4.1 Verify map matches RViz output.
  - [ ] 4.2 Measure latency of robot movement on screen vs reality.

## Dev Notes

### 🏗️ Architecture Compliance
- **Pattern**: Stick to the "Telemetry in-memory + WebSocket" decision (ARCH158). Do NOT introduce `rosbridge_suite` or `roslibjs` unless explicitly decided to pivot architecture (current arch uses custom FastAPI bridge).
- **Data Flow**: `ROS 2 Node` -> `FastAPI (via Shared Memory/ZMQ/Internal Interface?)` -> `WebSocket` -> `React Client`.
  - *Correction*: The current Architecture implies FastAPI talks to ROS via a custom bridge or just Python ROS 2 bindings directly in the API service? **ARCH145** says "ROS 2 serial bridge node" is a custom Python script. **CTX1** says "FastAPI backend".
  - *Recommendation*: The best pattern here is for the FastAPI backend to run a `rclpy` node (or separate thread) that subscribes to `/map` and `/scan`, converting them to JSON/Binary for WebSocket.

### 🚀 Performance Optimization
- **Bandwidth**: OccupancyGrids are large arrays.
  - *Technique*: Compress before sending (RLE or simple zlib if binary WS supported). Or just send raw if map is small (<1MB).
  - *Scan*: Downsample LiDAR points if UI is lagging (e.g., skip every 2nd point).
- **Rendering**: Use `requestAnimationFrame` for the canvas loop. Don't rely on React state updates for every single frame of animation if high frequency (use Refs for canvas context).

### 📐 Coordinate Systems
- **ROS**: X=Forward, Y=Left, Z=Up. Origin (0,0) is map center or specified origin. Resolution = meters/pixel.
- **Canvas**: X=Right, Y=Down. Origin (0,0) is top-left.
- **Conversion**:
  - `pixel_x = (ros_x - map_origin_x) / resolution`
  - `pixel_y = map_height_pixels - ((ros_y - map_origin_y) / resolution)` (Flip Y axis)

## Dev Agent Record

### Agent Model Used

{{agent_model_name_version}}

### Debug Log References

### Completion Notes List

### File List
