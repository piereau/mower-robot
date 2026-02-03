# Story 1.4: Real-time Telemetry Display

Status: done

<!-- Note: This story was already implemented in previous work. Validated 2026-02-03. -->

## Story

As a **robot operator**,
I want **to see real-time robot state, battery level, and connection status on the dashboard**,
So that **I can monitor the robot's health while operating**.

## Acceptance Criteria

### AC1: Robot State Display
**Given** the dashboard is connected
**When** telemetry is received from the robot
**Then** robot state is displayed (idle, moving, error)
**And** telemetry updates within 500ms of state change (NFR2)

### AC2: Battery Level Display with Warning
**Given** the robot is operational
**When** battery voltage is read by Arduino
**Then** battery percentage is calculated and displayed on dashboard
**And** low-battery warning appears when below 20%

### AC3: Connection Lost Indicator
**Given** the dashboard WebSocket connection
**When** connection is lost
**Then** "Disconnected" indicator is shown prominently
**And** automatic reconnection attempts occur with exponential backoff
**And** "Reconnecting..." status is displayed

### AC4: Connection Restored Resume
**Given** the dashboard is displaying telemetry
**When** connection is restored after interruption
**Then** "Connected" indicator is shown
**And** telemetry display resumes immediately

## Tasks / Subtasks

- [ ] **Task 1: Add Robot State to Backend Telemetry** (AC: 1)
  - [ ] 1.1 Add `robot_state` field to MOWER_MSGS/MowerStatus (or use existing field)
  - [ ] 1.2 Update `ros_bridge/client.py` to parse state from bridge telemetry
  - [ ] 1.3 Include state in WebSocket broadcast payload (`robot.py`)
  - [ ] 1.4 Define state enum: idle, moving, error, estop, tilt

- [ ] **Task 2: Create TelemetryPanel Component** (AC: 1, 2)
  - [ ] 2.1 Create `components/TelemetryPanel.tsx` - dedicated status display widget
  - [ ] 2.2 Display robot state with color-coded icon (green=idle, blue=moving, red=error)
  - [ ] 2.3 Convert battery_mv to percentage using voltage thresholds
  - [ ] 2.4 Integrate existing `BatteryIndicator.tsx` with real telemetry data
  - [ ] 2.5 Add low battery warning animation when < 20%

- [ ] **Task 3: Enhance Connection Status Logic** (AC: 3, 4)
  - [ ] 3.1 Update `useRobotTelemetry.ts` to expose detailed connection state transitions
  - [ ] 3.2 Add exponential backoff tracking (current retry count, next retry in X seconds)
  - [ ] 3.3 Create "Reconnecting..." animation in `ConnectionStatus.tsx`
  - [ ] 3.4 Ensure instant state update on reconnect (flash green confirmation)

- [ ] **Task 4: Integrate Telemetry into Dashboard** (AC: 1, 2, 3, 4)
  - [ ] 4.1 Add `TelemetryPanel` to main Dashboard view (visible on all tabs)
  - [ ] 4.2 Add `TelemetryPanel` to `ControlPanel.tsx` for operation monitoring
  - [ ] 4.3 Ensure prominent "Disconnected" overlay when connection lost
  - [ ] 4.4 Test NFR2: verify <500ms update latency using browser DevTools

- [ ] **Task 5: Integration Testing** (AC: 1, 2, 3, 4)
  - [ ] 5.1 Test state transitions (start/stop robot, verify state updates)
  - [ ] 5.2 Test battery display with simulated voltages
  - [ ] 5.3 Test disconnect/reconnect cycle (kill backend, restart, verify UI)
  - [ ] 5.4 Measure latency: change robot state, verify <500ms dashboard update

## Dev Notes

### Previous Story Intelligence (Story 1.3)

| Aspect | What Was Done | Impact on This Story |
|--------|---------------|---------------------|
| Bridge status | `bridge_status` dict includes `battery_mv`, `linear_vel`, `angular_vel`, `estop`, `watchdog` | Use for telemetry |
| WebSocket broadcast | `robot.py:broadcast_telemetry()` at configured interval | Already broadcasts data |
| Frontend hooks | `useRobotTelemetry()` returns `telemetry`, `connectionState` | Extend with parsed data |
| Connection status | Basic `ConnectionStatus.tsx` shows Wifi icon | Enhance with reconnect info |

### Existing Infrastructure (DO NOT RECREATE)

| Component | Location | How to Use |
|-----------|----------|------------|
| `BatteryIndicator.tsx` | `src/components/` | Pass percentage from telemetry |
| `ConnectionStatus.tsx` | `src/components/` | Extend for reconnect states |
| `useRobotTelemetry.ts` | `src/hooks/` | Hook returns telemetry + state |
| `broadcast_telemetry()` | `backend/app/ws/robot.py` | Already sends `bridge_status` |
| `RosBridgeClient` | `backend/app/ros_bridge/client.py` | Provides `status` property |

### Battery Voltage → Percentage Conversion

The Arduino reports `battery_mv` (motor battery voltage). Use these thresholds:

```typescript
// For 3S LiPo (nominally 11.1V)
const BATTERY_FULL_MV = 12600;    // 4.2V × 3 cells
const BATTERY_EMPTY_MV = 9900;    // 3.3V × 3 cells (safe cutoff)
const BATTERY_LOW_THRESHOLD = 20; // %

function batteryMvToPercent(mv: number): number {
  if (mv <= BATTERY_EMPTY_MV) return 0;
  if (mv >= BATTERY_FULL_MV) return 100;
  return Math.round(((mv - BATTERY_EMPTY_MV) / (BATTERY_FULL_MV - BATTERY_EMPTY_MV)) * 100);
}
```

### Robot State Enum

```typescript
type RobotState = 'idle' | 'moving' | 'error' | 'estop' | 'tilt';

// Derived from telemetry
function deriveRobotState(telemetry: TelemetryData): RobotState {
  if (telemetry.estop) return 'estop';
  if (telemetry.tilt_detected) return 'tilt';
  if (telemetry.error_flags > 0) return 'error';
  if (Math.abs(telemetry.linear_vel) > 0.01 || Math.abs(telemetry.angular_vel) > 0.01) return 'moving';
  return 'idle';
}
```

### WebSocket Telemetry Payload (Current Structure)

```json
{
  "timestamp": "2026-02-03T12:00:00Z",
  "bridge_connected": true,
  "bridge_status": {
    "serial_connected": true,
    "watchdog": false,
    "estop": false,
    "battery_mv": 12400,
    "linear_vel": 0.0,
    "angular_vel": 0.0
  }
}
```

### Architecture Compliance

| Requirement | Implementation |
|-------------|----------------|
| Latency <500ms (NFR2) | WebSocket push, no polling |
| Mobile-first UI (NFR20) | Use responsive Tailwind classes |
| High-contrast (NFR21) | Use bold colors for status indicators |
| User-friendly errors (NFR22) | Show descriptive status, not codes |

### Project Structure After This Story

```
apps/frontend/src/
├── components/
│   ├── TelemetryPanel.tsx       # NEW - Main telemetry display
│   ├── RobotStateIndicator.tsx  # NEW - State icon with color
│   ├── BatteryIndicator.tsx     # Modified - Use real data
│   └── ConnectionStatus.tsx     # Modified - Add reconnect UI
├── hooks/
│   └── useRobotTelemetry.ts     # Modified - Parse battery, state
└── types/
    └── telemetry.ts             # Modified - Add RobotState type
```

### Testing Strategy

**Manual Testing:**
```bash
# Terminal 1: Watch WebSocket frames
# Open browser to dashboard, open DevTools Network > WS tab

# Terminal 2: Simulate battery change (if Arduino connected)
# Otherwise, modify simulator in backend

# Test Disconnect: Stop FastAPI backend, verify "Reconnecting..." shows
# Restart FastAPI, verify "Connected" shows immediately
```

**Acceptance Criteria Verification:**

| AC | Test | Expected Result |
|----|------|-----------------|
| AC1 | Move robot, observe dashboard | State changes idle → moving → idle |
| AC2 | Check battery display | Shows percentage, warning at <20% |
| AC3 | Kill backend, check dashboard | "Disconnected" + "Reconnecting..." |
| AC4 | Restart backend | Immediate green "Connected" flash |

### References

- [Source: _bmad-output/planning-artifacts/epics.md#Story 1.4] - Original acceptance criteria
- [Source: _bmad-output/planning-artifacts/architecture.md] - System architecture
- [Source: _bmad-output/implementation-artifacts/1-3-teleoperation-via-dashboard.md] - Previous story patterns
- [Source: apps/backend/app/ws/robot.py] - WebSocket broadcast implementation
- [Source: apps/backend/app/ros_bridge/client.py] - Bridge status data source
- [Source: apps/frontend/src/components/BatteryIndicator.tsx] - Existing battery component

## Dev Agent Record

### Agent Model Used

{{agent_model_name_version}}

### Debug Log References

### Completion Notes List

### File List
