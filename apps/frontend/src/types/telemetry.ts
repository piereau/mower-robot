/**
 * Robot telemetry types matching backend API contract.
 */

export type RobotState = 'idle' | 'mowing' | 'error';

export interface RobotTelemetry {
  state: RobotState;
  battery: number;
  timestamp: string;
}

/**
 * Connection state for WebSocket
 */
export type ConnectionState = 'connecting' | 'connected' | 'disconnected' | 'reconnecting';

/**
 * Display status derived from telemetry data
 */
export type DisplayStatus = 'ready' | 'charging' | 'mowing';

/**
 * Derive display status from telemetry
 * - ready: battery >= 80% (Prêt à tondre)
 * - charging: at parking and battery < 80% (En charge)
 * - mowing: currently mowing (En cours de tonte)
 */
export function deriveDisplayStatus(telemetry: RobotTelemetry): DisplayStatus {
  // If actively mowing
  if (telemetry.state === 'mowing') {
    return 'mowing';
  }

  // If battery is full enough, ready to mow
  if (telemetry.battery >= 80) {
    return 'ready';
  }

  // Otherwise, charging (assuming at parking when idle with low battery)
  return 'charging';
}

/**
 * ROS 2 Map Message
 */
export interface MapMessage {
  type: 'map';
  info: {
    resolution: number;
    width: number;
    height: number;
    origin: {
      position: { x: number; y: number };
      orientation: { z: number; w: number };
    };
  };
  data: number[];
}

/**
 * ROS 2 Laser Scan Message
 */
export interface ScanMessage {
  type: 'scan';
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  range_min: number;
  range_max: number;
  ranges: number[];
}

/**
 * Robot Pose Message
 */
export interface PoseMessage {
  type: 'pose';
  x: number;
  y: number;
  yaw: number;
}
