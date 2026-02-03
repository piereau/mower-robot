/**
 * Hook for subscribing to robot telemetry via WebSocket.
 * 
 * Manages connection lifecycle, reconnection, and exposes
 * current telemetry data with connection state.
 */

import { useRobot } from '../contexts/RobotContext';

export function useRobotTelemetry() {
  const {
    telemetry,
    mapData,
    scanData,
    poseData,
    connectionState,
    isStale,
    lastUpdateAge,
    reconnect
  } = useRobot();

  return {
    telemetry,
    mapData,
    scanData,
    poseData,
    connectionState,
    isStale,
    lastUpdateAge,
    reconnect,
  };
}
