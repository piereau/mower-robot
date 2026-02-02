/**
 * Hook for sending drive commands over WebSocket.
 * 
 * Sends normalized joystick values (x, y) to the backend:
 * - x: left/right (-1 to 1), positive = right
 * - y: forward/back (-1 to 1), positive = forward
 * 
 * Rate limited to 10Hz (100ms) to save bandwidth.
 * The ROS 2 bridge handles conversion to Twist messages.
 */

import { useCallback } from 'react';
import { useRobot } from '../contexts/RobotContext';
import { useThrottledCallback, THROTTLE_MS } from './useThrottledCallback';

const DEADZONE = 0.08;

const applyDeadzone = (value: number) =>
  Math.abs(value) < DEADZONE ? 0 : value;

export function useRobotControl() {
  const { wsClient, connectionState } = useRobot();
  const connected = connectionState === 'connected';

  // Send joystick position to backend
  const sendPosition = useCallback((x: number, y: number) => {
    if (!wsClient || !connected) return;

    // Apply deadzone
    const dx = applyDeadzone(x);
    const dy = applyDeadzone(y);

    wsClient.sendJSON({
      type: 'control',
      x: dx,
      y: dy,
    });
  }, [wsClient, connected]);

  // Throttled version for continuous movement (10Hz)
  const sendJoystick = useThrottledCallback(sendPosition, THROTTLE_MS);

  // Immediate stop command (not throttled)
  const stop = useCallback(() => {
    if (!wsClient || !connected) return;

    wsClient.sendJSON({
      type: 'control',
      x: 0,
      y: 0,
    });
  }, [wsClient, connected]);

  // E-stop commands
  const sendEstop = useCallback(() => {
    if (!wsClient || !connected) return;
    wsClient.sendJSON({ type: 'estop' });
  }, [wsClient, connected]);

  const releaseEstop = useCallback(() => {
    if (!wsClient || !connected) return;
    wsClient.sendJSON({ type: 'release_estop' });
  }, [wsClient, connected]);

  return {
    connected,
    sendJoystick,
    stop,
    sendEstop,
    releaseEstop,
  };
}
