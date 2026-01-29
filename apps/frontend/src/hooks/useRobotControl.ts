/**
 * Hook for sending drive commands over WebSocket.
 */

import { useCallback, useEffect, useRef, useState } from 'react';
import { createWSClient, WSClient } from '../services/wsClient';
import type { DriveCommand } from '../types/control';

const SEND_INTERVAL_MS = 50;
const DEADZONE = 0.08;
const TURN_IN_PLACE_THRESHOLD = 0.2;

const clamp = (value: number, min: number, max: number) =>
  Math.min(max, Math.max(min, value));

const applyDeadzone = (value: number) =>
  Math.abs(value) < DEADZONE ? 0 : value;

const mapJoystickToDrive = (x: number, y: number): DriveCommand => {
  const dx = applyDeadzone(x);
  const dy = applyDeadzone(y);

  if (Math.abs(dy) < TURN_IN_PLACE_THRESHOLD && Math.abs(dx) > 0) {
    return {
      left: dx > 0 ? Math.abs(dx) : 0,
      right: dx < 0 ? Math.abs(dx) : 0,
    };
  }

  return {
    left: clamp(dy + dx, -1, 1),
    right: clamp(dy - dx, -1, 1),
  };
};

export function useRobotControl() {
  const [connected, setConnected] = useState(false);
  const wsClientRef = useRef<WSClient | null>(null);
  const lastSendRef = useRef(0);
  const pendingRef = useRef<DriveCommand | null>(null);
  const sendTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  useEffect(() => {
    const client = createWSClient(() => {}, setConnected);
    wsClientRef.current = client;
    client.connect();

    return () => {
      client.disconnect();
      wsClientRef.current = null;
      if (sendTimeoutRef.current) {
        clearTimeout(sendTimeoutRef.current);
        sendTimeoutRef.current = null;
      }
    };
  }, []);

  const flushSend = useCallback(() => {
    if (!pendingRef.current || !wsClientRef.current) return;

    wsClientRef.current.sendJSON({
      type: 'control',
      ...pendingRef.current,
    });
    pendingRef.current = null;
    lastSendRef.current = Date.now();
  }, []);

  const scheduleSend = useCallback(() => {
    if (sendTimeoutRef.current) return;

    const elapsed = Date.now() - lastSendRef.current;
    const delay = Math.max(0, SEND_INTERVAL_MS - elapsed);

    sendTimeoutRef.current = setTimeout(() => {
      sendTimeoutRef.current = null;
      flushSend();
    }, delay);
  }, [flushSend]);

  const sendJoystick = useCallback(
    (x: number, y: number) => {
      pendingRef.current = mapJoystickToDrive(x, y);

      if (Date.now() - lastSendRef.current >= SEND_INTERVAL_MS) {
        flushSend();
      } else {
        scheduleSend();
      }
    },
    [flushSend, scheduleSend]
  );

  const stop = useCallback(() => {
    pendingRef.current = { left: 0, right: 0 };
    flushSend();
  }, [flushSend]);

  return {
    connected,
    sendJoystick,
    stop,
  };
}
