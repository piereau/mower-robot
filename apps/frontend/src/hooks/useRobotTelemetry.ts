/**
 * Hook for subscribing to robot telemetry via WebSocket.
 * 
 * Manages connection lifecycle, reconnection, and exposes
 * current telemetry data with connection state.
 */

import { useEffect, useState, useCallback, useRef } from 'react';
import { createWSClient, WSClient } from '../services/wsClient';
import type { RobotTelemetry, ConnectionState } from '../types/telemetry';

interface UseRobotTelemetryResult {
  /** Current telemetry data (null if never received) */
  telemetry: RobotTelemetry | null;
  /** WebSocket connection state */
  connectionState: ConnectionState;
  /** Whether data is considered stale (> 10s since last update) */
  isStale: boolean;
  /** Time since last telemetry update in seconds */
  lastUpdateAge: number | null;
  /** Manually trigger reconnection */
  reconnect: () => void;
}

const STALE_THRESHOLD_MS = 10_000; // 10 seconds

export function useRobotTelemetry(): UseRobotTelemetryResult {
  const [telemetry, setTelemetry] = useState<RobotTelemetry | null>(null);
  const [connectionState, setConnectionState] = useState<ConnectionState>('connecting');
  const [lastUpdateTime, setLastUpdateTime] = useState<number | null>(null);
  const [now, setNow] = useState(Date.now());
  
  const wsClientRef = useRef<WSClient | null>(null);

  // Update "now" every second for stale calculation
  useEffect(() => {
    const interval = setInterval(() => {
      setNow(Date.now());
    }, 1000);
    
    return () => clearInterval(interval);
  }, []);

  // Handle incoming telemetry
  const handleMessage = useCallback((data: RobotTelemetry) => {
    setTelemetry(data);
    setLastUpdateTime(Date.now());
  }, []);

  // Handle connection state changes
  const handleConnectionChange = useCallback((connected: boolean) => {
    setConnectionState(connected ? 'connected' : 'disconnected');
  }, []);

  // Initialize WebSocket connection
  useEffect(() => {
    setConnectionState('connecting');
    
    const client = createWSClient(handleMessage, handleConnectionChange);
    wsClientRef.current = client;
    client.connect();

    return () => {
      client.disconnect();
      wsClientRef.current = null;
    };
  }, [handleMessage, handleConnectionChange]);

  // Manual reconnect function
  const reconnect = useCallback(() => {
    if (wsClientRef.current) {
      setConnectionState('reconnecting');
      wsClientRef.current.disconnect();
      wsClientRef.current.connect();
    }
  }, []);

  // Calculate staleness
  const lastUpdateAge = lastUpdateTime ? Math.floor((now - lastUpdateTime) / 1000) : null;
  const isStale = lastUpdateTime ? (now - lastUpdateTime) > STALE_THRESHOLD_MS : false;

  return {
    telemetry,
    connectionState,
    isStale,
    lastUpdateAge,
    reconnect,
  };
}

