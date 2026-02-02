import { createContext, useContext, useEffect, useRef, useState, useCallback, type ReactNode } from 'react';
import { createWSClient, WSClient } from '../services/wsClient';
import type { RobotTelemetry, ConnectionState } from '../types/telemetry';

interface RobotContextType {
    telemetry: RobotTelemetry | null;
    connectionState: ConnectionState;
    wsClient: WSClient | null;
    lastUpdateAge: number | null;
    isStale: boolean;
    reconnect: () => void;
}

const RobotContext = createContext<RobotContextType | null>(null);

const STALE_THRESHOLD_MS = 10_000;

export function RobotProvider({ children }: { children: ReactNode }) {
    const [telemetry, setTelemetry] = useState<RobotTelemetry | null>(null);
    const [connectionState, setConnectionState] = useState<ConnectionState>('disconnected');
    const [lastUpdateTime, setLastUpdateTime] = useState<number | null>(null);
    const [now, setNow] = useState(Date.now());

    const wsClientRef = useRef<WSClient | null>(null);

    // Update "now" every second for stale calculation
    useEffect(() => {
        const interval = setInterval(() => setNow(Date.now()), 1000);
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

    // Initialize Singleton WebSocket connection
    useEffect(() => {
        // Prevent double-init in Strict Mode
        if (wsClientRef.current) return;

        setConnectionState('connecting');
        const client = createWSClient(handleMessage, handleConnectionChange);
        wsClientRef.current = client;
        client.connect();

        return () => {
            // Clean up on unmount
            client.disconnect();
            wsClientRef.current = null;
        };
    }, [handleMessage, handleConnectionChange]);

    const reconnect = useCallback(() => {
        if (wsClientRef.current) {
            setConnectionState('reconnecting');
            wsClientRef.current.disconnect();
            // Small delay to allow cleanup
            setTimeout(() => {
                wsClientRef.current?.connect();
            }, 100);
        }
    }, []);

    const lastUpdateAge = lastUpdateTime ? Math.floor((now - lastUpdateTime) / 1000) : null;
    const isStale = lastUpdateTime ? (now - lastUpdateTime) > STALE_THRESHOLD_MS : false;

    return (
        <RobotContext.Provider value={{
            telemetry,
            connectionState,
            wsClient: wsClientRef.current,
            lastUpdateAge,
            isStale,
            reconnect
        }}>
            {children}
        </RobotContext.Provider>
    );
}

export function useRobot() {
    const context = useContext(RobotContext);
    if (!context) {
        throw new Error('useRobot must be used within a RobotProvider');
    }
    return context;
}
