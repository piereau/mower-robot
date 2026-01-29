/**
 * Connection status indicator - minimal version for new design.
 */

import { Wifi, WifiOff } from 'lucide-react';
import type { ConnectionState } from '../types/telemetry';

interface ConnectionStatusProps {
  state: ConnectionState;
}

export function ConnectionStatus({ state }: ConnectionStatusProps) {
  const isConnected = state === 'connected';
  const isConnecting = state === 'connecting' || state === 'reconnecting';
  
  if (isConnected) {
    return (
      <div className="flex items-center gap-1 text-emerald-600">
        <Wifi size={20} />
      </div>
    );
  }
  
  return (
    <div className={`flex items-center gap-1 ${isConnecting ? 'text-amber-500 animate-pulse' : 'text-red-500'}`}>
      <WifiOff size={20} />
    </div>
  );
}
