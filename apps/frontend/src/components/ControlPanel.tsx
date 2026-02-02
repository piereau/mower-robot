import { useState, useEffect } from 'react';
import { Joystick } from './Joystick';
import { useRobotControl } from '../hooks/useRobotControl';
import { useRobotTelemetry } from '../hooks/useRobotTelemetry';
import { AlertTriangle, Wifi, WifiOff, Activity } from 'lucide-react';

type ControlPanelProps = {
  cameraUrl: string;
};

export function ControlPanel({ cameraUrl }: ControlPanelProps) {
  const { sendJoystick, stop, sendEstop, releaseEstop } = useRobotControl();
  const { telemetry, connectionState } = useRobotTelemetry();
  const [cameraError, setCameraError] = useState(false);
  const [estopActive, setEstopActive] = useState(false);
  const [currentSpeed, setCurrentSpeed] = useState({ linear: 0, angular: 0 });

  // Extract bridge status from telemetry
  const telemetryExtended = telemetry as unknown as Record<string, unknown> | null;
  const bridgeConnected = telemetryExtended?.bridge_connected === true;
  const bridgeStatus = telemetryExtended?.bridge_status as
    | { linear_vel?: number; angular_vel?: number }
    | undefined;

  // Update speed from telemetry
  useEffect(() => {
    if (bridgeStatus) {
      setCurrentSpeed({
        linear: bridgeStatus.linear_vel ?? 0,
        angular: bridgeStatus.angular_vel ?? 0,
      });
    }
  }, [bridgeStatus]);

  const handleEstop = () => {
    if (estopActive) {
      releaseEstop();
      setEstopActive(false);
    } else {
      sendEstop();
      setEstopActive(true);
    }
  };

  const isConnected = connectionState === 'connected';
  const canControl = isConnected && bridgeConnected && !estopActive;

  return (
    <div className="space-y-4">
      {/* Camera Feed */}
      <div className="rounded-2xl bg-black/10 p-2">
        {cameraError ? (
          <div className="flex h-44 items-center justify-center rounded-xl bg-black/10 text-sm text-black/60">
            Flux caméra indisponible
          </div>
        ) : (
          <img
            src={cameraUrl}
            alt="Flux caméra"
            className="h-44 w-full rounded-xl object-cover"
            onError={() => setCameraError(true)}
          />
        )}
      </div>

      {/* Connection Status Bar */}
      <div className="flex items-center justify-between px-2 text-sm">
        <div className="flex items-center gap-2">
          {isConnected ? (
            <Wifi size={16} className="text-green-600" />
          ) : (
            <WifiOff size={16} className="text-red-500" />
          )}
          <span className={isConnected ? 'text-green-700' : 'text-red-600'}>
            {isConnected ? 'WebSocket OK' : 'Déconnecté'}
          </span>
        </div>
        <div className="flex items-center gap-2">
          <Activity size={16} className={bridgeConnected ? 'text-green-600' : 'text-orange-500'} />
          <span className={bridgeConnected ? 'text-green-700' : 'text-orange-600'}>
            {bridgeConnected ? 'ROS OK' : 'ROS Offline'}
          </span>
        </div>
      </div>

      {/* Speed Display */}
      <div className="grid grid-cols-2 gap-3">
        <div className="rounded-xl bg-white/80 p-3 text-center">
          <div className="text-xs text-black/60 uppercase tracking-wider">Vitesse</div>
          <div className="text-2xl font-bold text-black">
            {Math.abs(currentSpeed.linear).toFixed(2)}
          </div>
          <div className="text-xs text-black/50">m/s</div>
        </div>
        <div className="rounded-xl bg-white/80 p-3 text-center">
          <div className="text-xs text-black/60 uppercase tracking-wider">Rotation</div>
          <div className="text-2xl font-bold text-black">
            {Math.abs(currentSpeed.angular).toFixed(2)}
          </div>
          <div className="text-xs text-black/50">rad/s</div>
        </div>
      </div>

      {/* E-Stop Button */}
      <button
        onClick={handleEstop}
        className={`w-full py-3 rounded-xl font-bold text-white flex items-center justify-center gap-2 transition-all ${estopActive
          ? 'bg-orange-500 hover:bg-orange-600'
          : 'bg-red-600 hover:bg-red-700'
          }`}
      >
        <AlertTriangle size={20} />
        {estopActive ? 'RELÂCHER E-STOP' : 'ARRÊT D\'URGENCE'}
      </button>

      {/* Joystick */}
      <div className="space-y-2">
        <h2 className="text-xl font-medium tracking-tight text-black text-center">
          Joystick
        </h2>
        <Joystick onMove={sendJoystick} onEnd={stop} disabled={!canControl} />
        <p className="text-xs text-black/60 text-center">
          Haut = avance, bas = recule, gauche/droite = tourne.
        </p>
        {!isConnected && (
          <p className="text-xs text-red-600 text-center">
            ⚠️ WebSocket non connectée
          </p>
        )}
        {isConnected && !bridgeConnected && (
          <p className="text-xs text-orange-600 text-center">
            ⚠️ Bridge ROS 2 indisponible
          </p>
        )}
        {estopActive && (
          <p className="text-xs text-red-600 text-center font-bold">
            🛑 ARRÊT D'URGENCE ACTIF
          </p>
        )}
      </div>
    </div>
  );
}
