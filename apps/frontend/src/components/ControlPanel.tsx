import { useState } from 'react';
import { Joystick } from './Joystick';
import { useRobotControl } from '../hooks/useRobotControl';
import { useRobotTelemetry } from '../hooks/useRobotTelemetry';
import { MapCanvas } from './MapCanvas';
import { AlertTriangle, Wifi, WifiOff, Activity } from 'lucide-react';

type ControlPanelProps = {
  cameraUrl: string;
};

export function ControlPanel({ cameraUrl }: ControlPanelProps) {
  const { sendJoystick, stop, sendEstop, releaseEstop } = useRobotControl();
  const {
    telemetry,
    connectionState,
    mapData,
    scanData,
    poseData
  } = useRobotTelemetry();

  const [cameraError, setCameraError] = useState(false);
  const [estopActive, setEstopActive] = useState(false);

  // Extract bridge status from telemetry
  const telemetryExtended = telemetry as unknown as Record<string, unknown> | null;
  const bridgeConnected = telemetryExtended?.bridge_connected === true;

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

      {/* Map Visualization */}
      <div className="rounded-2xl bg-white p-2 shadow-sm">
        <div className="text-xs text-black/60 uppercase tracking-wider mb-2 px-1">Carte & Navigation</div>
        <MapCanvas
          mapData={mapData}
          scanData={scanData}
          robotPose={poseData}
          className="w-full aspect-square bg-gray-100 rounded-xl"
          showLidar={true}
        />
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
      <div className="flex flex-col items-center justify-center min-h-[40vh] space-y-4">
        <Joystick onMove={sendJoystick} onEnd={stop} disabled={!canControl} />

        {/* Alerts */}
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
