/**
 * Main dashboard page - Figma design implementation.
 */

import { useState } from 'react';
import { useRobotTelemetry } from '../hooks/useRobotTelemetry';
import { RobotImage } from '../components/RobotImage';
import { StatusIndicator } from '../components/StatusIndicator';
import { BatteryIndicator } from '../components/BatteryIndicator';
import { LocationIndicator } from '../components/LocationIndicator';
import { AutonomyIndicator } from '../components/AutonomyIndicator';
import { ScheduleCard } from '../components/ScheduleCard';
import { ControlPanel } from '../components/ControlPanel';
import { Loader2 } from 'lucide-react';
import { deriveDisplayStatus } from '../types/telemetry';

type Schedule = {
  zone: string;
  time: string;
};

interface DashboardProps {
  schedules: Schedule[];
  onZoneClick?: (zoneName: string) => void;
  onScheduleClick?: () => void;
}

export function Dashboard({ schedules, onZoneClick, onScheduleClick }: DashboardProps) {
  const { telemetry, connectionState } = useRobotTelemetry();
  const [activeTab, setActiveTab] = useState<'monitor' | 'control'>('monitor');
  const cameraUrl = (() => {
    const direct = import.meta.env.VITE_CAMERA_URL as string | undefined;
    if (direct) {
      return direct;
    }

    const wsUrl = import.meta.env.VITE_WS_URL as string | undefined;
    if (wsUrl) {
      try {
        const parsed = new URL(wsUrl);
        const protocol = parsed.protocol === 'wss:' ? 'https:' : 'http:';
        return `${protocol}//${parsed.host}/camera/stream`;
      } catch {
        // Fall back to default.
      }
    }

    return 'http://localhost:8000/camera/stream';
  })();

  const showOfflineOverlay = connectionState === 'disconnected';

  // Derive display status from telemetry
  const displayStatus = telemetry ? deriveDisplayStatus(telemetry) : null;

  // Derive location based on status
  const location = displayStatus === 'mowing' ? 'Parcelle A1' : 'Parking';

  return (
    <div className="mx-auto w-full max-w-md min-h-screen bg-[#d9e7f3] px-5 py-8 relative pb-32">

      {activeTab === 'monitor' && (
        <>
          {/* Robot image section */}
          <div className="pt-2">
            <RobotImage />
          </div>

          {/* Status section */}
          <div className="space-y-3 mb-12">
            {telemetry && displayStatus ? (
              <>
                {/* Status + Battery row */}
                <div className="flex items-center justify-between">
                  <StatusIndicator status={displayStatus} />
                  <BatteryIndicator level={telemetry.battery} />
                </div>

                {/* Location + Autonomy row */}
                <div className="flex items-center justify-between">
                  <LocationIndicator location={location} />
                  <AutonomyIndicator batteryLevel={telemetry.battery} />
                </div>
              </>
            ) : (
              /* Loading state */
              <div className="flex items-center gap-2 text-black/50">
                <Loader2 size={32} className="animate-spin" />
                <span className="text-[32px] font-medium tracking-tight">
                  Connexion...
                </span>
              </div>
            )}
          </div>

          {/* Schedule section */}
          <div className="space-y-4">
            <h2 className="text-[32px] font-medium tracking-tight text-black">
              Prochaines tontes
            </h2>

            <div className="flex gap-3 overflow-x-auto pb-2 -mx-5 px-5">
              {/* Add schedule button */}
              <button
                onClick={onScheduleClick}
                className="bg-white rounded-xl p-3 flex flex-col items-center justify-center min-w-[120px] min-h-[100px] active:scale-[0.98] transition-transform border-2 border-dashed border-black/20"
              >
                <span className="text-[32px] text-black/40">+</span>
                <span className="text-[16px] font-medium text-black/60">Planifier</span>
              </button>

              {schedules.map((schedule, index) => (
                <ScheduleCard
                  key={index}
                  zone={schedule.zone}
                  time={schedule.time}
                  onClick={() => onZoneClick?.(schedule.zone)}
                />
              ))}
            </div>
          </div>
        </>
      )}

      {activeTab === 'control' && (
        <ControlPanel cameraUrl={cameraUrl} />
      )}

      {/* Tabs */}
      {/* Bottom Tabs */}
      <div className="fixed bottom-8 left-1/2 -translate-x-1/2 w-full max-w-md px-5 z-20">
        <div className="flex gap-2 rounded-full bg-white/90 p-1 shadow-lg backdrop-blur-sm">
          <button
            onClick={() => setActiveTab('monitor')}
            className={`flex-1 rounded-full py-3 text-sm font-medium transition-all ${activeTab === 'monitor'
              ? 'bg-black text-white shadow-md'
              : 'text-black/60 hover:bg-black/5'
              }`}
          >
            Monitor
          </button>
          <button
            onClick={() => setActiveTab('control')}
            className={`flex-1 rounded-full py-3 text-sm font-medium transition-all ${activeTab === 'control'
              ? 'bg-black text-white shadow-md'
              : 'text-black/60 hover:bg-black/5'
              }`}
          >
            Control
          </button>
        </div>
      </div>

      {/* Offline overlay */}
      {showOfflineOverlay && (
        <div className="fixed inset-0 bg-black/40 flex items-center justify-center p-6 z-50">
          <div className="bg-white rounded-2xl p-6 max-w-sm w-full text-center shadow-xl">
            <div className="w-16 h-16 bg-red-100 rounded-full flex items-center justify-center mx-auto mb-4">
              <span className="text-3xl">📡</span>
            </div>
            <h2 className="text-xl font-bold text-black mb-2">
              Connexion perdue
            </h2>
            <p className="text-black/60 mb-4">
              Tentative de reconnexion au robot...
            </p>
            <div className="flex justify-center gap-1">
              <span className="w-2 h-2 bg-black/40 rounded-full animate-bounce" />
              <span className="w-2 h-2 bg-black/40 rounded-full animate-bounce [animation-delay:100ms]" />
              <span className="w-2 h-2 bg-black/40 rounded-full animate-bounce [animation-delay:200ms]" />
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
