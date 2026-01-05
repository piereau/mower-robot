/**
 * Main dashboard page - Figma design implementation.
 */

import { useRobotTelemetry } from '../hooks/useRobotTelemetry';
import { RobotImage } from '../components/RobotImage';
import { StatusIndicator } from '../components/StatusIndicator';
import { BatteryIndicator } from '../components/BatteryIndicator';
import { LocationIndicator } from '../components/LocationIndicator';
import { ScheduleCard } from '../components/ScheduleCard';
import { ConnectionStatus } from '../components/ConnectionStatus';
import { Loader2 } from 'lucide-react';
import { deriveDisplayStatus } from '../types/telemetry';

// Mock schedule data (would come from backend in future)
const MOCK_SCHEDULES = [
  { zone: 'Parcelle A1', time: 'Lundi 9h30' },
  { zone: 'Parcelle B2', time: 'Mardi 14h00' },
];

interface DashboardProps {
  onZoneClick?: (zoneName: string) => void;
}

export function Dashboard({ onZoneClick }: DashboardProps) {
  const { telemetry, connectionState } = useRobotTelemetry();
  
  const showOfflineOverlay = connectionState === 'disconnected';
  
  // Derive display status from telemetry
  const displayStatus = telemetry ? deriveDisplayStatus(telemetry) : null;
  
  // Derive location based on status
  const location = displayStatus === 'mowing' ? 'Parcelle A1' : 'Parking';
  
  return (
    <div className="min-h-screen bg-[#d9e7f3] flex justify-center">
      {/* Mobile-width container */}
      <div className="w-full max-w-md px-5 py-8 relative">
        {/* Connection indicator - top right */}
        <div className="absolute top-4 right-4">
          <ConnectionStatus state={connectionState} />
        </div>
        
        {/* Robot image section */}
        <div className="pt-8 pb-12">
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
              
              {/* Location */}
              <LocationIndicator location={location} />
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
            {MOCK_SCHEDULES.map((schedule, index) => (
              <ScheduleCard 
                key={index} 
                zone={schedule.zone} 
                time={schedule.time}
                onClick={() => onZoneClick?.(schedule.zone)}
              />
            ))}
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
    </div>
  );
}
