/**
 * Autonomy indicator showing remaining time based on battery level.
 * Same style as BatteryIndicator.
 */

import { Clock } from 'lucide-react';

interface AutonomyIndicatorProps {
  batteryLevel: number;
}

// Max autonomy at 100% battery (in minutes)
const MAX_AUTONOMY_MINUTES = 180; // 3 hours

function formatAutonomy(level: number): string {
  const minutes = Math.round((level / 100) * MAX_AUTONOMY_MINUTES);
  const hours = Math.floor(minutes / 60);
  const mins = minutes % 60;
  
  if (hours === 0) {
    return `${mins}min`;
  }
  if (mins === 0) {
    return `${hours}h`;
  }
  return `${hours}h${mins.toString().padStart(2, '0')}`;
}

export function AutonomyIndicator({ batteryLevel }: AutonomyIndicatorProps) {
  const autonomy = formatAutonomy(batteryLevel);
  
  return (
    <div className="flex items-center gap-1">
      <Clock size={28} className="-rotate-90 text-black" />
      <span className="text-[24px] font-medium tracking-tight text-black">
        {autonomy}
      </span>
    </div>
  );
}
