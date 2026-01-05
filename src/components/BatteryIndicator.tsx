/**
 * Battery indicator with icon and percentage.
 */

import { BatteryFull, BatteryMedium, BatteryLow, BatteryWarning } from 'lucide-react';

interface BatteryIndicatorProps {
  level: number;
}

function getBatteryIcon(level: number) {
  if (level >= 75) return BatteryFull;
  if (level >= 40) return BatteryMedium;
  if (level >= 15) return BatteryLow;
  return BatteryWarning;
}

function getBatteryColor(level: number): string {
  if (level >= 40) return 'text-black';
  if (level >= 15) return 'text-amber-500';
  return 'text-red-500';
}

export function BatteryIndicator({ level }: BatteryIndicatorProps) {
  const clampedLevel = Math.max(0, Math.min(100, level));
  const Icon = getBatteryIcon(clampedLevel);
  const colorClass = getBatteryColor(clampedLevel);
  
  return (
    <div className="flex items-end">
      <Icon size={32} className={`-rotate-90 ${colorClass}`} />
      <span className={`text-[24px] font-medium tracking-tight w-[50px] ${colorClass}`}>
        {clampedLevel}%
      </span>
    </div>
  );
}
