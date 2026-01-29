/**
 * Status indicator showing robot state with icon.
 * 
 * States:
 * - ready: Prêt à tondre (green check) - battery >= 80%
 * - charging: En charge (amber zap) - at parking, battery < 80%
 * - mowing: En cours (blue accessibility) - actively mowing
 */

import { CircleCheck, Zap, Accessibility } from 'lucide-react';
import type { DisplayStatus } from '../types/telemetry';

interface StatusIndicatorProps {
  status: DisplayStatus;
}

const STATUS_CONFIG: Record<DisplayStatus, { 
  label: string; 
  Icon: typeof CircleCheck; 
  iconClass: string;
  textClass: string;
}> = {
  ready: {
    label: 'Prêt à tondre',
    Icon: CircleCheck,
    iconClass: 'text-emerald-500',
    textClass: 'text-black',
  },
  charging: {
    label: 'En charge',
    Icon: Zap,
    iconClass: 'text-amber-500',
    textClass: 'text-black',
  },
  mowing: {
    label: 'En cours',
    Icon: Accessibility,
    iconClass: 'text-blue-500',
    textClass: 'text-black',
  },
};

export function StatusIndicator({ status }: StatusIndicatorProps) {
  const config = STATUS_CONFIG[status];
  const Icon = config.Icon;
  
  return (
    <div className="flex items-center gap-1">
      <Icon 
        size={32} 
        className={`${config.iconClass} ${status === 'mowing' ? 'animate-pulse' : ''}`}
        fill={status === 'charging' ? 'currentColor' : 'none'}
      />
      <span className={`text-[32px] font-medium tracking-tight ${config.textClass}`}>
        {config.label}
      </span>
    </div>
  );
}
