/**
 * Card showing a scheduled mowing session.
 * Clickable to navigate to map view.
 */

import { Clock } from 'lucide-react';

interface ScheduleCardProps {
  zone: string;
  time: string;
  onClick?: () => void;
}

export function ScheduleCard({ zone, time, onClick }: ScheduleCardProps) {
  return (
    <button
      onClick={onClick}
      className="bg-white rounded-xl p-3 flex flex-col gap-3 min-w-[160px] text-left active:scale-[0.98] transition-transform"
    >
      <p className="text-[24px] font-medium tracking-tight text-black">
        {zone}
      </p>
      <div className="flex items-center gap-3">
        <Clock size={24} className="text-black" />
        <span className="text-[24px] font-medium tracking-tight text-black whitespace-nowrap">
          {time}
        </span>
      </div>
    </button>
  );
}
