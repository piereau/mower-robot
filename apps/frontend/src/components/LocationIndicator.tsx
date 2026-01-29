/**
 * Location indicator showing where the robot is.
 */

import { MapPin } from 'lucide-react';

interface LocationIndicatorProps {
  location: string;
}

export function LocationIndicator({ location }: LocationIndicatorProps) {
  return (
    <div className="flex items-center gap-1">
      <MapPin size={32} className="text-black" />
      <span className="text-[32px] font-medium tracking-tight text-black">
        {location}
      </span>
    </div>
  );
}

