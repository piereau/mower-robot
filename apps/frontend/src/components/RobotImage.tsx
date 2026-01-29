/**
 * 3D Robot image display with shadow effect.
 */

import robotImage from '../assets/robot-3d.png';

export function RobotImage() {
  return (
    <div className="relative w-full max-w-[356px] h-[244px]">
      {/* Robot image - shifted to compensate for visual imbalance */}
      <img 
        src={robotImage} 
        alt="RoboMower 3D" 
        className="absolute inset-0 w-full h-full object-contain translate-x-8 object-bottom"
      />
    </div>
  );
}
