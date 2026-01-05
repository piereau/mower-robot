/**
 * 3D Robot image display with shadow effect.
 */

import robotImage from '../assets/robot-3d.png';

export function RobotImage() {
  return (
    <div className="relative w-full max-w-[321px] h-[204px]">
      {/* Shadow effect */}
      {/* <div 
        className="absolute inset-0 pointer-events-none"
        style={{
          boxShadow: `-67px 217px 64px 0px rgba(0,0,0,0),
                      -43px 139px 58px 0px rgba(0,0,0,0.01),
                      -24px 78px 49px 0px rgba(0,0,0,0.05),
                      -11px 35px 36px 0px rgba(0,0,0,0.09),
                      -3px 9px 20px 0px rgba(0,0,0,0.1)`,
        }}
      /> */}
      
      {/* Robot image */}
      <img 
        src={robotImage} 
        alt="RoboMower 3D" 
        className="absolute inset-0 w-full h-full object-contain object-center"
      />
    </div>
  );
}
