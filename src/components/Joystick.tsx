import { useCallback, useRef, useState } from 'react';

type JoystickProps = {
  onMove: (x: number, y: number) => void;
  onEnd: () => void;
  disabled?: boolean;
};

type Position = {
  x: number;
  y: number;
};

const KNOB_SIZE = 64;

export function Joystick({ onMove, onEnd, disabled = false }: JoystickProps) {
  const baseRef = useRef<HTMLDivElement | null>(null);
  const activeRef = useRef(false);
  const [knob, setKnob] = useState<Position>({ x: 0, y: 0 });

  const updateFromPointer = useCallback(
    (clientX: number, clientY: number) => {
      const base = baseRef.current;
      if (!base) return;

      const rect = base.getBoundingClientRect();
      const radius = rect.width / 2;
      const centerX = rect.left + radius;
      const centerY = rect.top + radius;

      const dx = clientX - centerX;
      const dy = clientY - centerY;
      const distance = Math.sqrt(dx * dx + dy * dy);
      const scale = distance > radius ? radius / distance : 1;

      const clampedX = dx * scale;
      const clampedY = dy * scale;

      setKnob({ x: clampedX, y: clampedY });
      onMove(clampedX / radius, -clampedY / radius);
    },
    [onMove]
  );

  const handlePointerDown = useCallback(
    (event: React.PointerEvent<HTMLDivElement>) => {
      if (disabled) return;
      activeRef.current = true;
      event.currentTarget.setPointerCapture(event.pointerId);
      updateFromPointer(event.clientX, event.clientY);
    },
    [disabled, updateFromPointer]
  );

  const handlePointerMove = useCallback(
    (event: React.PointerEvent<HTMLDivElement>) => {
      if (!activeRef.current || disabled) return;
      updateFromPointer(event.clientX, event.clientY);
    },
    [disabled, updateFromPointer]
  );

  const handlePointerUp = useCallback(
    (event: React.PointerEvent<HTMLDivElement>) => {
      if (disabled) return;
      activeRef.current = false;
      event.currentTarget.releasePointerCapture(event.pointerId);
      setKnob({ x: 0, y: 0 });
      onMove(0, 0);
      onEnd();
    },
    [disabled, onEnd, onMove]
  );

  const handlePointerLeave = useCallback(() => {
    if (!activeRef.current || disabled) return;
    activeRef.current = false;
    setKnob({ x: 0, y: 0 });
    onMove(0, 0);
    onEnd();
  }, [disabled, onEnd, onMove]);

  return (
    <div className="w-full">
      <div
        ref={baseRef}
        className={`relative mx-auto h-52 w-52 rounded-full border-2 border-black/20 bg-white/70 shadow-inner ${
          disabled ? 'opacity-50' : ''
        }`}
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerCancel={handlePointerLeave}
        onPointerLeave={handlePointerLeave}
        style={{ touchAction: 'none' }}
      >
        <div
          className="absolute left-1/2 top-1/2 rounded-full bg-black/80 shadow-lg"
          style={{
            width: KNOB_SIZE,
            height: KNOB_SIZE,
            transform: `translate(-50%, -50%) translate(${knob.x}px, ${knob.y}px)`,
          }}
        />
      </div>
    </div>
  );
}
