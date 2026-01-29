import { useState } from 'react';
import { Joystick } from './Joystick';
import { useRobotControl } from '../hooks/useRobotControl';

type ControlPanelProps = {
  cameraUrl: string;
};

export function ControlPanel({ cameraUrl }: ControlPanelProps) {
  const { sendJoystick, stop, connected } = useRobotControl();
  const [cameraError, setCameraError] = useState(false);

  return (
    <div className="space-y-4">
      <div className="rounded-2xl bg-black/10 p-2">
        {cameraError ? (
          <div className="flex h-52 items-center justify-center rounded-xl bg-black/10 text-sm text-black/60">
            Flux caméra indisponible
          </div>
        ) : (
          <img
            src={cameraUrl}
            alt="Flux caméra"
            className="h-52 w-full rounded-xl object-cover"
            onError={() => setCameraError(true)}
          />
        )}
      </div>

      <div className="space-y-3">
        <h2 className="text-[28px] font-medium tracking-tight text-black">
          Joystick
        </h2>
        <Joystick onMove={sendJoystick} onEnd={stop} disabled={!connected} />
        <p className="text-sm text-black/60 text-center">
          Haut = avance, bas = recule, gauche/droite = tourne.
        </p>
        {!connected && (
          <p className="text-xs text-black/50 text-center">
            Commande désactivée : WebSocket non connectée.
          </p>
        )}
      </div>
    </div>
  );
}
