/**
 * Interactive map component showing parcels, zones, and robot position.
 */

import { useEffect, useRef, useState } from "react";
import { ArrowLeft } from "lucide-react";

type Pt = { x: number; y: number };
type Zone = {
  id: string;
  type: "mow" | "noGo";
  name?: string;
  color: string;
  points: Pt[];
};

type MapData = {
  width: number;
  height: number;
  parcel: Pt[];
  zones: Zone[];
};

type RobotState = {
  x: number;
  y: number;
  headingRad: number;
};

function ptsToString(points: Pt[]) {
  return points.map((p) => `${p.x},${p.y}`).join(" ");
}

function clamp(v: number, min: number, max: number) {
  return Math.max(min, Math.min(max, v));
}

// Simple path: robot moves along these points in a loop
const PATH: Pt[] = [
  { x: 180, y: 220 },
  { x: 820, y: 240 },
  { x: 780, y: 760 },
  { x: 260, y: 820 },
  { x: 180, y: 220 },
];

const MOCK_MAP: MapData = {
  width: 1000,
  height: 1000,
  parcel: [
    { x: 120, y: 160 },
    { x: 900, y: 150 },
    { x: 860, y: 860 },
    { x: 160, y: 900 },
  ],
  zones: [
    {
      id: "zone-a",
      type: "mow",
      name: "Zone A",
      color: "#00DC8C",
      points: [
        { x: 160, y: 190 },
        { x: 520, y: 180 },
        { x: 520, y: 620 },
        { x: 220, y: 720 },
        { x: 160, y: 520 },
      ],
    },
    {
      id: "zone-b",
      type: "mow",
      name: "Zone B",
      color: "#155DFC",
      points: [
        { x: 520, y: 180 },
        { x: 880, y: 170 },
        { x: 840, y: 680 },
        { x: 520, y: 620 },
      ],
    },
    {
      id: "nog-1",
      type: "noGo",
      name: "No-Go",
      color: "#FB2C36",
      points: [
        { x: 640, y: 520 },
        { x: 740, y: 500 },
        { x: 770, y: 610 },
        { x: 660, y: 640 },
      ],
    },
  ],
};

interface MowerMapProps {
  zoneName: string;
  onBack: () => void;
}

export function MowerMap({ zoneName, onBack }: MowerMapProps) {
  const map = MOCK_MAP;

  const containerRef = useRef<HTMLDivElement | null>(null);
  const [scale, setScale] = useState(0.6);
  const [tx, setTx] = useState(0);
  const [ty, setTy] = useState(0);

  const [isPanning, setIsPanning] = useState(false);
  const panStart = useRef<{ x: number; y: number; tx: number; ty: number } | null>(null);

  // Robot sim
  const [robot, setRobot] = useState<RobotState>({ x: PATH[0].x, y: PATH[0].y, headingRad: 0 });
  const simRef = useRef<{ seg: number; t: number }>({ seg: 0, t: 0 });

  // Center content on mount
  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const { width, height } = el.getBoundingClientRect();

    const s = Math.min(width / map.width, height / map.height) * 0.9;
    setScale(s);

    const centeredTx = (width - map.width * s) / 2;
    const centeredTy = (height - map.height * s) / 2;
    setTx(centeredTx);
    setTy(centeredTy);
  }, [map.width, map.height]);

  // Robot animation loop
  useEffect(() => {
    let raf = 0;
    let last = performance.now();

    const tick = (now: number) => {
      const dt = (now - last) / 1000;
      last = now;

      const speed = 160;
      const st = simRef.current;
      let seg = st.seg;
      let t = st.t;

      const a = PATH[seg];
      const b = PATH[seg + 1] ?? PATH[0];

      const dx = b.x - a.x;
      const dy = b.y - a.y;
      const len = Math.hypot(dx, dy) || 1;

      const distThisFrame = speed * dt;
      const dtOnSeg = distThisFrame / len;
      t += dtOnSeg;

      while (t >= 1) {
        t -= 1;
        seg = (seg + 1) % (PATH.length - 1);
      }

      const x = a.x + (b.x - a.x) * t;
      const y = a.y + (b.y - a.y) * t;
      const headingRad = Math.atan2(dy, dx);

      simRef.current = { seg, t };
      setRobot({ x, y, headingRad });

      raf = requestAnimationFrame(tick);
    };

    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, []);

  const onWheel = (e: React.WheelEvent<HTMLDivElement>) => {
    e.preventDefault();
    const el = containerRef.current;
    if (!el) return;

    const rect = el.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;

    const zoomIntensity = 0.0016;
    const delta = -e.deltaY;
    const factor = Math.exp(delta * zoomIntensity);

    const newScale = clamp(scale * factor, 0.25, 3.5);

    const worldX = (mx - tx) / scale;
    const worldY = (my - ty) / scale;

    const newTx = mx - worldX * newScale;
    const newTy = my - worldY * newScale;

    setScale(newScale);
    setTx(newTx);
    setTy(newTy);
  };

  const onPointerDown = (e: React.PointerEvent<HTMLDivElement>) => {
    (e.currentTarget as HTMLDivElement).setPointerCapture(e.pointerId);
    setIsPanning(true);
    panStart.current = { x: e.clientX, y: e.clientY, tx, ty };
  };

  const onPointerMove = (e: React.PointerEvent<HTMLDivElement>) => {
    if (!isPanning || !panStart.current) return;
    const dx = e.clientX - panStart.current.x;
    const dy = e.clientY - panStart.current.y;
    setTx(panStart.current.tx + dx);
    setTy(panStart.current.ty + dy);
  };

  const onPointerUp = (e: React.PointerEvent<HTMLDivElement>) => {
    setIsPanning(false);
    panStart.current = null;
    try {
      (e.currentTarget as HTMLDivElement).releasePointerCapture(e.pointerId);
    } catch {
      // Ignore
    }
  };

  return (
    <div className="w-full max-w-md min-h-screen bg-[#d9e7f3] px-5 py-6">
      {/* Header with back button */}
      <div className="mb-4 flex items-center gap-3">
        <button
          onClick={onBack}
          className="flex items-center justify-center w-10 h-10 rounded-xl bg-white shadow-sm"
        >
          <ArrowLeft size={20} className="text-black" />
        </button>
        <div>
          <h1 className="text-2xl font-semibold text-black">{zoneName}</h1>
          <p className="text-sm text-black/50">Carte de la parcelle</p>
        </div>
      </div>

      {/* Map container */}
      <div
        ref={containerRef}
        onWheel={onWheel}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        className="relative h-[480px] w-full touch-none overflow-hidden rounded-2xl  ring-1 ring-slate-200"
      >
        {/* UI overlay */}
        <div className="pointer-events-none absolute left-3 top-3 rounded-xl bg-white/80 px-3 py-2 text-xs text-slate-700 backdrop-blur">
          <div className="font-medium">Mapping</div>
          <div className="text-slate-500">Zoom: {(scale * 100).toFixed(0)}%</div>
        </div>

        <svg className="h-full w-full">
          <g transform={`translate(${tx} ${ty}) scale(${scale})`}>
            {/* Background grid - more visible */}
            <defs>
              <pattern id="grid" width="50" height="50" patternUnits="userSpaceOnUse">
                <path d="M 50 0 L 0 0 0 50" fill="none" stroke="rgba(100, 116, 139, 0.2)" strokeWidth="1" />
              </pattern>
              <clipPath id="parcelClip">
                <polygon points={ptsToString(map.parcel)} />
              </clipPath>
            </defs>

            {/* Extended grid to appear infinite */}
            <rect x={-map.width * 2} y={-map.height * 2} width={map.width * 5} height={map.height * 5} fill="url(#grid)" />

            {/* Parcel outline */}
            <polygon
              points={ptsToString(map.parcel)}
              fill="#FFFFFF"
              stroke="rgba(15,23,42,0.25)"
              strokeWidth="6"
              strokeLinejoin="round"
            />

            {/* Zones */}
            <g clipPath="url(#parcelClip)">
              {map.zones
                .filter((z) => z.type === "mow")
                .map((z) => (
                  <polygon
                    key={z.id}
                    points={ptsToString(z.points)}
                    fill={z.color}
                    opacity={0.85}
                    stroke="rgba(15,23,42,0.25)"
                    strokeWidth="2"
                    strokeLinejoin="round"
                  />
                ))}

              {map.zones
                .filter((z) => z.type === "noGo")
                .map((z) => (
                  <polygon
                    key={z.id}
                    points={ptsToString(z.points)}
                    fill={z.color}
                    opacity={0.9}
                    stroke="rgba(15,23,42,0.3)"
                    strokeWidth="2"
                    strokeDasharray="10 8"
                    strokeLinejoin="round"
                  />
                ))}
            </g>

            {/* Robot marker */}
            <g transform={`translate(${robot.x} ${robot.y}) rotate(${(robot.headingRad * 180) / Math.PI})`}>
              <circle r="14" fill="white" stroke="rgba(15,23,42,0.25)" strokeWidth="3" />
              <path
                d="M 0 -20 L 6 -8 L -6 -8 Z"
                fill="rgba(59,130,246,0.95)"
                stroke="rgba(15,23,42,0.15)"
                strokeWidth="1"
              />
              <circle r="3" fill="rgba(59,130,246,0.95)" />
            </g>
          </g>
        </svg>

      </div>

      {/* Zone legend */}
      <div className="mt-4 flex flex-wrap gap-3">
        {map.zones.map((zone) => (
          <div key={zone.id} className="flex items-center gap-2">
            <div
              className="w-4 h-4 rounded"
              style={{ backgroundColor: zone.color, opacity: zone.type === "noGo" ? 0.9 : 0.85 }}
            />
            <span className="text-sm text-black/70">{zone.name}</span>
          </div>
        ))}
      </div>

      {/* Stats section */}
      <div className="mt-6 bg-[#f0f5fa] rounded-2xl p-5">
        {/* Unschedule button */}
        <button className="w-full bg-black text-white rounded-xl py-4 px-6 flex items-center justify-center gap-3 font-medium text-lg">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="3" strokeLinecap="round">
            <line x1="8" y1="5" x2="8" y2="19" />
            <line x1="16" y1="5" x2="16" y2="19" />
          </svg>
          Unschedule
        </button>

        {/* Stats row */}
        <div className="mt-6 flex justify-around">
          {/* Surface */}
          <div className="text-center">
            <div className="flex items-baseline justify-center gap-1">
              <span className="text-5xl font-semibold text-black">300</span>
              <span className="text-xl text-black">m²</span>
            </div>
            <p className="text-sm text-black/50 mt-1">Surface totale</p>
          </div>

          {/* Duration */}
          <div className="text-center">
            <div className="flex items-baseline justify-center gap-1">
              <span className="text-5xl font-semibold text-black">1h20</span>
            </div>
            <p className="text-sm text-black/50 mt-1">Durée estimée</p>
          </div>
        </div>
      </div>
    </div>
  );
}
