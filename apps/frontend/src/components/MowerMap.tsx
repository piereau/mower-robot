/**
 * Interactive vineyard map component showing rows and corridors.
 * Rows are numbered (1, 2, 3...), corridors are lettered (A, B, C...).
 */

import { useEffect, useRef, useState } from "react";
import { ArrowLeft } from "lucide-react";

type RobotState = {
  x: number;
  y: number;
  headingRad: number;
};

// Map configuration
const MAP_CONFIG = {
  width: 400,
  height: 500,
  rowCount: 8,
  rowSpacing: 40,
  rowStartX: 60,
  rowTopY: 50,
  rowHeight: 380,
};

// Generate row positions
function generateRows(config: typeof MAP_CONFIG) {
  const rows = [];
  for (let i = 0; i < config.rowCount; i++) {
    rows.push({
      id: i + 1,
      x: config.rowStartX + i * config.rowSpacing,
    });
  }
  return rows;
}

// Generate corridor labels (between rows)
function generateCorridors(config: typeof MAP_CONFIG) {
  const corridors = [];
  for (let i = 0; i < config.rowCount - 1; i++) {
    corridors.push({
      id: String.fromCharCode(65 + i), // A, B, C...
      x: config.rowStartX + i * config.rowSpacing + config.rowSpacing / 2,
      active: i < 3, // First 3 corridors are active (being mowed)
    });
  }
  return corridors;
}

// Robot path through corridors (continuous serpentine pattern)
function generateRobotPath(config: typeof MAP_CONFIG) {
  const path: { x: number; y: number }[] = [];

  // Build a continuous serpentine path through corridors A, B, C
  // Forward: A (down) -> B (up) -> C (down)
  // Return: C (up) -> B (down) -> A (up)

  const getCorridorX = (i: number) =>
    config.rowStartX + i * config.rowSpacing + config.rowSpacing / 2;

  const top = config.rowTopY;
  const bottom = config.rowTopY + config.rowHeight;

  // Forward pass: A (down) -> B (up) -> C (down)
  // Start at top of A
  path.push({ x: getCorridorX(0), y: top });
  path.push({ x: getCorridorX(0), y: bottom }); // End A at bottom

  path.push({ x: getCorridorX(1), y: bottom }); // Move to B at bottom
  path.push({ x: getCorridorX(1), y: top }); // Go up B to top

  path.push({ x: getCorridorX(2), y: top }); // Move to C at top
  path.push({ x: getCorridorX(2), y: bottom }); // Go down C to bottom

  // Return pass: C (up) -> B (down) -> A (up)
  path.push({ x: getCorridorX(2), y: top }); // Go up C to top

  path.push({ x: getCorridorX(1), y: top }); // Move to B at top
  path.push({ x: getCorridorX(1), y: bottom }); // Go down B to bottom

  path.push({ x: getCorridorX(0), y: bottom }); // Move to A at bottom
  path.push({ x: getCorridorX(0), y: top }); // Go up A to top (back to start)

  return path;
}

const ROWS = generateRows(MAP_CONFIG);
const CORRIDORS = generateCorridors(MAP_CONFIG);
const ROBOT_PATH = generateRobotPath(MAP_CONFIG);

interface MowerMapProps {
  zoneName: string;
  onBack: () => void;
}

export function MowerMap({ zoneName, onBack }: MowerMapProps) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const [scale, setScale] = useState(1);
  const [tx, setTx] = useState(0);
  const [ty, setTy] = useState(0);

  const [isPanning, setIsPanning] = useState(false);
  const panStart = useRef<{ x: number; y: number; tx: number; ty: number } | null>(null);

  // Robot simulation
  const [robot, setRobot] = useState<RobotState>({ x: ROBOT_PATH[0].x, y: ROBOT_PATH[0].y, headingRad: Math.PI / 2 });
  const simRef = useRef<{ seg: number; t: number }>({ seg: 0, t: 0 });

  // Center content on mount
  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const { width, height } = el.getBoundingClientRect();

    const s = Math.min(width / MAP_CONFIG.width, height / MAP_CONFIG.height) * 0.95;
    setScale(s);

    const centeredTx = (width - MAP_CONFIG.width * s) / 2;
    const centeredTy = (height - MAP_CONFIG.height * s) / 2;
    setTx(centeredTx);
    setTy(centeredTy);
  }, []);

  // Robot animation loop
  useEffect(() => {
    let raf = 0;
    let last = performance.now();

    const tick = (now: number) => {
      const dt = (now - last) / 1000;
      last = now;

      const speed = 80;
      const st = simRef.current;
      let seg = st.seg;
      let t = st.t;

      const a = ROBOT_PATH[seg];
      const b = ROBOT_PATH[seg + 1] ?? ROBOT_PATH[0];

      const dx = b.x - a.x;
      const dy = b.y - a.y;
      const len = Math.hypot(dx, dy) || 1;

      const distThisFrame = speed * dt;
      const dtOnSeg = distThisFrame / len;
      t += dtOnSeg;

      while (t >= 1) {
        t -= 1;
        seg = (seg + 1) % (ROBOT_PATH.length - 1);
      }

      const currA = ROBOT_PATH[seg];
      const currB = ROBOT_PATH[seg + 1] ?? ROBOT_PATH[0];
      const x = currA.x + (currB.x - currA.x) * t;
      const y = currA.y + (currB.y - currA.y) * t;
      const headingRad = Math.atan2(currB.y - currA.y, currB.x - currA.x);

      simRef.current = { seg, t };
      setRobot({ x, y, headingRad });

      raf = requestAnimationFrame(tick);
    };

    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, []);

  // Pan/zoom handlers
  const clamp = (v: number, min: number, max: number) => Math.max(min, Math.min(max, v));

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

    const newScale = clamp(scale * factor, 0.5, 3);

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
          <p className="text-sm text-black/50">Carte des rangs</p>
        </div>
      </div>

      {/* Map container */}
      <div
        ref={containerRef}
        onWheel={onWheel}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        className="relative h-[480px] w-full touch-none overflow-hidden rounded-2xl bg-[#f0f4f8] ring-1 ring-slate-200"
      >
        <svg className="h-full w-full">
          <g transform={`translate(${tx} ${ty}) scale(${scale})`}>
            {/* Active corridors (green glow) */}
            {CORRIDORS.filter(c => c.active).map((corridor) => (
              <rect
                key={`corridor-glow-${corridor.id}`}
                x={corridor.x - 12}
                y={MAP_CONFIG.rowTopY}
                width={24}
                height={MAP_CONFIG.rowHeight}
                rx={8}
                fill="#00C950"
                opacity={0.7}
                filter="url(#corridorGlow)"
              />
            ))}

            {/* Glow filter */}
            <defs>
              <filter id="corridorGlow" x="-50%" y="-50%" width="200%" height="200%">
                <feGaussianBlur stdDeviation="5" result="blur" />
                <feMerge>
                  <feMergeNode in="blur" />
                  <feMergeNode in="SourceGraphic" />
                </feMerge>
              </filter>
            </defs>

            {/* Row lines (vineyard rows) */}
            {ROWS.map((row) => (
              <line
                key={`row-${row.id}`}
                x1={row.x}
                y1={MAP_CONFIG.rowTopY}
                x2={row.x}
                y2={MAP_CONFIG.rowTopY + MAP_CONFIG.rowHeight}
                stroke="#1e293b"
                strokeWidth={2}
                strokeLinecap="round"
              />
            ))}

            {/* Row numbers (top) */}
            {ROWS.map((row) => (
              <text
                key={`row-label-${row.id}`}
                x={row.x}
                y={MAP_CONFIG.rowTopY - 12}
                textAnchor="middle"
                className="text-xs font-medium fill-black"
                style={{ fontSize: 14 }}
              >
                {row.id}
              </text>
            ))}

            {/* Corridor letters (bottom) */}
            {CORRIDORS.map((corridor) => (
              <text
                key={`corridor-label-${corridor.id}`}
                x={corridor.x}
                y={MAP_CONFIG.rowTopY + MAP_CONFIG.rowHeight + 25}
                textAnchor="middle"
                className="text-xs font-medium fill-black"
                style={{ fontSize: 14 }}
              >
                {corridor.id}
              </text>
            ))}

            {/* Robot marker */}
            <g transform={`translate(${robot.x} ${robot.y}) rotate(${(robot.headingRad * 180) / Math.PI + 90})`}>
              <circle r="10" fill="white" stroke="rgba(15,23,42,0.3)" strokeWidth="2" />
              <path
                d="M 0 -14 L 4 -6 L -4 -6 Z"
                fill="rgba(59,130,246,0.95)"
                stroke="rgba(15,23,42,0.15)"
                strokeWidth="1"
              />
              <circle r="2" fill="rgba(59,130,246,0.95)" />
            </g>
          </g>
        </svg>


      </div>

      {/* Stats section */}
      <div className="mt-6 bg-[#f0f5fa] rounded-2xl p-5">
        {/* Déprogrammer button */}
        <button className="w-full bg-black text-white rounded-xl py-4 px-6 flex items-center justify-center gap-3 font-medium text-lg">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="3" strokeLinecap="round">
            <line x1="8" y1="5" x2="8" y2="19" />
            <line x1="16" y1="5" x2="16" y2="19" />
          </svg>
          Déprogrammer
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
