import React, { useRef, useEffect } from 'react';
import type { MapMessage, ScanMessage, PoseMessage } from '../types/telemetry';

interface MapCanvasProps {
    mapData: MapMessage | null;
    scanData: ScanMessage | null;
    robotPose: PoseMessage | null;
    className?: string;
    showLidar?: boolean;
}

export const MapCanvas: React.FC<MapCanvasProps> = ({
    mapData,
    scanData,
    robotPose,
    className = "",
    showLidar = true
}) => {
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const [transform, setTransform] = React.useState({ x: 0, y: 0, k: 1 });
    const [isDragging, setIsDragging] = React.useState(false);
    const dragStart = useRef({ x: 0, y: 0 });

    // Handle Pan/Zoom
    const handleWheel = (e: React.WheelEvent) => {
        e.preventDefault();
        const sc = 0.001;
        const newK = Math.max(0.1, Math.min(transform.k * Math.exp(-e.deltaY * sc), 50));
        setTransform(t => ({ ...t, k: newK }));
    };

    const handlePointerDown = (e: React.PointerEvent) => {
        (e.currentTarget as Element).setPointerCapture(e.pointerId);
        setIsDragging(true);
        dragStart.current = { x: e.clientX - transform.x, y: e.clientY - transform.y };
    };

    const handlePointerMove = (e: React.PointerEvent) => {
        if (!isDragging) return;
        setTransform(t => ({ ...t, x: e.clientX - dragStart.current.x, y: e.clientY - dragStart.current.y }));
    };

    const handlePointerUp = (e: React.PointerEvent) => {
        setIsDragging(false);
        try { (e.currentTarget as Element).releasePointerCapture(e.pointerId); } catch { }
    };

    // Initial center when map loads
    useEffect(() => {
        if (mapData && transform.k === 1 && transform.x === 0 && transform.y === 0) {
            // Optional: Auto-fit logic could go here, but defaulting to 1x scale centered might be fine
            // or we keep the previous auto-scale logic but just set the initial transform state
            const canvas = canvasRef.current;
            if (!canvas) return;

            const { width, height } = mapData.info;
            const scaleX = canvas.width / width;
            const scaleY = canvas.height / height;
            const scale = Math.min(scaleX, scaleY) * 0.9; // 90% fit

            const offsetX = (canvas.width - width * scale) / 2;
            const offsetY = (canvas.height - height * scale) / 2;

            setTransform({ x: offsetX, y: offsetY, k: scale });
        }
    }, [mapData]);

    useEffect(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // Fill background
        ctx.fillStyle = '#e5e7eb';
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        if (!mapData) {
            ctx.fillStyle = '#6b7280';
            ctx.font = '16px sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText('Waiting for map...', canvas.width / 2, canvas.height / 2);
            return;
        }

        const { width, height, resolution, origin } = mapData.info;
        const data = mapData.data;

        ctx.save();
        ctx.translate(transform.x, transform.y);
        ctx.scale(transform.k, transform.k);

        // Draw Map
        // Iterate only visible area optimization could be added here, 
        // but for now we rely on canvas clipping for performance on reasonable map sizes.

        // We still need the base offsets from the previous logic? 
        // No, transform.x/y/k REPLACES the fixed scale/offset. 
        // But we need to define what (0,0) means. 
        // Let's assume (0,0) in canvas context AFTER transform is the top-left of the map grid.

        for (let y = 0; y < height; y++) {
            for (let x = 0; x < width; x++) {
                const i = y * width + x;
                const val = data[i];
                if (val === -1) continue;

                if (val === 0) ctx.fillStyle = '#ffffff';
                else if (val === 100) ctx.fillStyle = '#1f2937';
                else {
                    const gray = 255 - Math.floor((val / 100) * 255);
                    ctx.fillStyle = `rgb(${gray}, ${gray}, ${gray})`;
                }

                // Flip Y for visual consistency with standard map expectations if needed
                // Default: Row 0 is top. 
                // ROS: Row 0 is usually bottom (y=0).
                // Let's draw row 0 at visual bottom: (height - 1 - y)

                ctx.fillRect(x, height - 1 - y, 1, 1);
            }
        }

        // Helper for Lidar/Robot overlay
        const worldToLocal = (wx: number, wy: number) => {
            const cx = (wx - origin.position.x) / resolution;
            const cy = (wy - origin.position.y) / resolution;
            return { x: cx, y: height - 1 - cy };
        };

        // Draw Lidar
        if (showLidar && scanData && robotPose) {
            ctx.fillStyle = '#ef4444';
            const angleMin = scanData.angle_min;
            const angleInc = scanData.angle_increment;
            const rx = robotPose.x;
            const ry = robotPose.y;
            const r_yaw = robotPose.yaw;

            for (let i = 0; i < scanData.ranges.length; i++) {
                const range = scanData.ranges[i];
                if (range < scanData.range_min || range > scanData.range_max || range === 0) continue;

                const angle = angleMin + i * angleInc;
                const lx = range * Math.cos(angle);
                const ly = range * Math.sin(angle);

                const mx = rx + (lx * Math.cos(r_yaw) - ly * Math.sin(r_yaw));
                const my = ry + (lx * Math.sin(r_yaw) + ly * Math.cos(r_yaw));

                const pt = worldToLocal(mx, my);

                // Draw point (radius relative to scale? No, keep fixed pixel size)
                // To keep fixed pixel size despite zoom, we inverse scale
                const dotSize = 2 / transform.k;
                ctx.beginPath();
                ctx.arc(pt.x, pt.y, dotSize, 0, 2 * Math.PI);
                ctx.fill();
            }
        }

        // Draw Robot
        if (robotPose) {
            const pt = worldToLocal(robotPose.x, robotPose.y);

            ctx.save();
            ctx.translate(pt.x, pt.y);
            ctx.rotate(-robotPose.yaw); // Canvas rotation

            ctx.fillStyle = '#3b82f6';
            ctx.beginPath();
            // Scale robot icon size inversely so it doesn't get huge? 
            // Or keep it physical size? Physical is better for map.
            // Let's assume physical size: 0.5m ~ 10 pixels at res 0.05
            // Draw relative to map units (1 unit = 1 pixel at scale 1 = 1 cell = resolution meters)
            // Robot is approx 0.5m. Res is 0.05m. So robot is 10 units long.

            ctx.moveTo(5, 0);
            ctx.lineTo(-3, 3);
            ctx.lineTo(-3, -3);
            ctx.closePath();
            ctx.fill();

            ctx.restore();
        }

        ctx.restore();

    }, [mapData, scanData, robotPose, showLidar, transform]);

    return (
        <div className={`relative rounded-lg overflow-hidden bg-gray-100 ${className} touch-none`}>
            <canvas
                ref={canvasRef}
                width={600}
                height={600} // Internal resolution
                className="w-full h-full object-contain cursor-move"
                onWheel={handleWheel}
                onPointerDown={handlePointerDown}
                onPointerMove={handlePointerMove}
                onPointerUp={handlePointerUp}
                onPointerLeave={handlePointerUp}
            />

            {/* Legend / Status Overlay */}
            <div className="absolute top-2 left-2 bg-white/80 p-2 rounded text-xs pointer-events-none select-none">
                {robotPose && (
                    <div>
                        POSE: ({robotPose.x.toFixed(2)}, {robotPose.y.toFixed(2)})
                        θ: {(robotPose.yaw * 180 / Math.PI).toFixed(0)}°
                    </div>
                )}
                <div className="text-gray-500 mt-1">Scroll to Zoom • Drag to Pan</div>
            </div>
        </div>
    );
};
