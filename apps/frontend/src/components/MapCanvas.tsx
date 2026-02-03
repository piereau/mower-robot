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

    useEffect(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // Fill background (Unknown space)
        ctx.fillStyle = '#e5e7eb'; // gray-200
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        if (!mapData) {
            // Draw placeholder text
            ctx.fillStyle = '#6b7280'; // gray-500
            ctx.font = '16px sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText('Waiting for map...', canvas.width / 2, canvas.height / 2);
            return;
        }

        // --- Draw Map ---
        const { width, height, resolution, origin } = mapData.info;
        const data = mapData.data;

        // Calculate scale to fit map in canvas
        // We want to preserve aspect ratio
        const scaleX = canvas.width / width;
        const scaleY = canvas.height / height;
        const scale = Math.min(scaleX, scaleY);

        // Center the map in canvas
        const offsetX = (canvas.width - width * scale) / 2;
        const offsetY = (canvas.height - height * scale) / 2;

        // Draw Occupancy Grid
        // Optimizing: Draw solely 1x1 rectangles for occupied/free cells is slow for strict pixel mapping?
        // Map pixels are cells.
        // We can iterate data.

        // Standard ROS Map: 0-100 (prob), -1 (unknown).
        // Data is row-major, starting from (0,0) which is usually bottom-left in map frame?
        // ROS OccupancyGrid origin is the pose of the cell (0,0).
        // If orientation is identity, rows go X+, cols go Y+? No.
        // Standard: index = y * width + x.
        // (x,y) corresponds to world: origin + (x*res, y*res).

        // We need to transform World -> Canvas.
        // Canvas X increases Right. Canvas Y increases Down.
        // ROS Map usually: World Y increases Up.
        // So we need to flip Y.

        // Let's simply draw the grid first treating index roughly as x,y.
        // Iterate y from 0 to height-1
        for (let y = 0; y < height; y++) {
            for (let x = 0; x < width; x++) {
                const i = y * width + x;
                const val = data[i];

                if (val === -1) continue; // Unknown (already gray)

                // Determine color
                // 0 = Free (White)
                // 100 = Occupied (Black)
                // 1-99 = Prob
                if (val === 0) {
                    ctx.fillStyle = '#ffffff';
                } else if (val === 100) {
                    ctx.fillStyle = '#1f2937'; // gray-800
                } else {
                    // Gradient
                    const gray = 255 - Math.floor((val / 100) * 255);
                    ctx.fillStyle = `rgb(${gray}, ${gray}, ${gray})`;
                }

                // Draw Rect
                // Canvas Coordinates:
                // We flip Y: row 0 is at bottom of map?
                // Usually ROS map data row 0 is for y=0 (bottom).
                // So we want to draw row 0 at the BOTTOM of our canvas map area.
                // visual_y = (height - 1 - y)

                const visualY = (height - 1 - y);

                ctx.fillRect(
                    offsetX + x * scale,
                    offsetY + visualY * scale,
                    scale,
                    scale
                );
            }
        }

        // --- Helper: World -> Canvas Transform ---
        // World point (wx, wy)
        // 1. To Map Cell (cx, cy):
        //    cx = (wx - origin.x) / res
        //    cy = (wy - origin.y) / res
        // 2. To Visual Cell (vx, vy):
        //    vx = cx
        //    vy = height - 1 - cy
        // 3. To Canvas Pixel (px, py):
        //    px = offsetX + vx * scale
        //    py = offsetY + vy * scale

        const worldToCanvas = (wx: number, wy: number) => {
            const cx = (wx - origin.position.x) / resolution;
            const cy = (wy - origin.position.y) / resolution;

            const vx = cx;
            const vy = height - 1 - cy; // Flip Y

            return {
                x: offsetX + vx * scale,
                y: offsetY + vy * scale
            };
        };

        // --- Draw LiDAR ---
        if (showLidar && scanData && robotPose) {
            ctx.fillStyle = '#ef4444'; // red-500
            const angleMin = scanData.angle_min;
            const angleInc = scanData.angle_increment;

            // Robot Pose + Lidar Transform?
            // Assume scan is in robot frame (or we need TF).
            // If we only have Robot Pose in Map frame, we assume Scan is in Robot frame (base_link).
            // Simplified: Lidar is AT robot center (or close enough).

            // Robot Pose: x, y, yaw (in Map frame).
            const rx = robotPose.x;
            const ry = robotPose.y;
            const r_yaw = robotPose.yaw;

            for (let i = 0; i < scanData.ranges.length; i++) {
                const range = scanData.ranges[i];

                // Skip invalid
                if (range < scanData.range_min || range > scanData.range_max) continue;
                if (range === 0) continue;

                const angle = angleMin + i * angleInc;
                // Point in Robot Frame
                // Lx = r * cos(angle)
                // Ly = r * sin(angle)

                const lx = range * Math.cos(angle);
                const ly = range * Math.sin(angle);

                // Transform to Map Frame (2D rotation + translation)
                // Mx = rx + (Lx * cos(yaw) - Ly * sin(yaw))
                // My = ry + (Lx * sin(yaw) + Ly * cos(yaw))

                const mx = rx + (lx * Math.cos(r_yaw) - ly * Math.sin(r_yaw));
                const my = ry + (lx * Math.sin(r_yaw) + ly * Math.cos(r_yaw));

                const canvasPt = worldToCanvas(mx, my);

                // Draw point
                ctx.beginPath();
                ctx.arc(canvasPt.x, canvasPt.y, 2, 0, 2 * Math.PI); // 2px radius
                ctx.fill();
            }
        }

        // --- Draw Robot ---
        if (robotPose) {
            const pt = worldToCanvas(robotPose.x, robotPose.y);

            // Draw Triangle
            ctx.save();
            ctx.translate(pt.x, pt.y);
            // Rotate: Canvas rotation also needs to account for Y flip?
            // Map Y increases Up. Canvas Y increases Down.
            // A positive yaw (CCW from X) in Map...
            // In Canvas, +angle is CW (usually).
            // And since Y is flipped, a CCW rotation in world might look different.
            // Let's think:
            // World: Yaw=0 -> East (X+). Yaw=90 -> North (Y+).
            // Canvas:
            // Start East (Right).
            // If Yaw=90 (World Y+):
            //   Canvas Y decreases.
            //   So we want to point UP.
            //   Canvas rotation of -90deg points Up.
            // So Canvas Rot = -World Yaw.

            ctx.rotate(-robotPose.yaw);

            ctx.fillStyle = '#3b82f6'; // blue-500
            ctx.beginPath();
            // Triangle pointing Right (East)
            ctx.moveTo(10, 0);   // Nose
            ctx.lineTo(-6, 6);   // Back Left
            ctx.lineTo(-6, -6);  // Back Right
            ctx.closePath();
            ctx.fill();

            ctx.restore();
        }

    }, [mapData, scanData, robotPose, showLidar]);

    return (
        <div className={`relative rounded-lg overflow-hidden bg-gray-100 ${className}`}>
            <canvas
                ref={canvasRef}
                width={600}
                height={600}
                className="w-full h-full object-contain"
            />

            {/* Legend / Status Overlay */}
            <div className="absolute top-2 left-2 bg-white/80 p-2 rounded text-xs pointer-events-none">
                <div>RESOLUTION: {mapData?.info.resolution.toFixed(3) ?? '-'} m/px</div>
                {robotPose && (
                    <div>
                        POSE: ({robotPose.x.toFixed(2)}, {robotPose.y.toFixed(2)})
                        θ: {(robotPose.yaw * 180 / Math.PI).toFixed(0)}°
                    </div>
                )}
            </div>
        </div>
    );
};
