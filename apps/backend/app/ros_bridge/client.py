"""
ROS 2 Bridge Client for FastAPI backend.

Provides async communication with the ROS 2 ws_bridge node via Unix domain socket.
Sends velocity commands and receives robot status updates.
"""

import asyncio
import json
import logging
import socket
from dataclasses import dataclass
from typing import Callable, Optional

logger = logging.getLogger(__name__)

# Socket configuration
SOCKET_PATH = '/tmp/mower_ros_bridge.sock'
RECONNECT_DELAY = 2.0  # seconds
BUFFER_SIZE = 4096


@dataclass
class RobotStatus:
    """Current robot status from ROS 2 bridge."""
    connected: bool = False
    watchdog: bool = False
    estop: bool = False
    battery_mv: int = 0
    error_flags: int = 0
    linear_vel: float = 0.0
    angular_vel: float = 0.0


class RosBridgeClient:
    """
    Async client for communicating with ROS 2 ws_bridge node.
    
    Uses Unix domain socket with JSON protocol:
    - Send: {"type": "velocity", "x": 0.5, "y": 0.3}
    - Send: {"type": "estop"}
    - Receive: {"type": "status", ...}
    """
    
    def __init__(self, socket_path: str = SOCKET_PATH):
        self.socket_path = socket_path
        self._socket: Optional[socket.socket] = None
        self._connected = False
        self._running = False
        self._read_task: Optional[asyncio.Task] = None
        self._status = RobotStatus()
        self._status_callbacks: list[Callable[[RobotStatus], None]] = []
        self._map_callbacks: list[Callable[[dict], None]] = []
        self._scan_callbacks: list[Callable[[dict], None]] = []
        self._pose_callbacks: list[Callable[[dict], None]] = []
        self._lock = asyncio.Lock()
    
    @property
    def connected(self) -> bool:
        """Whether the bridge is connected."""
        return self._connected
    
    @property
    def status(self) -> RobotStatus:
        """Current robot status."""
        return self._status
    
    def on_status(self, callback: Callable[[RobotStatus], None]) -> None:
        """Register a callback for status updates."""
        self._status_callbacks.append(callback)

    def on_map(self, callback: Callable[[dict], None]) -> None:
        """Register a callback for map updates."""
        self._map_callbacks.append(callback)

    def on_scan(self, callback: Callable[[dict], None]) -> None:
        """Register a callback for LiDAR scan updates."""
        self._scan_callbacks.append(callback)

    def on_pose(self, callback: Callable[[dict], None]) -> None:
        """Register a callback for robot pose updates."""
        self._pose_callbacks.append(callback)
    
    async def connect(self) -> bool:
        """Connect to the ROS 2 bridge socket."""
        if self._connected:
            return True
        
        try:
            self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self._socket.setblocking(False)
            
            loop = asyncio.get_event_loop()
            await loop.sock_connect(self._socket, self.socket_path)
            
            self._connected = True
            self._running = True
            self._read_task = asyncio.create_task(self._read_loop())
            
            logger.info(f"Connected to ROS 2 bridge at {self.socket_path}")
            return True
            
        except FileNotFoundError:
            logger.warning(f"ROS 2 bridge socket not found: {self.socket_path}")
            self._connected = False
            return False
        except ConnectionRefusedError:
            logger.warning("ROS 2 bridge connection refused")
            self._connected = False
            return False
        except Exception as e:
            logger.error(f"Failed to connect to ROS 2 bridge: {e}")
            self._connected = False
            return False
    
    async def disconnect(self) -> None:
        """Disconnect from the bridge."""
        self._running = False
        
        if self._read_task:
            self._read_task.cancel()
            try:
                await self._read_task
            except asyncio.CancelledError:
                pass
            self._read_task = None
        
        if self._socket:
            self._socket.close()
            self._socket = None
        
        self._connected = False
        logger.info("Disconnected from ROS 2 bridge")
    
    async def send_velocity(self, x: float, y: float) -> bool:
        """
        Send velocity command to the bridge.
        
        Args:
            x: Left/right value (-1 to 1), positive = right
            y: Forward/back value (-1 to 1), positive = forward
        
        Returns:
            True if sent successfully, False otherwise
        """
        if not self._connected:
            # Attempt to reconnect
            if not await self.connect():
                logger.debug("Cannot send velocity: bridge not connected")
                return False
        
        return await self._send_command({
            "type": "velocity",
            "x": float(x),
            "y": float(y)
        })
    
    async def send_estop(self) -> bool:
        """Activate emergency stop."""
        if not self._connected:
            if not await self.connect():
                logger.warning("Cannot send E-STOP: bridge not connected")
                return False
        
        logger.warning("Sending E-STOP command")
        return await self._send_command({"type": "estop"})
    
    async def send_release_estop(self) -> bool:
        """Release emergency stop."""
        if not self._connected:
            if not await self.connect():
                return False
        
        logger.info("Releasing E-STOP")
        return await self._send_command({"type": "release_estop"})
    
    async def _send_command(self, command: dict) -> bool:
        """Send a command to the bridge."""
        if not self._socket:
            return False
        
        try:
            message = json.dumps(command) + '\n'
            async with self._lock:
                loop = asyncio.get_event_loop()
                await loop.sock_sendall(self._socket, message.encode('utf-8'))
            return True
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            logger.error(f"Send failed, disconnecting: {e}")
            self._connected = False
            return False
    
    async def _read_loop(self) -> None:
        """Background task to read status updates from bridge.
        
        Note: The bridge only sends data when it has status to report
        (e.g., when /mower/status is published). Empty reads are normal.
        """
        buffer = ''
        
        while self._running and self._socket:
            try:
                loop = asyncio.get_event_loop()
                data = await asyncio.wait_for(
                    loop.sock_recv(self._socket, BUFFER_SIZE),
                    timeout=5.0  # Longer timeout since bridge may not always send data
                )
                
                if not data:
                    # Empty data on socket means actual EOF (remote closed)
                    logger.warning("Bridge connection closed by remote")
                    self._connected = False
                    break
                
                buffer += data.decode('utf-8')
                
                # Process complete messages
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        await self._process_message(line.strip())
                        
            except asyncio.TimeoutError:
                # No data received, but connection is still alive
                # This is normal when bridge has no status updates to send
                continue
            except asyncio.CancelledError:
                break
            except Exception as e:
                if self._running:
                    logger.error(f"Read error: {e}")
                    self._connected = False
                break
    
    async def _process_message(self, line: str) -> None:
        """Process a message from the bridge."""
        try:
            msg = json.loads(line)
            msg_type = msg.get('type')
            
            if msg_type == 'status':
                self._status.connected = msg.get('connected', False)
                self._status.watchdog = msg.get('watchdog', False)
                self._status.estop = msg.get('estop', False)
                self._status.battery_mv = msg.get('battery_mv', 0)
                self._status.error_flags = msg.get('error_flags', 0)
                
                for callback in self._status_callbacks:
                    try:
                        if asyncio.iscoroutinefunction(callback):
                            await callback(self._status)
                        else:
                            callback(self._status)
                    except Exception as e:
                        logger.error(f"Status callback error: {e}")
                        
            elif msg_type == 'odom':
                self._status.linear_vel = msg.get('linear', 0.0)
                self._status.angular_vel = msg.get('angular', 0.0)
            
            elif msg_type == 'map':
                for callback in self._map_callbacks:
                    try:
                        if asyncio.iscoroutinefunction(callback):
                            await callback(msg)
                        else:
                            callback(msg)
                    except Exception as e:
                        logger.error(f"Map callback error: {e}")

            elif msg_type == 'scan':
                for callback in self._scan_callbacks:
                    try:
                        if asyncio.iscoroutinefunction(callback):
                            await callback(msg)
                        else:
                            callback(msg)
                    except Exception as e:
                        logger.error(f"Scan callback error: {e}")

            elif msg_type == 'pose':
                for callback in self._pose_callbacks:
                    try:
                        if asyncio.iscoroutinefunction(callback):
                            await callback(msg)
                        else:
                            callback(msg)
                    except Exception as e:
                        logger.error(f"Pose callback error: {e}")
                
        except json.JSONDecodeError:
            logger.warning(f"Invalid JSON from bridge: {line[:50]}")


# Singleton instance
_bridge_client: Optional[RosBridgeClient] = None


def get_ros_bridge_client() -> RosBridgeClient:
    """Get the singleton ROS bridge client."""
    global _bridge_client
    if _bridge_client is None:
        _bridge_client = RosBridgeClient()
    return _bridge_client
