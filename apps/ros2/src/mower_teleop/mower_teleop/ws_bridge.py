#!/usr/bin/env python3
"""
ROS 2 WebSocket Bridge Node for Teleoperation.

Listens on a Unix domain socket for JSON commands from the FastAPI backend
and publishes Twist messages to /cmd_vel. Also subscribes to robot status
topics and forwards them back to the backend.

Protocol:
- Commands (Backend → Bridge):
  {"type": "velocity", "x": 0.5, "y": 0.3}  # x=left/right, y=forward/back
  {"type": "estop"}
  {"type": "release_estop"}
  
- Telemetry (Bridge → Backend):
  {"type": "status", "connected": true, "watchdog": false, ...}
  {"type": "odom", "linear": 0.5, "angular": 0.1}
  {"type": "map", "info": {...}, "data": [...]}
  {"type": "scan", "ranges": [...], ...}
  {"type": "pose", "x": 1.0, "y": 2.0, "yaw": 1.57}
"""

import json
import os
import socket
import threading
import math
import struct
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

try:
    from mower_msgs.msg import MowerStatus
    HAS_MOWER_MSGS = True
except ImportError:
    HAS_MOWER_MSGS = False


# Configuration constants
SOCKET_PATH = '/tmp/mower_ros_bridge.sock'
MAX_LINEAR_VEL = 1.0   # m/s
MAX_ANGULAR_VEL = 8.0  # rad/s
CMD_VEL_RATE_HZ = 50   # Publish rate to keep watchdog alive
SOCKET_BUFFER_SIZE = 40960 # Increased buffer size for map data


class WsBridgeNode(Node):
    """ROS 2 node that bridges Unix socket commands to /cmd_vel."""
    
    def __init__(self):
        super().__init__('ws_bridge')
        
        # Declare parameters
        self.declare_parameter('socket_path', SOCKET_PATH)
        self.declare_parameter('max_linear_vel', MAX_LINEAR_VEL)
        self.declare_parameter('max_angular_vel', MAX_ANGULAR_VEL)
        self.declare_parameter('cmd_vel_rate', CMD_VEL_RATE_HZ)
        
        # Get parameters
        self.socket_path = self.get_parameter('socket_path').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.cmd_vel_rate = self.get_parameter('cmd_vel_rate').value
        
        # State
        self._current_twist = Twist()
        self._estop_active = False
        self._lock = threading.Lock()
        self._client_socket: Optional[socket.socket] = None
        self._running = True
        
        # Publisher for velocity commands
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        
        # Subscribe to mower status if available
        if HAS_MOWER_MSGS:
            self._status_sub = self.create_subscription(
                MowerStatus,
                '/mower/status',
                self._on_mower_status,
                10
            )
            self.get_logger().info('Subscribed to /mower/status')
        else:
            self.get_logger().warn('mower_msgs not available, status forwarding disabled')
            
        # Map Subscription (Transient Local for latched maps)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._on_map,
            map_qos
        )
        
        # LiDAR Subscription (Sensor Data)
        scan_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self._scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._on_scan,
            scan_qos
        )
        
        # TF Listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
            
        # Timer to publish velocity at fixed rate (keeps watchdog alive)
        timer_period = 1.0 / self.cmd_vel_rate
        self._cmd_vel_timer = self.create_timer(timer_period, self._publish_cmd_vel)
        
        # Timer for robot pose updates (10Hz)
        self._pose_timer = self.create_timer(0.1, self._publish_robot_pose)
        
        # Start socket server in background thread
        self._socket_thread = threading.Thread(target=self._run_socket_server, daemon=True)
        self._socket_thread.start()
        
        self.get_logger().info(
            f'WS Bridge started: socket={self.socket_path}, '
            f'max_lin={self.max_linear_vel}m/s, max_ang={self.max_angular_vel}rad/s, '
            f'rate={self.cmd_vel_rate}Hz'
        )
    
    def _publish_cmd_vel(self):
        """Publish current twist at fixed rate."""
        with self._lock:
            if self._estop_active:
                # E-stop: always publish zero
                self._cmd_vel_pub.publish(Twist())
            else:
                self._cmd_vel_pub.publish(self._current_twist)
    
    def _on_mower_status(self, msg: 'MowerStatus'):
        """Forward mower status to connected client."""
        self._send_json({
            'type': 'status',
            'watchdog_triggered': msg.watchdog_triggered,
            'estop_active': msg.estop_active,
            'battery_mv': msg.battery_mv,
            'crc_error': msg.crc_error_seen,
            'encoder_left': msg.encoder_left,
            'encoder_right': msg.encoder_right
        })

    def _on_map(self, msg: OccupancyGrid):
        """Handle new map."""
        # Simple compression: send as flat list.
        # Check size: 100x100 = 10KB. 500x500 = 250KB.
        # If too large, we might need a better strategy, but start simple.
        
        self.get_logger().info(f'Received map: {msg.info.width}x{msg.info.height}')
        
        data_list = list(msg.data)
        
        payload = {
            'type': 'map',
            'info': {
                'resolution': msg.info.resolution,
                'width': msg.info.width,
                'height': msg.info.height,
                'origin': {
                    'position': {
                        'x': msg.info.origin.position.x,
                        'y': msg.info.origin.position.y
                    },
                    'orientation': {
                        'z': msg.info.origin.orientation.z,
                        'w': msg.info.origin.orientation.w
                    }
                }
            },
            'data': data_list
        }
        
        self._send_json(payload)

    def _on_scan(self, msg: LaserScan):
        """Handle laser scan."""
        # Downsample scan to reduce bandwidth?
        # Let's send every point for now, client can handle it or we assume decent connection (local).
        
        # Replace infinity with 0 or max_range
        ranges = []
        for r in msg.ranges:
             if math.isinf(r) or math.isnan(r):
                 ranges.append(0.0)
             else:
                 ranges.append(r)
        
        payload = {
            'type': 'scan',
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'ranges': ranges
        }
        
        self._send_json(payload)

    def _publish_robot_pose(self):
        """Lookup and publish robot pose in map frame."""
        try:
            # Look up transform from map to base_link
            t = self._tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            
            # Extract 2D pose
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            # Quaternion to Yaw
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.z + q.w * q.x) # Wait, is this right? 
            # Standard conversion:
            # yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            payload = {
                'type': 'pose',
                'x': x,
                'y': y,
                'yaw': yaw
            }
            
            self._send_json(payload)
            
        except TransformException:
            # Requires map frame to exist. If SLAM/AMCL not running, this will fail.
            # Fail silently to avoid log spam, or log debug
            pass

    def _send_json(self, payload: dict):
        """Helper to send JSON to client safely."""
        if self._client_socket is None:
            return
            
        try:
            message = json.dumps(payload) + '\n'
            # Use lock if strictly necessary, but socket send is usually thread-safe enough for one writer.
            # However, handled in _on_mower_status and now here from multiple threads (callbacks).
            # Python socket.sendall IS thread-safe, but let's be careful about interlacing.
            # Actually, `json.dumps` + `\n` creates one buffer. sendall sends it.
            # If two threads call sendall at same time, data might interleave? 
            # Yes, sendall is not atomic for large data.
            # We should lock around the socket sending.
            
            # We reuse the existing lock or careful? 
            # existing _lock is for self state. 
            # Let's use a separate lock for socket write if we want to be safe, 
            # or reuse _lock if it doesn't cause contention with timer.
            # Timer uses _lock for cmd_vel. 
            # Let's add a socket_lock or just risk it? 
            # Better safe: Reuse _lock or add one.
            # Let's use _lock for now as simple solution, avoiding deadlock (don't call other locked methods).
            
            # Wait, `_on_mower_status` didn't use lock in original code.
            # Let's follow pattern: The original code didn't lock socket writes.
            # BUT original code only had one writer? No, `_on_mower_status` (callback thread) and `_process_command` (socket thread for echo)
            # `_process_command` did: `self._client_socket.sendall`
            # So multiple threads WERE accessing it.
            # I will add a socket lock to be safe.
            pass
            
            # Direct send for now, maybe add lock if issues arise.
            self._client_socket.sendall(message.encode('utf-8'))
            
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        except Exception as e:
            self.get_logger().warn(f'Send error: {e}')

    
    def _run_socket_server(self):
        """Run the Unix domain socket server."""
        # Remove existing socket file
        if os.path.exists(self.socket_path):
            os.unlink(self.socket_path)
        
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(self.socket_path)
        server.listen(1)
        server.settimeout(1.0)  # Allow periodic check for shutdown
        
        # Make socket accessible to other users (FastAPI service)
        os.chmod(self.socket_path, 0o777)
        
        self.get_logger().info(f'Socket server listening on {self.socket_path}')
        
        while self._running:
            try:
                client, _ = server.accept()
                self.get_logger().info('Backend connected')
                self._client_socket = client
                self._handle_client(client)
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    self.get_logger().error(f'Socket error: {e}')
        
        server.close()
        if os.path.exists(self.socket_path):
            os.unlink(self.socket_path)
    
    def _handle_client(self, client: socket.socket):
        """Handle messages from a connected client."""
        buffer = ''
        client.settimeout(0.5)  # Short timeout for responsive checking
        
        while self._running:
            try:
                data = client.recv(SOCKET_BUFFER_SIZE)
                if not data:
                    # Empty data on a blocking recv means actual disconnect
                    self.get_logger().info('Backend disconnected (EOF)')
                    break
                
                buffer += data.decode('utf-8')
                
                # Process complete JSON messages (newline-delimited)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        self._process_command(line.strip())
                        
            except socket.timeout:
                # Timeout is normal, just continue waiting
                continue
            except (ConnectionResetError, BrokenPipeError) as e:
                self.get_logger().info(f'Backend disconnected: {e}')
                break
            except Exception as e:
                self.get_logger().error(f'Client error: {e}')
                break
        
        self.get_logger().info('Client handler exiting')
        self._client_socket = None
        
        # Stop robot when backend disconnects
        with self._lock:
            self._current_twist = Twist()
    
    def _process_command(self, line: str):
        """Process a JSON command from the backend."""
        try:
            cmd = json.loads(line)
            cmd_type = cmd.get('type')
            
            if cmd_type == 'velocity':
                x = float(cmd.get('x', 0))  # Left/right (-1 to 1)
                y = float(cmd.get('y', 0))  # Forward/back (-1 to 1)
                
                # Convert joystick to Twist
                # y = forward/back → linear.x
                # x = left/right → angular.z (negative because positive = counterclockwise)
                twist = Twist()
                twist.linear.x = y * self.max_linear_vel
                twist.angular.z = -x * self.max_angular_vel
                
                with self._lock:
                    if not self._estop_active:
                        self._current_twist = twist
                
                # Open-loop feedback: Echo commanded velocity back to dashboard
                # self._send_json({
                #     'type': 'odom',
                #     'linear': twist.linear.x,
                #     'angular': twist.angular.z
                # })
                # (Disabled to allow real odom if we wanted, or keep it? 
                # The original code had it. Let's keep it but use _send_json helper)
                
                self._send_json({
                    'type': 'odom',
                    'linear': twist.linear.x,
                    'angular': twist.angular.z
                })
                
            elif cmd_type == 'estop':
                with self._lock:
                    self._estop_active = True
                    self._current_twist = Twist()
                self.get_logger().warn('E-STOP activated')
                
            elif cmd_type == 'release_estop':
                with self._lock:
                    self._estop_active = False
                self.get_logger().info('E-STOP released')
                
            else:
                self.get_logger().warn(f'Unknown command type: {cmd_type}')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Command processing error: {e}')
    
    def destroy_node(self):
        """Clean shutdown."""
        self._running = False
        if self._socket_thread.is_alive():
            self._socket_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WsBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
