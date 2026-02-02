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
"""

import json
import os
import socket
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist

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
SOCKET_BUFFER_SIZE = 4096


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
        
        # Timer to publish velocity at fixed rate (keeps watchdog alive)
        timer_period = 1.0 / self.cmd_vel_rate
        self._cmd_vel_timer = self.create_timer(timer_period, self._publish_cmd_vel)
        
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
        if self._client_socket is None:
            return
        
        try:
            status_json = json.dumps({
                'type': 'status',
                'watchdog_triggered': msg.watchdog_triggered,
                'estop_active': msg.estop_active,
                'battery_mv': msg.battery_mv,
                'crc_error': msg.crc_error_seen,
                'encoder_left': msg.encoder_left,
                'encoder_right': msg.encoder_right
            }) + '\n'
            self._client_socket.sendall(status_json.encode('utf-8'))
        except (BrokenPipeError, ConnectionResetError, OSError):
            # Client disconnected, ignore
            pass
    
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
                
                self.get_logger().debug(
                    f'Velocity: x={x:.2f}, y={y:.2f} → '
                    f'lin={twist.linear.x:.2f}, ang={twist.angular.z:.2f}'
                )
                
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
