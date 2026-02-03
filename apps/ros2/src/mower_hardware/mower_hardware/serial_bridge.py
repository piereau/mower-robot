#!/usr/bin/env python3
"""
ROS 2 Serial Bridge Node for Mower Robot.

Bridges serial communication between Arduino and ROS 2 topics:
- Subscribes to /cmd_vel and sends velocity commands to Arduino
- Publishes telemetry to /odom and /mower/status
- Publishes connection health to /mower/serial_status
"""

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    serial = None

from mower_msgs.msg import MowerStatus, SerialStatus
from mower_hardware.protocol import (
    PacketParser,
    TelemetryPacket,
    build_velocity_packet,
    build_estop_packet,
)


# Default parameters
DEFAULT_SERIAL_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD_RATE = 115200
DEFAULT_CMD_RATE = 50.0  # Hz
DEFAULT_STATUS_RATE = 1.0  # Hz
DEFAULT_CONNECTION_TIMEOUT = 1.0  # seconds

# Wheel odometry parameters (calibrate with real hardware!)
WHEEL_SEPARATION = 0.3  # meters (track width)
TICKS_PER_METER = 1000.0  # encoder ticks per meter traveled

# Reconnection backoff
MIN_RECONNECT_DELAY = 1.0  # seconds
MAX_RECONNECT_DELAY = 30.0  # seconds


class SerialBridge(Node):
    """ROS 2 node bridging serial communication with Arduino."""

    def __init__(self):
        super().__init__('serial_bridge')

        # Declare parameters
        self.declare_parameter('serial_port', DEFAULT_SERIAL_PORT)
        self.declare_parameter('baud_rate', DEFAULT_BAUD_RATE)
        self.declare_parameter('cmd_rate', DEFAULT_CMD_RATE)
        self.declare_parameter('status_rate', DEFAULT_STATUS_RATE)
        self.declare_parameter('connection_timeout', DEFAULT_CONNECTION_TIMEOUT)
        self.declare_parameter('wheel_separation', WHEEL_SEPARATION)
        self.declare_parameter('ticks_per_meter', TICKS_PER_METER)

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.cmd_rate = self.get_parameter('cmd_rate').get_parameter_value().double_value
        self.status_rate = self.get_parameter('status_rate').get_parameter_value().double_value
        self.connection_timeout = self.get_parameter('connection_timeout').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.ticks_per_meter = self.get_parameter('ticks_per_meter').get_parameter_value().double_value
        
        # Safety: ensure ticks_per_meter is never zero (would cause division by zero)
        if self.ticks_per_meter <= 0:
            self.get_logger().warn(f'Invalid ticks_per_meter ({self.ticks_per_meter}), using default {TICKS_PER_METER}')
            self.ticks_per_meter = float(TICKS_PER_METER)

        # Serial port state
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        self.reconnect_delay = MIN_RECONNECT_DELAY
        self.serial_lock = threading.Lock()

        # Protocol state
        self.parser = PacketParser()
        self.sequence = 0
        self.last_cmd_vel: Optional[Twist] = None
        self.cmd_vel_lock = threading.Lock()

        # Statistics
        self.packets_received = 0
        self.packets_sent = 0
        self.crc_errors = 0
        self.last_packet_time: Optional[float] = None

        # Odometry state
        self.last_encoder_left: Optional[int] = None
        self.last_encoder_right: Optional[int] = None
        self.last_odom_time: Optional[Time] = None
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

        # Create publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.status_pub = self.create_publisher(MowerStatus, '/mower/status', 10)
        self.serial_status_pub = self.create_publisher(SerialStatus, '/mower/serial_status', 10)

        # Create TF broadcaster for odom -> base_link transform
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create timers
        cmd_period = 1.0 / self.cmd_rate
        self.cmd_timer = self.create_timer(cmd_period, self.cmd_timer_callback)

        status_period = 1.0 / self.status_rate
        self.status_timer = self.create_timer(status_period, self.status_timer_callback)

        # Start serial reader thread
        self.reader_thread_running = True
        self.reader_thread = threading.Thread(target=self.serial_reader_thread, daemon=True)
        self.reader_thread.start()

        self.get_logger().info(f'Serial bridge initialized: {self.serial_port} @ {self.baud_rate} baud')

    def destroy_node(self):
        """Clean shutdown."""
        self.reader_thread_running = False
        with self.serial_lock:
            if self.serial and self.serial.is_open:
                self.serial.close()
        super().destroy_node()

    def cmd_vel_callback(self, msg: Twist):
        """Store latest velocity command."""
        with self.cmd_vel_lock:
            self.last_cmd_vel = msg

    def cmd_timer_callback(self):
        """Send velocity command at fixed rate."""
        with self.cmd_vel_lock:
            cmd = self.last_cmd_vel

        if not self.connected:
            return

        # Build and send packet
        if cmd is not None:
            packet = build_velocity_packet(cmd.linear.x, cmd.angular.z, self.sequence)
        else:
            # Send zero velocity when no command received
            packet = build_velocity_packet(0.0, 0.0, self.sequence)

        self.sequence = (self.sequence + 1) & 0xFF

        with self.serial_lock:
            if self.serial and self.serial.is_open:
                try:
                    self.serial.write(packet)
                    self.packets_sent += 1
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial write error: {e}')
                    self.connected = False

    def status_timer_callback(self):
        """Publish serial status at fixed rate."""
        now = self.get_clock().now()

        # Check for connection timeout
        if self.last_packet_time is not None:
            time_since_packet = time.time() - self.last_packet_time
            if time_since_packet > self.connection_timeout:
                if self.connected:
                    self.get_logger().warn('Connection timeout - no packets received')
                    self.connected = False

        # Publish serial status
        status = SerialStatus()
        status.connected = self.connected
        status.last_packet_time = self.last_packet_time if self.last_packet_time else 0.0
        status.packets_received = self.packets_received
        status.packets_sent = self.packets_sent
        status.crc_errors = self.crc_errors + self.parser.crc_error_count

        self.serial_status_pub.publish(status)

    def serial_reader_thread(self):
        """Background thread for non-blocking serial I/O."""
        while self.reader_thread_running:
            if not self.connected:
                self._try_connect()
                if not self.connected:
                    time.sleep(self.reconnect_delay)
                    # Exponential backoff
                    self.reconnect_delay = min(self.reconnect_delay * 2, MAX_RECONNECT_DELAY)
                    continue
                else:
                    # Reset backoff on successful connection
                    self.reconnect_delay = MIN_RECONNECT_DELAY

            # Read available data
            try:
                with self.serial_lock:
                    if self.serial and self.serial.is_open and self.serial.in_waiting > 0:
                        data = self.serial.read(self.serial.in_waiting)
                    else:
                        data = None
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                self.connected = False
                continue

            if data:
                packets = self.parser.feed(data)
                for pkt in packets:
                    self._process_telemetry(pkt)

            time.sleep(0.001)  # 1ms polling interval

    def _try_connect(self):
        """Try to open serial port."""
        if not SERIAL_AVAILABLE:
            self.get_logger().error('pyserial not installed - cannot connect')
            return

        try:
            self.get_logger().info(f'Connecting to {self.serial_port}...')
            with self.serial_lock:
                self.serial = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    timeout=0.1
                )
            self.connected = True
            self.get_logger().info(f'Connected to {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().warn(f'Failed to connect: {e}')
            self.connected = False

    def _process_telemetry(self, pkt: TelemetryPacket):
        """Process received telemetry packet."""
        now = self.get_clock().now()
        self.last_packet_time = time.time()
        self.packets_received += 1

        # Publish MowerStatus
        status = MowerStatus()
        status.encoder_left = pkt.encoder_left
        status.encoder_right = pkt.encoder_right
        status.watchdog_triggered = pkt.watchdog_triggered
        status.estop_active = pkt.estop_active
        status.crc_error_seen = pkt.crc_error_seen
        status.battery_mv = pkt.battery_mv
        status.sequence = pkt.sequence
        self.status_pub.publish(status)

        # Calculate and publish odometry
        self._update_odometry(pkt, now)

    def _update_odometry(self, pkt: TelemetryPacket, now: Time):
        """Calculate wheel odometry from encoder ticks."""
        # First packet - just store values
        if self.last_encoder_left is None:
            self.last_encoder_left = pkt.encoder_left
            self.last_encoder_right = pkt.encoder_right
            self.last_odom_time = now
            return

        # Calculate deltas
        delta_left = pkt.encoder_left - self.last_encoder_left
        delta_right = pkt.encoder_right - self.last_encoder_right

        # Handle encoder overflow (32-bit signed)
        if abs(delta_left) > 2147483647 / 2:
            delta_left = 0
        if abs(delta_right) > 2147483647 / 2:
            delta_right = 0

        # Time delta
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 0.02  # Default to 50Hz if time is weird

        # Convert ticks to meters
        left_dist = delta_left / self.ticks_per_meter
        right_dist = delta_right / self.ticks_per_meter

        # Differential drive kinematic model
        linear_dist = (left_dist + right_dist) / 2.0
        angular_dist = (right_dist - left_dist) / self.wheel_separation

        # Update pose
        if abs(angular_dist) < 1e-6:
            # Straight line motion
            self.odom_x += linear_dist * math.cos(self.odom_theta)
            self.odom_y += linear_dist * math.sin(self.odom_theta)
        else:
            # Arc motion
            radius = linear_dist / angular_dist
            self.odom_x += radius * (math.sin(self.odom_theta + angular_dist) - math.sin(self.odom_theta))
            self.odom_y += radius * (math.cos(self.odom_theta) - math.cos(self.odom_theta + angular_dist))
            self.odom_theta += angular_dist

        # Normalize theta to [-pi, pi]
        self.odom_theta = math.atan2(math.sin(self.odom_theta), math.cos(self.odom_theta))

        # Calculate velocities
        linear_vel = linear_dist / dt
        angular_vel = angular_dist / dt

        # Create and publish Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.odom_theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.odom_theta / 2.0)

        # Velocity
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel

        # Covariance (simple diagonal estimate)
        # Position covariance
        odom.pose.covariance[0] = 0.01  # x
        odom.pose.covariance[7] = 0.01  # y
        odom.pose.covariance[35] = 0.01  # yaw
        # Velocity covariance
        odom.twist.covariance[0] = 0.01  # linear x
        odom.twist.covariance[35] = 0.01  # angular z

        self.odom_pub.publish(odom)

        # Publish odom -> base_link transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # Store for next iteration
        self.last_encoder_left = pkt.encoder_left
        self.last_encoder_right = pkt.encoder_right
        self.last_odom_time = now


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)

    if not SERIAL_AVAILABLE:
        print('ERROR: pyserial not installed. Install with: pip install pyserial')
        return 1

    node = SerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    main()
