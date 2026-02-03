#!/usr/bin/env python3
"""Launch file for LD06 LiDAR on GPIO UART.

This launch file configures the ldlidar_ros2 driver for the LD06 LiDAR
connected via RPi GPIO serial (/dev/serial0).

Story: 2.1 LiDAR Integration
AC: 1, 2 - Publishes sensor_msgs/LaserScan to /scan at 8-15Hz

Note: TF transform base_link -> laser_frame is now provided by
      robot_state_publisher via URDF (mower_description package).
      See: Story 2.2 Robot Description & TF
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for LD06 LiDAR."""

    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/serial0',  # GPIO UART on RPi
        description='Serial port for LiDAR (GPIO: /dev/serial0, USB: /dev/ttyUSB0)'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',  # Matches URDF convention
        description='TF frame ID for laser data'
    )

    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='/scan',  # Standard Nav2 topic
        description='Topic name for LaserScan messages'
    )

    # LD06 LiDAR driver node
    ldlidar_node = Node(
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        name='ldlidar_node',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD06',
            'laser_scan_topic_name': LaunchConfiguration('topic_name'),
            'point_cloud_2d_topic_name': 'pointcloud2d',
            'frame_id': LaunchConfiguration('frame_id'),
            'port_name': LaunchConfiguration('serial_port'),
            'serial_baudrate': 230400,  # LD06 fixed baud rate
            'laser_scan_dir': True,  # Counterclockwise
            'enable_angle_crop_func': False,
            'angle_crop_min': 0.0,
            'angle_crop_max': 360.0,
            'range_min': 0.02,  # 2cm min range
            'range_max': 12.0,  # 12m max range
        }]
    )

    # Note: Static TF (base_link -> laser_frame) removed in Story 2.2
    # TF now published by robot_state_publisher from URDF

    return LaunchDescription([
        serial_port_arg,
        frame_id_arg,
        topic_name_arg,
        ldlidar_node,
    ])

