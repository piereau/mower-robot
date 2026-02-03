#!/usr/bin/env python3
"""Main robot bringup launch file.

Launches the complete mower robot stack including:
- Robot description (URDF -> TF via robot_state_publisher)
- Serial bridge (motor control, odometry, telemetry)
- LiDAR driver
- Teleop nodes (optional)

Story: 2.2 Robot Description & TF
Usage:
    ros2 launch mower_bringup robot.launch.py
    ros2 launch mower_bringup robot.launch.py use_lidar:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for full robot stack."""

    # =========================================================================
    # Launch Arguments
    # =========================================================================
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Launch LiDAR driver'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B001X9Y7-if00-port0',
        description='Serial port for motor controller'
    )

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/serial0',
        description='Serial port for LiDAR'
    )

    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Launch SLAM Toolbox for mapping'
    )

    # =========================================================================
    # Robot Description (URDF -> TF)
    # =========================================================================
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mower_description'),
                'launch',
                'description.launch.py'
            ])
        ])
    )

    # =========================================================================
    # LiDAR Driver
    # =========================================================================
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mower_bringup'),
                'launch',
                'lidar.launch.py'
            ])
        ]),
        launch_arguments={
            'serial_port': LaunchConfiguration('lidar_port'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_lidar'))
    )

    # =========================================================================
    # SLAM (Mapping)
    # =========================================================================
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mower_localization'),
                'launch',
                'slam.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('use_slam'))
    )

    # =========================================================================
    # Serial Bridge (Motor Control + Odometry + Telemetry)
    # =========================================================================
    serial_bridge_node = Node(
        package='mower_hardware',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 115200,
        }]
    )

    return LaunchDescription([
        # Arguments
        use_lidar_arg,
        use_slam_arg,
        serial_port_arg,
        lidar_port_arg,

        # Launch includes
        description_launch,
        lidar_launch,
        slam_launch,

        # Nodes
        serial_bridge_node,
    ])
