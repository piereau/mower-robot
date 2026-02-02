#!/usr/bin/env python3
"""Launch file for teleoperation bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for teleop bridge."""

    # Declare arguments
    socket_path_arg = DeclareLaunchArgument(
        'socket_path',
        default_value='/tmp/mower_ros_bridge.sock',
        description='Unix socket path for FastAPI communication'
    )

    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='1.0',
        description='Maximum linear velocity (m/s)'
    )

    max_angular_vel_arg = DeclareLaunchArgument(
        'max_angular_vel',
        default_value='8.0',
        description='Maximum angular velocity (rad/s)'
    )

    cmd_vel_rate_arg = DeclareLaunchArgument(
        'cmd_vel_rate',
        default_value='50',
        description='Rate to publish /cmd_vel (Hz)'
    )

    # Teleop bridge node
    teleop_bridge_node = Node(
        package='mower_teleop',
        executable='ws_bridge',
        name='ws_bridge',
        output='screen',
        parameters=[{
            'socket_path': LaunchConfiguration('socket_path'),
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'max_angular_vel': LaunchConfiguration('max_angular_vel'),
            'cmd_vel_rate': LaunchConfiguration('cmd_vel_rate'),
        }],
    )

    return LaunchDescription([
        socket_path_arg,
        max_linear_vel_arg,
        max_angular_vel_arg,
        cmd_vel_rate_arg,
        teleop_bridge_node,
    ])
