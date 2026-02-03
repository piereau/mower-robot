"""
Headless robot description launch file.

Launches robot_state_publisher with the mower URDF.
Does not launch RViz or any visualization.

Usage:
    ros2 launch mower_description description.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('mower_description')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'mower.urdf.xacro')

    # Generate URDF from xacro - wrap in ParameterValue to treat as string
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    )

    return LaunchDescription([
        robot_state_publisher,
    ])

