"""
Robot description with RViz visualization launch file.

Launches robot_state_publisher and RViz for URDF visualization.

Usage:
    ros2 launch mower_description display.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('mower_description')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'mower.urdf.xacro')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'mower.rviz')

    # Launch arguments
    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

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

    # Joint State Publisher (for joints without hardware - if any movable joints)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        use_rviz,
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])
