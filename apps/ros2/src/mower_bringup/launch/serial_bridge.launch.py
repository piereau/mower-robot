#!/usr/bin/env python3
"""Launch file for serial bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for serial bridge."""

    # Declare arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B001X9Y7-if00-port0',
        description='Serial port for Arduino connection'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )

    use_params_file_arg = DeclareLaunchArgument(
        'use_params_file',
        default_value='true',
        description='Whether to use the params file'
    )

    # Path to params file
    params_file = PathJoinSubstitution([
        FindPackageShare('mower_bringup'),
        'config',
        'serial_bridge_params.yaml'
    ])

    # Serial bridge node
    serial_bridge_node = Node(
        package='mower_hardware',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[
            params_file,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
            }
        ],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        use_params_file_arg,
        serial_bridge_node,
    ])
