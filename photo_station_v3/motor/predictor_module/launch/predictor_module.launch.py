#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare launch arguments
    device_path_arg = DeclareLaunchArgument(
        'device_path',
        default_value='/dev/ttyACM0',
        description='Path to the RS485 serial device'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='19200',
        description='Baud rate for RS485 communication'
    )

    front_motor_id_arg = DeclareLaunchArgument(
        'front_motor_id',
        default_value='1',
        description='Slave ID for the front motor'
    )

    rear_motor_id_arg = DeclareLaunchArgument(
        'rear_motor_id',
        default_value='2',
        description='Slave ID for the rear motor'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Rate at which to publish motor velocities (Hz)'
    )

    # Create the node
    predictor_node = Node(
        package='predictor_module',
        executable='predictor_node',
        name='predictor_node',
        output='screen',
        parameters=[{
            'device_path': LaunchConfiguration('device_path'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'front_motor_id': LaunchConfiguration('front_motor_id'),
            'rear_motor_id': LaunchConfiguration('rear_motor_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        remappings=[
            # You can add remappings here if needed
            # ('/predictor/front_motor/cmd_vel', '/custom/front_cmd'),
        ]
    )

    return LaunchDescription([
        device_path_arg,
        baud_rate_arg,
        front_motor_id_arg,
        rear_motor_id_arg,
        publish_rate_arg,
        predictor_node,
    ])
