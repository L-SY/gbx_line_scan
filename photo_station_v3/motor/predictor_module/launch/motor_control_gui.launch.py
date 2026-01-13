#!/usr/bin/env python3
"""
Launch file for Motor Control GUI

This launch file starts the motor control GUI for the predictor module.
The GUI allows controlling front and rear motors via ROS2 topics.

Usage:
    ros2 launch predictor_module motor_control_gui.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for motor control GUI"""
    
    motor_control_gui_node = Node(
        package='predictor_module',
        executable='motor_control_gui.py',
        name='motor_control_gui',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        motor_control_gui_node,
    ])
