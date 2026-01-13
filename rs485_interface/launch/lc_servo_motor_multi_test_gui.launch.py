#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the path to the executable
    package_name = 'rs485_interface'
    executable_name = 'lc_servo_motor_multi_test_gui'
    
    # Try to find the executable in install directory first
    install_dir = os.path.join(os.path.expanduser('~'), 'gbx_line_scan', 'install', package_name, 'lib', package_name)
    executable_path = os.path.join(install_dir, executable_name)
    
    # If not found in install, try to find in build directory (for development)
    if not os.path.exists(executable_path):
        build_dir = os.path.join(os.path.expanduser('~'), 'gbx_line_scan', 'build', package_name)
        executable_path = os.path.join(build_dir, executable_name)
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=[executable_path],
            output='screen',
            shell=True
        )
    ])



