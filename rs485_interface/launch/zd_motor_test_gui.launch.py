#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get the path to the executable
    package_name = 'rs485_interface'
    executable_name = 'zd_motor_test_gui'
    
    # Get the install directory
    install_dir = os.path.join(os.path.expanduser('~'), 'gbx_line_scan', 'install', package_name, 'lib', package_name)
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=[os.path.join(install_dir, executable_name)],
            output='screen',
            shell=True
        )
    ])

