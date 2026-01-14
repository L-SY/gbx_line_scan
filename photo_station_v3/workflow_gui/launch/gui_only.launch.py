#!/usr/bin/env python3
"""
Workflow GUI Only Launch File

This launch file starts only the GUI without any other nodes.
Use this when the other nodes (cameras, motors, lights) are already running.

Usage:
    ros2 launch workflow_gui gui_only.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    # Fix Qt plugin conflict between OpenCV and PyQt5
    # Force using system Qt plugins instead of OpenCV's bundled ones
    set_qt_plugin_path = SetEnvironmentVariable(
        name='QT_PLUGIN_PATH',
        value='/usr/lib/x86_64-linux-gnu/qt5/plugins'
    )
    
    # Clear any OpenCV Qt paths
    clear_cv2_qt = SetEnvironmentVariable(
        name='QT_QPA_PLATFORM_PLUGIN_PATH',
        value=''
    )
    
    workflow_gui_node = Node(
        package='workflow_gui',
        executable='workflow_gui',
        name='workflow_gui',
        output='screen',
        # Additional environment variables to prevent Qt conflicts
        additional_env={
            'QT_PLUGIN_PATH': '/usr/lib/x86_64-linux-gnu/qt5/plugins',
            'OPENCV_VIDEOIO_PRIORITY_GSTREAMER': '0',
        },
    )
    
    return LaunchDescription([
        set_qt_plugin_path,
        clear_cv2_qt,
        workflow_gui_node,
    ])
