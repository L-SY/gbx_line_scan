#!/usr/bin/env python3
"""
Transport Workflow GUI Launch File

This launch file starts:
1. Transport Logic Node (if not gui_only)
2. Transport Workflow GUI

Usage:
    ros2 launch transport_workflow transport_workflow.launch.py

Optional arguments:
    gui_only:=true          - Launch only the GUI (assumes transport_logic_node is running)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # ==================== Launch Arguments ====================
    gui_only_arg = DeclareLaunchArgument(
        'gui_only',
        default_value='false',
        description='Launch only the GUI without transport_logic_node'
    )
    
    # ==================== GUI Node ====================
    transport_workflow_gui_node = Node(
        package='transport_workflow',
        executable='transport_workflow',
        name='transport_workflow',
        output='screen',
    )
    
    # Delay GUI start to allow other nodes to initialize
    delayed_gui = TimerAction(
        period=2.0,
        actions=[transport_workflow_gui_node]
    )
    
    # ==================== Include Transport Logic Launch ====================
    transport_logic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('transport_logic'),
                'launch',
                'transport_logic.launch.py'
            ])
        ]),
        condition=UnlessCondition(LaunchConfiguration('gui_only'))
    )
    
    # ==================== Full Launch Description ====================
    return LaunchDescription([
        # Arguments
        gui_only_arg,
        
        # Launch transport logic (includes distance sensor now)
        GroupAction(
            actions=[transport_logic_launch],
            condition=UnlessCondition(LaunchConfiguration('gui_only'))
        ),
        
        # Launch GUI (delayed by 2 seconds to allow other nodes to initialize)
        delayed_gui,
    ])
