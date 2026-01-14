#!/usr/bin/env python3
"""
Complete Workflow GUI Launch File

This launch file starts all necessary components for the photo station workflow:
1. Predictor Module (Motor Control)
2. Light Controller
3. Dual Camera with Stitching
4. Workflow GUI

Usage:
    ros2 launch workflow_gui workflow_gui.launch.py

Optional arguments:
    gui_only:=true          - Launch only the GUI (assumes other nodes are running)
    skip_cameras:=true      - Skip camera launch (for testing without cameras)
    skip_motors:=true       - Skip motor launch
    skip_lights:=true       - Skip light controller launch
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
    OpaqueFunction
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
        description='Launch only the GUI without other nodes'
    )
    
    skip_cameras_arg = DeclareLaunchArgument(
        'skip_cameras',
        default_value='false',
        description='Skip camera launch'
    )
    
    skip_motors_arg = DeclareLaunchArgument(
        'skip_motors',
        default_value='false',
        description='Skip motor controller launch'
    )
    
    skip_lights_arg = DeclareLaunchArgument(
        'skip_lights',
        default_value='false',
        description='Skip light controller launch'
    )
    
    # Motor parameters
    motor_device_arg = DeclareLaunchArgument(
        'motor_device',
        default_value='/dev/ttyACM0',
        description='RS485 device path for motor controller'
    )
    
    # Light controller parameters
    light_ip_arg = DeclareLaunchArgument(
        'light_ip',
        default_value='192.168.3.101',
        description='IP address of the light controller'
    )
    
    # ==================== GUI Node ====================
    workflow_gui_node = Node(
        package='workflow_gui',
        executable='workflow_gui',
        name='workflow_gui',
        output='screen',
    )
    
    # Delay GUI start to allow other nodes to initialize
    delayed_gui = TimerAction(
        period=5.0,
        actions=[workflow_gui_node]
    )
    
    # ==================== Include Other Launch Files ====================
    
    # Predictor Module (Motor Control)
    predictor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('predictor_module'),
                'launch',
                'predictor_module.launch.py'
            ])
        ]),
        launch_arguments={
            'device_path': LaunchConfiguration('motor_device'),
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('skip_motors'))
    )
    
    # Light Controller
    light_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('hk_light_controller'),
                'launch',
                'hk_light_controller.launch.py'
            ])
        ]),
        launch_arguments={
            'ip_address': LaunchConfiguration('light_ip'),
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('skip_lights'))
    )
    
    # Dual Camera with Stitching
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('hk_line_camera'),
                'launch',
                'dual_camera_with_stitching.launch.py'
            ])
        ]),
        condition=UnlessCondition(LaunchConfiguration('skip_cameras'))
    )
    
    # ==================== Full Launch Description ====================
    return LaunchDescription([
        # Arguments
        gui_only_arg,
        skip_cameras_arg,
        skip_motors_arg,
        skip_lights_arg,
        motor_device_arg,
        light_ip_arg,
        
        # Launch motor controller (immediate)
        GroupAction(
            actions=[predictor_launch],
            condition=UnlessCondition(LaunchConfiguration('gui_only'))
        ),
        
        # Launch light controller (immediate)
        GroupAction(
            actions=[light_controller_launch],
            condition=UnlessCondition(LaunchConfiguration('gui_only'))
        ),
        
        # Launch cameras (immediate)
        GroupAction(
            actions=[camera_launch],
            condition=UnlessCondition(LaunchConfiguration('gui_only'))
        ),
        
        # Launch GUI (delayed by 5 seconds to allow other nodes to initialize)
        delayed_gui,
    ])
