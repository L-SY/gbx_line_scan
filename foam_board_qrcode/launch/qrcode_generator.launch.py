#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch文件：启动发泡板二维码生成节点
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成launch描述"""
    
    # 声明launch参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('foam_board_qrcode'),
            'config',
            'qrcode_params.yaml'
        ]),
        description='配置文件路径'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/tmp/foam_board_qrcodes',
        description='二维码输出目录'
    )
    
    # 二维码生成节点
    qrcode_generator_node = Node(
        package='foam_board_qrcode',
        executable='qrcode_generator_node',
        name='qrcode_generator',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'output_directory': LaunchConfiguration('output_dir')
            }
        ],
        emulate_tty=True
    )
    
    return LaunchDescription([
        config_file_arg,
        output_dir_arg,
        qrcode_generator_node,
    ])

