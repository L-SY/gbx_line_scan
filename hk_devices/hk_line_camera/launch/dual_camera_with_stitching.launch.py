"""
双相机带拼接启动文件 (Dual Camera with Stitching Launch File)

启动两个独立的线扫相机节点和两个拼接节点：
- 前相机 (Front): 192.168.1.101
  - 原始图像: camera_front/front/image_raw
  - 拼接图像: camera_front/front/image_stitched
- 后相机 (Rear): 192.168.2.101
  - 原始图像: camera_rear/rear/image_raw
  - 拼接图像: camera_rear/rear/image_stitched

使用方法:
  ros2 launch hk_line_camera dual_camera_with_stitching.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('hk_line_camera')
    
    # ==================== Launch Arguments ====================
    # Front camera config
    front_camera_config_arg = DeclareLaunchArgument(
        'front_camera_config',
        default_value=os.path.join(pkg_share, 'config', 'camera_front_params.yaml'),
        description='Path to the front camera parameters YAML file'
    )
    
    # Rear camera config
    rear_camera_config_arg = DeclareLaunchArgument(
        'rear_camera_config',
        default_value=os.path.join(pkg_share, 'config', 'camera_rear_params.yaml'),
        description='Path to the rear camera parameters YAML file'
    )
    
    # Front stitching config
    front_stitching_config_arg = DeclareLaunchArgument(
        'front_stitching_config',
        default_value=os.path.join(pkg_share, 'config', 'stitching_front_params.yaml'),
        description='Path to the front stitching parameters YAML file'
    )
    
    # Rear stitching config
    rear_stitching_config_arg = DeclareLaunchArgument(
        'rear_stitching_config',
        default_value=os.path.join(pkg_share, 'config', 'stitching_rear_params.yaml'),
        description='Path to the rear stitching parameters YAML file'
    )
    
    # ==================== Front Camera Nodes ====================
    # Front camera node (启动时间: 0s)
    front_camera_node = Node(
        package='hk_line_camera',
        executable='hk_line_camera_node',
        name='hk_line_camera_front',
        namespace='camera_front',
        parameters=[LaunchConfiguration('front_camera_config')],
        output='screen'
    )
    
    # Front stitching node (启动时间: 0s)
    front_stitching_node = Node(
        package='hk_line_camera',
        executable='image_stitching_node',
        name='image_stitching_front',
        namespace='camera_front',
        parameters=[LaunchConfiguration('front_stitching_config')],
        output='screen'
    )
    
    # ==================== Rear Camera Nodes ====================
    # Rear camera node (延迟 3 秒启动，避免 SDK 初始化冲突)
    rear_camera_node = Node(
        package='hk_line_camera',
        executable='hk_line_camera_node',
        name='hk_line_camera_rear',
        namespace='camera_rear',
        parameters=[LaunchConfiguration('rear_camera_config')],
        output='screen'
    )
    
    # Rear stitching node
    rear_stitching_node = Node(
        package='hk_line_camera',
        executable='image_stitching_node',
        name='image_stitching_rear',
        namespace='camera_rear',
        parameters=[LaunchConfiguration('rear_stitching_config')],
        output='screen'
    )
    
    # 使用 TimerAction 延迟启动后相机节点，避免两个相机同时初始化 SDK 导致冲突
    delayed_rear_camera = TimerAction(
        period=3.0,  # 延迟 3 秒
        actions=[rear_camera_node]
    )
    
    delayed_rear_stitching = TimerAction(
        period=3.0,  # 延迟 3 秒
        actions=[rear_stitching_node]
    )
    
    return LaunchDescription([
        # Launch arguments
        front_camera_config_arg,
        rear_camera_config_arg,
        front_stitching_config_arg,
        rear_stitching_config_arg,
        # Front nodes (立即启动)
        front_camera_node,
        front_stitching_node,
        # Rear nodes (延迟 3 秒启动)
        delayed_rear_camera,
        delayed_rear_stitching
    ])
