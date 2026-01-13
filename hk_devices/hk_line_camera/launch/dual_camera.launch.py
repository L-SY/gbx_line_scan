"""
双相机启动文件 (Dual Camera Launch File)

启动两个独立的线扫相机节点：
- 前相机 (Front): 192.168.1.101, 发布到 front/image_raw
- 后相机 (Rear):  192.168.2.101, 发布到 rear/image_raw

使用方法:
  ros2 launch hk_line_camera dual_camera.launch.py

也可以单独指定配置文件:
  ros2 launch hk_line_camera dual_camera.launch.py \
      front_config:=/path/to/front_config.yaml \
      rear_config:=/path/to/rear_config.yaml
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
    
    # Declare launch arguments
    front_config_arg = DeclareLaunchArgument(
        'front_config',
        default_value=os.path.join(pkg_share, 'config', 'camera_front_params.yaml'),
        description='Path to the front camera parameters YAML file'
    )
    
    rear_config_arg = DeclareLaunchArgument(
        'rear_config',
        default_value=os.path.join(pkg_share, 'config', 'camera_rear_params.yaml'),
        description='Path to the rear camera parameters YAML file'
    )
    
    # Front camera node (立即启动)
    front_camera_node = Node(
        package='hk_line_camera',
        executable='hk_line_camera_node',
        name='hk_line_camera_front',
        namespace='camera_front',
        parameters=[LaunchConfiguration('front_config')],
        output='screen'
    )
    
    # Rear camera node (延迟启动)
    rear_camera_node = Node(
        package='hk_line_camera',
        executable='hk_line_camera_node',
        name='hk_line_camera_rear',
        namespace='camera_rear',
        parameters=[LaunchConfiguration('rear_config')],
        output='screen'
    )
    
    # 使用 TimerAction 延迟启动后相机节点，避免两个相机同时初始化 SDK 导致冲突
    delayed_rear_camera = TimerAction(
        period=3.0,  # 延迟 3 秒
        actions=[rear_camera_node]
    )
    
    return LaunchDescription([
        front_config_arg,
        rear_config_arg,
        front_camera_node,
        delayed_rear_camera
    ])
