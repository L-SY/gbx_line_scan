from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('hk_line_camera')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'camera_params.yaml'),
        description='Path to the camera parameters YAML file'
    )
    
    # Node
    hk_line_camera_node = Node(
        package='hk_line_camera',
        executable='hk_line_camera_node',
        name='hk_line_camera_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        hk_line_camera_node
    ])
