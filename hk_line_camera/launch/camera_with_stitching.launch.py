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
    camera_config_arg = DeclareLaunchArgument(
        'camera_config_file',
        default_value=os.path.join(pkg_share, 'config', 'camera_params.yaml'),
        description='Path to the camera parameters YAML file'
    )
    
    stitching_config_arg = DeclareLaunchArgument(
        'stitching_config_file',
        default_value=os.path.join(pkg_share, 'config', 'stitching_params.yaml'),
        description='Path to the stitching parameters YAML file'
    )
    
    # Camera node
    hk_line_camera_node = Node(
        package='hk_line_camera',
        executable='hk_line_camera_node',
        name='hk_line_camera_node',
        parameters=[LaunchConfiguration('camera_config_file')],
        output='screen'
    )
    
    # Image stitching node
    image_stitching_node = Node(
        package='hk_line_camera',
        executable='image_stitching_node',
        name='image_stitching_node',
        parameters=[LaunchConfiguration('stitching_config_file')],
        output='screen'
    )
    
    return LaunchDescription([
        camera_config_arg,
        stitching_config_arg,
        hk_line_camera_node,
        image_stitching_node
    ])

