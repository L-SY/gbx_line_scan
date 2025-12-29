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
        default_value=os.path.join(pkg_share, 'config', 'stitching_params.yaml'),
        description='Path to the stitching parameters YAML file'
    )
    
    # Image stitching node
    image_stitching_node = Node(
        package='hk_line_camera',
        executable='image_stitching_node',
        name='image_stitching_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        image_stitching_node
    ])

