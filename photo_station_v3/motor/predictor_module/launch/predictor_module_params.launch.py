#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('predictor_module')
    
    # Path to the params file
    params_file = os.path.join(pkg_share, 'config', 'predictor_params.yaml')

    # Create the node with params file
    predictor_node = Node(
        package='predictor_module',
        executable='predictor_node',
        name='predictor_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            # You can add remappings here if needed
        ]
    )

    return LaunchDescription([
        predictor_node,
    ])
