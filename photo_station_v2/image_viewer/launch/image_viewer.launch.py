from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Declare launch argument for topic
    topic_arg = DeclareLaunchArgument(
        'topic',
        default_value='/image_stitched',
        description='Image topic to subscribe to'
    )
    
    # Launch the image viewer node using Node action
    # Note: Qt GUI applications typically work better when run directly with:
    # ros2 run image_viewer image_viewer
    # as they require an interactive display environment
    image_viewer_node = Node(
        package='image_viewer',
        executable='image_viewer',
        name='image_viewer',
        output='screen'
    )
    
    return LaunchDescription([
        topic_arg,
        image_viewer_node,
    ])

