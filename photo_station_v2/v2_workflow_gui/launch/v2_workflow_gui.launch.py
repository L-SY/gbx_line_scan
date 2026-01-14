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
    # ros2 run v2_workflow_gui v2_workflow_gui
    # as they require an interactive display environment
    v2_workflow_gui_node = Node(
        package='v2_workflow_gui',
        executable='v2_workflow_gui',
        name='v2_workflow_gui',
        output='screen'
    )
    
    return LaunchDescription([
        topic_arg,
        v2_workflow_gui_node,
    ])

