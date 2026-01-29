from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Include print_motor launch file to start motor node
    print_motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('xyt300_motor'),
                'launch',
                'print_motor.launch.py'
            ])
        ])
    )
    
    # Start GUI node
    gui_node = Node(
        package='print_workflow',
        executable='print_workflow',
        name='print_workflow_gui',
        output='screen'
    )
    
    return LaunchDescription([
        print_motor_launch,
        gui_node,
    ])
