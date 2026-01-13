from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ip_address',
            default_value='192.168.3.101',
            description='IP address of the light controller'
        ),
        DeclareLaunchArgument(
            'status_publish_rate',
            default_value='1.0',
            description='Status publish rate in Hz'
        ),
        Node(
            package='hk_light_controller',
            executable='hk_light_controller_node',
            name='hk_light_controller',
            output='screen',
            parameters=[{
                'ip_address': LaunchConfiguration('ip_address'),
                'status_publish_rate': LaunchConfiguration('status_publish_rate'),
            }],
            remappings=[
                # You can add remappings here if needed
            ]
        ),
    ])

