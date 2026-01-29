from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device_path',
            default_value='/dev/ttyUSB1',
            description='Path to serial device'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for communication'
        ),
        DeclareLaunchArgument(
            'read_rate',
            default_value='10.0',
            description='Reading rate in Hz'
        ),
        DeclareLaunchArgument(
            'timeout_ms',
            default_value='1000',
            description='Read timeout in milliseconds'
        ),
        Node(
            package='microswitch',
            executable='microswitch_node',
            name='microswitch_node',
            parameters=[{
                'device_path': LaunchConfiguration('device_path'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'read_rate': LaunchConfiguration('read_rate'),
                'timeout_ms': LaunchConfiguration('timeout_ms'),
            }],
            output='screen'
        )
    ])
