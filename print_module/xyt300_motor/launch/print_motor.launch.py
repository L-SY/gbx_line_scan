from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device_path',
            default_value='/dev/ttyACM1',
            description='Path to serial device'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for communication'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Publishing rate in Hz'
        ),
        DeclareLaunchArgument(
            'timeout_ms',
            default_value='1000',
            description='Communication timeout in milliseconds'
        ),
        # Motor 1 (ID=1) - Single motor node
        Node(
            package='xyt300_motor',
            executable='xyt300_motor_node',
            name='motor1_node',
            namespace='motor1',
            parameters=[{
                'device_path': LaunchConfiguration('device_path'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'slave_address': 1,
                'publish_rate': LaunchConfiguration('publish_rate'),
                'timeout_ms': LaunchConfiguration('timeout_ms'),
            }],
            output='screen'
        ),
    ])
