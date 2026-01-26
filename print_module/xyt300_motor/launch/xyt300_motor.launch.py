from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device_path',
            default_value='/dev/ttyACM0',
            description='Path to serial device'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for communication'
        ),
        DeclareLaunchArgument(
            'slave_address',
            default_value='1',
            description='MODBUS slave address (1-247)'
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
        Node(
            package='xyt300_motor',
            executable='xyt300_motor_node',
            name='xyt300_motor_node',
            parameters=[{
                'device_path': LaunchConfiguration('device_path'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'slave_address': LaunchConfiguration('slave_address'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'timeout_ms': LaunchConfiguration('timeout_ms'),
            }],
            output='screen'
        )
    ])
