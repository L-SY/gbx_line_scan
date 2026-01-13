from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Find package share directory
    pkg_share = FindPackageShare('rs485_interface')
    
    # Declare launch arguments
    device_path_arg = DeclareLaunchArgument(
        'device_path',
        default_value='/dev/ttyACM0',
        description='Path to RS485 serial device (e.g., /dev/ttyACM0)'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    slave_address_arg = DeclareLaunchArgument(
        'slave_address',
        default_value='1',
        description='MODBUS slave address'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share,
            'config',
            'distance_sensor_params.yaml'
        ]),
        description='Path to configuration file'
    )
    
    # Distance sensor node
    distance_sensor_node = Node(
        package='rs485_interface',
        executable='distance_sensor_node',
        name='distance_sensor_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'device_path': LaunchConfiguration('device_path'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'slave_address': LaunchConfiguration('slave_address'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        device_path_arg,
        baud_rate_arg,
        slave_address_arg,
        publish_rate_arg,
        config_file_arg,
        distance_sensor_node,
    ])

