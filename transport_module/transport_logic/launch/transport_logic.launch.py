from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Microswitch 参数
    microswitch_device_path_arg = DeclareLaunchArgument(
        'microswitch_device_path',
        default_value='/dev/ttyUSB0',
        description='Path to microswitch serial device'
    )
    microswitch_baud_rate_arg = DeclareLaunchArgument(
        'microswitch_baud_rate',
        default_value='115200',
        description='Baud rate for microswitch communication'
    )

    # ZD Motor 参数
    motor_device_path_arg = DeclareLaunchArgument(
        'motor_device_path',
        default_value='/dev/ttyACM1',
        description='Path to ZD motor RS485 device'
    )
    motor_baud_rate_arg = DeclareLaunchArgument(
        'motor_baud_rate',
        default_value='19200',
        description='Baud rate for ZD motor communication'
    )
    motor_slave_address_arg = DeclareLaunchArgument(
        'motor_slave_address',
        default_value='1',
        description='ZD motor MODBUS slave address'
    )

    # 速度参数
    reverse_speed_arg = DeclareLaunchArgument(
        'reverse_speed_rpm',
        default_value='1000',
        description='Reverse speed in RPM'
    )
    forward_speed_arg = DeclareLaunchArgument(
        'forward_speed_rpm',
        default_value='1000',
        description='Forward speed in RPM'
    )

    # 启动 microswitch
    microswitch_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('microswitch'),
                'launch',
                'microswitch.launch.py'
            ])
        ]),
        launch_arguments={
            'device_path': LaunchConfiguration('microswitch_device_path'),
            'baud_rate': LaunchConfiguration('microswitch_baud_rate'),
        }.items()
    )

    # 启动 transport_logic_node
    transport_logic_node = Node(
        package='transport_logic',
        executable='transport_logic_node',
        name='transport_logic_node',
        parameters=[{
            'motor_device_path': LaunchConfiguration('motor_device_path'),
            'motor_baud_rate': LaunchConfiguration('motor_baud_rate'),
            'motor_slave_address': LaunchConfiguration('motor_slave_address'),
            'reverse_speed_rpm': LaunchConfiguration('reverse_speed_rpm'),
            'forward_speed_rpm': LaunchConfiguration('forward_speed_rpm'),
        }],
        output='screen'
    )

    return LaunchDescription([
        microswitch_device_path_arg,
        microswitch_baud_rate_arg,
        motor_device_path_arg,
        motor_baud_rate_arg,
        motor_slave_address_arg,
        reverse_speed_arg,
        forward_speed_arg,
        microswitch_launch,
        transport_logic_node,
    ])
