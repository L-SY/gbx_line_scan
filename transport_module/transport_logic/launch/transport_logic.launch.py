from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 启用控制参数
    enable_zd_motor_arg = DeclareLaunchArgument(
        'enable_zd_motor',
        default_value='true',
        description='Enable ZD Motor'
    )
    enable_servo_motor_arg = DeclareLaunchArgument(
        'enable_servo_motor',
        default_value='true',
        description='Enable LC Servo Motor'
    )

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
        default_value='/dev/ttyACM0',
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

    # ZD Motor 速度参数
    reverse_speed_arg = DeclareLaunchArgument(
        'reverse_speed_rpm',
        default_value='1000',
        description='ZD Motor reverse speed in RPM'
    )
    forward_speed_arg = DeclareLaunchArgument(
        'forward_speed_rpm',
        default_value='1000',
        description='ZD Motor forward speed in RPM'
    )

    # LC Servo Motor 参数
    servo_device_path_arg = DeclareLaunchArgument(
        'servo_device_path',
        default_value='/dev/ttyACM0',
        description='Path to LC Servo motor RS485 device'
    )
    servo_baud_rate_arg = DeclareLaunchArgument(
        'servo_baud_rate',
        default_value='19200',
        description='Baud rate for LC Servo motor communication'
    )
    servo_slave_address_arg = DeclareLaunchArgument(
        'servo_slave_address',
        default_value='2',
        description='LC Servo motor MODBUS slave address'
    )

    # LC Servo Motor 速度参数
    servo_forward_speed_arg = DeclareLaunchArgument(
        'servo_forward_speed_rpm',
        default_value='1000.0',
        description='LC Servo Motor forward speed in RPM'
    )
    servo_reverse_speed_arg = DeclareLaunchArgument(
        'servo_reverse_speed_rpm',
        default_value='1000.0',
        description='LC Servo Motor reverse speed in RPM'
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

    # 加载配置文件
    config_file = PathJoinSubstitution([
        FindPackageShare('transport_logic'),
        'config',
        'transport_logic_params.yaml'
    ])

    # 启动 transport_logic_node
    # 只从 yaml 文件加载参数
    # 如果需要通过命令行覆盖参数，可以使用：ros2 launch ... --ros-args -p servo_forward_speed_rpm:=200.0
    transport_logic_node = Node(
        package='transport_logic',
        executable='transport_logic_node',
        name='transport_logic_node',
        parameters=[
            config_file
        ],
        output='screen'
    )

    return LaunchDescription([
        enable_zd_motor_arg,
        enable_servo_motor_arg,
        microswitch_device_path_arg,
        microswitch_baud_rate_arg,
        motor_device_path_arg,
        motor_baud_rate_arg,
        motor_slave_address_arg,
        reverse_speed_arg,
        forward_speed_arg,
        servo_device_path_arg,
        servo_baud_rate_arg,
        servo_slave_address_arg,
        servo_forward_speed_arg,
        servo_reverse_speed_arg,
        microswitch_launch,
        transport_logic_node,
    ])
