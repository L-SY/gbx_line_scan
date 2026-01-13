#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 声明启动参数
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="是否使用仿真时间",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_file",
            default_value="test_robot.urdf.xacro",
            description="机器人描述文件名",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="test_controllers.yaml",
            description="控制器配置文件名",
        )
    )

    # 获取启动参数
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description_file = LaunchConfiguration("robot_description_file")
    controllers_file = LaunchConfiguration("controllers_file")

    # 获取包路径
    pkg_share = FindPackageShare(package='actuator_interface').find('actuator_interface')
    
    # 机器人描述
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([pkg_share, "test", "urdf", robot_description_file]),
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # 控制器参数
    robot_controllers = PathJoinSubstitution(
        [pkg_share, "test", "config", controllers_file]
    )

    # 控制管理器节点
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # 机器人状态发布器
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # 关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 努力控制器生成器
    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller", "--controller-manager", "/controller_manager"],
    )

    # 在关节状态广播器启动后启动努力控制器
    delay_effort_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[effort_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_effort_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes) 