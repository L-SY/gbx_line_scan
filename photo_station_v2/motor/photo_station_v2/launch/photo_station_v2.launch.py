#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("photo_station_v2"), "urdf", "photo_station_v2.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("photo_station_v2"), "rviz", "photo_station_v2.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Controller manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [FindPackageShare("photo_station_v2"), "config", "photo_station_v2_controllers.yaml"]
            ),
            {"use_sim_time": use_sim_time},
        ],
        output="both",
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # PID velocity controller spawner (now handles both joints)
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay velocity controller after joint state broadcaster
    delay_velocity_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[velocity_controller_spawner],
        )
    )

    nodes = [
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_velocity_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes) 