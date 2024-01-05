# Copyright 2023 SAM XL (Eugenio Bernardi)

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("kuka_fortec_support"),
                    "urdf",
                    "kr560_r3100_2.urdf.xacro",
                ]
            ),
            " ",
            "use_fake_hardware:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kuka_resources"), "config", "view_6_axis_urdf.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_launch",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="log",
    )

    return LaunchDescription([robot_state_publisher, rviz_node, joint_state_publisher])
