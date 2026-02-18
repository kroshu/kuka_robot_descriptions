# Copyright 2022 Aron Svastits
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_family = LaunchConfiguration("robot_family")
    moveit_config_pkg = LaunchConfiguration("moveit_config")
    dof = LaunchConfiguration("dof")

    rviz_config_file = (
        get_package_share_directory("kuka_resources")
        + f"/config/planning_{dof.perform(context)}_axis.rviz"
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(f"kuka_{robot_family.perform(context)}_support"),
                    "urdf",
                    robot_model.perform(context) + ".urdf.xacro",
                ]
            ),
            " ",
            "mode:=",
            "mock",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    controller_config = (
        get_package_share_directory("kuka_resources")
        + f"/config/fake_hardware_config_{dof.perform(context)}_axis.yaml"
    )

    controller_manager_node = "/controller_manager"

    moveit_config = (
        MoveItConfigsBuilder(f"kuka_{moveit_config_pkg.perform(context)}")
        .robot_description(
            file_path=get_package_share_directory(f"kuka_{robot_family.perform(context)}_support")
            + f"/urdf/{robot_model.perform(context)}.urdf.xacro"
        )
        .robot_description_semantic(
            get_package_share_directory(f"kuka_{moveit_config_pkg.perform(context)}_moveit_config")
            + f"/urdf/{robot_model.perform(context)}.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .joint_limits(
            file_path=get_package_share_directory(f"kuka_{robot_family.perform(context)}_support")
            + f"/config/{robot_model.perform(context)}_joint_limits.yaml"
        )
        .to_moveit_configs()
    )

    move_group_server = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
    )

    robot_description_kinematics = {
        "robot_description_kinematics": {
            "manipulator": {"kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin"}
        }
    }

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "error"],
        parameters=[
            robot_description_kinematics,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Spawn controllers
    def controller_spawner(controller_with_config):
        arg_list = [controller_with_config[0], "-c", controller_manager_node]
        if controller_with_config[1] is not None:
            arg_list.append("-p")
            arg_list.append(controller_with_config[1])
        return Node(package="controller_manager", executable="spawner", arguments=arg_list)

    controller_names_and_config = [
        ("joint_state_broadcaster", None),
        ("joint_trajectory_controller", controller_config),
    ]

    controller_spawners = [
        controller_spawner(controllers) for controllers in controller_names_and_config
    ]

    to_start = [control_node, robot_state_publisher, rviz, move_group_server] + controller_spawners

    return to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("robot_family", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("moveit_config", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("dof", default_value="6"))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
