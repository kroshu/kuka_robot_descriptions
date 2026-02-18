# Copyright 2025 KUKA Hungaria Kft.
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
import os


def launch_setup(context, *args, **kwargs):
    # Launch configurations
    world = LaunchConfiguration("gz_world")
    robot_model = LaunchConfiguration("robot_model")
    robot_family = LaunchConfiguration("robot_family")
    ns = LaunchConfiguration("namespace")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    use_gui = LaunchConfiguration("use_gui")

    # TF prefix
    tf_prefix = (ns.perform(context) + "_") if ns.perform(context) != "" else ""

    # Resolve world path inside kuka_gazebo share
    world_path = os.path.join(get_package_share_directory("kuka_gazebo"), world.perform(context))

    # Build robot_description from xacro
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
            "gazebo",
            " ",
            "prefix:=",
            tf_prefix,
            " ",
            "x:=",
            x,
            " ",
            "y:=",
            y,
            " ",
            "z:=",
            z,
            " ",
            "roll:=",
            roll,
            " ",
            "pitch:=",
            pitch,
            " ",
            "yaw:=",
            yaw,
        ],
        on_stderr="capture",
    )
    robot_description = {"robot_description": robot_description_content}

    # robot_state_publisher
    robot_state_publisher = Node(
        namespace=ns,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True, "mode": "gazebo"}],
    )

    # Gazebo (GUI mode)
    gz_sim_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [world_path, " -r -v1"]}.items(),
        condition=IfCondition(use_gui),
    )

    # Gazebo (headless CI mode)
    gz_server_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_server.launch.py"])
        ),
        launch_arguments={
            "world_sdf_file": world_path,
            "container_name": "ros_gz_container",
            "create_own_container": "False",
            "use_composition": "False",
        }.items(),
        condition=UnlessCondition(use_gui),
    )

    # Bridge via the ros_gz_bridge launch file + config (works for both modes)
    ros_gz_bridge_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_bridge"), "launch", "ros_gz_bridge.launch.py"]
            )
        ),
        launch_arguments={
            "config_file": PathJoinSubstitution(
                [FindPackageShare("kuka_gazebo"), "config", "bridge_config.yaml"]
            ),
            "bridge_name": "ros_gz_bridge",
        }.items(),
    )

    # Spawn the robot into Gazebo
    label = ["-x", "-y", "-z", "-R", "-P", "-Y"]
    tf = [str(v) for v in [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    gazebo_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            robot_model,
            "-allow_renaming",
        ]
        + [item for pair in zip(label, tf) for item in pair],
        output="screen",
    )

    # Controller spawner helper
    def controller_spawner(controller_name, param_file=None, activate=False):
        args = [controller_name, "-c", "controller_manager", "-n", ns]
        if param_file:
            args.extend(["--param-file", param_file])
        if not activate:
            args.append("--inactive")
        return Node(package="controller_manager", executable="spawner", arguments=args)

    controllers = {
        "joint_state_broadcaster": None,
        "joint_trajectory_controller": None,
    }
    controller_spawners = [
        controller_spawner(name, param_file, True) for name, param_file in controllers.items()
    ]

    nodes = [
        robot_state_publisher,
        gz_sim_ld,
        gz_server_ld,
        gazebo_robot_node,
        ros_gz_bridge_ld,
    ] + controller_spawners

    return nodes


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument("robot_model", default_value="lbr_iisy3_r760"),
        DeclareLaunchArgument("robot_family", default_value="lbr_iisy"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("x", default_value="0"),
        DeclareLaunchArgument("y", default_value="0"),
        DeclareLaunchArgument("z", default_value="0"),
        DeclareLaunchArgument("roll", default_value="0"),
        DeclareLaunchArgument("pitch", default_value="0"),
        DeclareLaunchArgument("yaw", default_value="0"),
        DeclareLaunchArgument(
            "gz_world",
            default_value="world/empty_world.sdf",
            description="The world to be loaded by Gazebo.",
        ),
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="If true, launch gz_sim (GUI). If false, launch gz_server (headless).",
        ),
    ]
    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])
