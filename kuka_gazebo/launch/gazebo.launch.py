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
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("gz_world")
    robot_model = LaunchConfiguration("robot_model")
    robot_family_path = LaunchConfiguration("robot_family_support")
    ns = LaunchConfiguration("namespace")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    if ns.perform(context) == "":
        tf_prefix = ""
    else:
        tf_prefix = ns.perform(context) + "_"

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(robot_family_path.perform(context)),
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

    # Get URDF via xacro
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        namespace=ns,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": True, "mode": "gazebo"},
        ],
    )

    # Gazebo
    world_path = os.path.join(get_package_share_directory("kuka_gazebo"), world.perform(context))
    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            ),
        ),
        launch_arguments={"gz_args": [world_path, " -r -v1"]}.items(),
    )

    label = ["-x", "-y", "-z", "-R", "-P", "-Y"]
    tf = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    tf = [str(x) for x in tf]
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

    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Spawn controllers
    def controller_spawner(controller_name, param_file=None, activate=False):
        arg_list = [
            controller_name,
            "-c",
            "controller_manager",
            "-n",
            ns,
        ]

        # Add param-file if it's provided
        if param_file:
            arg_list.extend(["--param-file", param_file])

        if not activate:
            arg_list.append("--inactive")

        return Node(package="controller_manager", executable="spawner", arguments=arg_list)

    controllers = {
        "joint_state_broadcaster": None,
        "joint_trajectory_controller": None,
    }

    controller_spawners = [
        controller_spawner(name, param_file, True) for name, param_file in controllers.items()
    ]

    nodes_to_start = [
        robot_state_publisher,
        gazebo_ld,
        gazebo_robot_node,
        gazebo_bridge,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value="lbr_iisy3_r760"))
    launch_arguments.append(
        DeclareLaunchArgument("robot_family_support", default_value="kuka_lbr_iisy_support")
    )
    launch_arguments.append(DeclareLaunchArgument("namespace", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("x", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("y", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("z", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("roll", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("pitch", default_value="0"))
    launch_arguments.append(DeclareLaunchArgument("yaw", default_value="0"))
    launch_arguments.append(
        DeclareLaunchArgument(
            name="gz_world",
            default_value="world/empty_world.sdf",
            description="The world to be loaded by Gazebo.",
        )
    )
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
