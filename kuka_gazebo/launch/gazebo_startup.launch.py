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


def _ros2_control_macro_file_from_family(robot_family):
    if robot_family.startswith("lbr_"):
        return f"{robot_family}_ros2_control_macro.xacro"
    return f"kr_{robot_family}_ros2_control_macro.xacro"


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
    use_external_axis = LaunchConfiguration("use_external_axis")
    use_external_axis_value = use_external_axis.perform(context).lower() == "true"
    kl_model = LaunchConfiguration("kl_model")
    kl_prefix = LaunchConfiguration("kl_prefix")
    kl_support_package = LaunchConfiguration("kl_support_package")
    kl_ros2_control_macro_file = LaunchConfiguration("kl_ros2_control_macro_file")
    kl_ros2_control_joints_macro = LaunchConfiguration("kl_ros2_control_joints_macro")

    robot_model_value = robot_model.perform(context)
    robot_family_value = robot_family.perform(context)
    kl_model_value = kl_model.perform(context)
    kl_prefix_value = kl_prefix.perform(context)
    kl_support_package_value = kl_support_package.perform(context)
    kl_ros2_control_macro_file_value = kl_ros2_control_macro_file.perform(context)
    kl_ros2_control_joints_macro_value = kl_ros2_control_joints_macro.perform(context)
    robot_support_package = f"kuka_{robot_family_value}_support"

    # TF prefix
    tf_prefix = (ns.perform(context) + "_") if ns.perform(context) != "" else ""

    # Resolve world path inside kuka_gazebo share
    world_path = os.path.join(get_package_share_directory("kuka_gazebo"), world.perform(context))

    # Build robot_description from xacro
    xacro_arguments = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
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
    ]

    if use_external_axis_value:
        urdf_source = PathJoinSubstitution(
            [
                FindPackageShare("kuka_resources"),
                "urdf",
                "robot_with_external_axis_template.urdf.xacro",
            ]
        )
        model_name = f"{robot_model_value}_with_{kl_model_value}"
        xacro_arguments.extend(
            [
                " ",
                "composed_model:=",
                model_name,
                " ",
                "robot_model:=",
                robot_model_value,
                " ",
                "robot_family:=",
                robot_family_value,
                " ",
                "robot_support_package:=",
                robot_support_package,
                " ",
                "kl_support_package:=",
                kl_support_package_value,
                " ",
                "robot_ros2_control_macro_file:=",
                _ros2_control_macro_file_from_family(robot_family_value),
                " ",
                "kl_ros2_control_macro_file:=",
                kl_ros2_control_macro_file_value,
                " ",
                "kl_model:=",
                kl_model_value,
                " ",
                "kl_ros2_control_joints_macro:=",
                kl_ros2_control_joints_macro_value,
                " ",
                "kl_prefix:=",
                kl_prefix_value,
            ]
        )
    else:
        urdf_source = PathJoinSubstitution(
            [FindPackageShare(robot_support_package), "urdf", robot_model_value + ".urdf.xacro"]
        )
        model_name = robot_model_value

    xacro_arguments.extend(
        [
            " ",
            urdf_source,
        ]
    )

    robot_description_content = Command(
        xacro_arguments,
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
            model_name,
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
        DeclareLaunchArgument(
            "use_external_axis",
            default_value="false",
            description="Enable external axis support in the robot xacro model.",
        ),
        DeclareLaunchArgument(
            "kl_model",
            default_value="kl100_2",
            description="External axis model used when use_external_axis is true.",
        ),
        DeclareLaunchArgument(
            "kl_support_package",
            default_value="kuka_kl_support",
            description="Package containing the external axis URDF files.",
        ),
        DeclareLaunchArgument(
            "kl_ros2_control_macro_file",
            default_value="kl_ros2_control_macro.xacro",
            description="External axis ros2_control macro file in the external axis package.",
        ),
        DeclareLaunchArgument(
            "kl_ros2_control_joints_macro",
            default_value="kuka_kl_ros2_control_joints",
            description=(
                "External axis ros2_control joints macro name in the external axis package."
            ),
        ),
        DeclareLaunchArgument(
            "kl_prefix",
            default_value="rail_",
            description="Joint/link prefix for the external axis when use_external_axis is true.",
        ),
    ]
    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])
