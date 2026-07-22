# Copyright 2026 KUKA Hungaria Kft.
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
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def _ros2_control_macro_file_from_family(robot_family):
    if robot_family.startswith("lbr_"):
        return f"{robot_family}_ros2_control_macro.xacro"
    return f"kr_{robot_family}_ros2_control_macro.xacro"


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_family = LaunchConfiguration("robot_family")
    moveit_config_pkg = LaunchConfiguration("moveit_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    dof = LaunchConfiguration("dof")
    use_external_axis = LaunchConfiguration("use_external_axis")
    kl_model = LaunchConfiguration("kl_model")
    kl_prefix = LaunchConfiguration("kl_prefix")
    kl_urdf_package = LaunchConfiguration("kl_urdf_package")
    mode = LaunchConfiguration("mode")
    prefix = LaunchConfiguration("prefix")

    robot_model_value = robot_model.perform(context)
    robot_family_value = robot_family.perform(context)
    moveit_config_pkg_value = moveit_config_pkg.perform(context)
    use_external_axis_value = use_external_axis.perform(context).lower() == "true"
    kl_model_value = kl_model.perform(context)
    kl_prefix_value = kl_prefix.perform(context)
    kl_urdf_package_value = kl_urdf_package.perform(context)

    robot_support_package = f"kuka_{robot_family_value}_support"

    if use_external_axis_value:
        urdf_file_path = (
            get_package_share_directory("kuka_resources")
            + "/urdf/robot_with_external_axis_template.urdf.xacro"
        )
        urdf_mappings = {
            "composed_model": f"{robot_model_value}_with_{kl_model_value}",
            "robot_model": robot_model_value,
            "robot_family": robot_family_value,
            "robot_support_package": robot_support_package,
            "kl_urdf_package": kl_urdf_package_value,
            "robot_ros2_control_macro_file": _ros2_control_macro_file_from_family(
                robot_family_value
            ),
            "kl_model": kl_model_value,
            "kl_prefix": kl_prefix_value,
            "prefix": prefix,
        }

        srdf_file_path = (
            get_package_share_directory("kuka_resources")
            + "/srdf/robot_with_external_axis_template.srdf.xacro"
        )
        srdf_mappings = {
            "composed_model": f"{robot_model_value}_with_{kl_model_value}",
            "kl_urdf_package": kl_urdf_package_value,
            "prefix": prefix,
            "kl_prefix": kl_prefix_value,
        }

        controllers_file_path = (
            get_package_share_directory("kuka_resources")
            + f"/config/moveit_controllers_{dof.perform(context)}_axis_kl.yaml"
        )
        rviz_config_file = (
            get_package_share_directory("kuka_resources")
            + f"/config/planning_{dof.perform(context)}_axis_kl.rviz"
        )
    else:
        urdf_file_path = (
            get_package_share_directory(robot_support_package)
            + f"/urdf/{robot_model_value}.urdf.xacro"
        )
        urdf_mappings = None

        # Keep the existing non-external-axis semantic resolution unchanged.
        srdf_file_path = (
            get_package_share_directory(f"kuka_{moveit_config_pkg_value}_moveit_config")
            + f"/urdf/{robot_model_value}.srdf"
        )
        srdf_mappings = None

        controllers_file_path = "config/moveit_controllers.yaml"
        rviz_config_file = (
            get_package_share_directory("kuka_resources")
            + f"/config/planning_{dof.perform(context)}_axis.rviz"
        )

    robot_joint_limits_file_path = (
        get_package_share_directory(robot_support_package)
        + f"/config/{robot_model_value}_joint_limits.yaml"
    )

    moveit_builder = MoveItConfigsBuilder(f"kuka_{moveit_config_pkg_value}")
    if urdf_mappings is None:
        moveit_builder = moveit_builder.robot_description(file_path=urdf_file_path)
    else:
        moveit_builder = moveit_builder.robot_description(
            file_path=urdf_file_path,
            mappings=urdf_mappings,
        )

    if srdf_mappings is None:
        moveit_builder = moveit_builder.robot_description_semantic(srdf_file_path)
    else:
        moveit_builder = moveit_builder.robot_description_semantic(
            file_path=srdf_file_path,
            mappings=srdf_mappings,
        )

    with open(
        robot_joint_limits_file_path,
        "r",
        encoding="utf-8",
    ) as joint_limits_file:
        merged_joint_limits = yaml.safe_load(joint_limits_file)
    if use_external_axis_value:
        with open(
            get_package_share_directory(kl_urdf_package_value)
            + f"/config/{kl_model_value}_joint_limits.yaml",
            "r",
            encoding="utf-8",
        ) as ext_axis_joint_limits_file:
            ext_axis_joint_limits = yaml.safe_load(ext_axis_joint_limits_file)
        merged_joint_limits.setdefault("joint_limits", {}).update(
            ext_axis_joint_limits.get("joint_limits", {})
        )
    moveit_config = (
        moveit_builder.robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path=controllers_file_path)
        .joint_limits(file_path=robot_joint_limits_file_path)
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    moveit_config_dict = moveit_config.to_dict()
    robot_description_planning = moveit_config_dict.get("robot_description_planning", {})
    if not isinstance(robot_description_planning, dict):
        robot_description_planning = {}

    # Merge pre-loaded joint limits without writing a temporary YAML file.
    for key, value in merged_joint_limits.items():
        if isinstance(value, dict) and isinstance(robot_description_planning.get(key), dict):
            robot_description_planning[key].update(value)
        else:
            robot_description_planning[key] = value

    moveit_config_dict["robot_description_planning"] = robot_description_planning

    move_group_server = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config_dict,
            {"publish_planning_scene_hz": 30.0},
            {"use_sim_time": use_sim_time},
        ],
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

    return [rviz, move_group_server]


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("robot_family", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("moveit_config", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("dof", default_value="6"))
    launch_arguments.append(DeclareLaunchArgument("use_external_axis", default_value="false"))
    launch_arguments.append(DeclareLaunchArgument("kl_model", default_value="kl100_2"))
    launch_arguments.append(DeclareLaunchArgument("kl_urdf_package", default_value="kuka_kl_support"))
    launch_arguments.append(DeclareLaunchArgument("kl_prefix", default_value="rail_"))
    launch_arguments.append(DeclareLaunchArgument("prefix", default_value=""))
    launch_arguments.append(DeclareLaunchArgument("use_sim_time", default_value="False"))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
