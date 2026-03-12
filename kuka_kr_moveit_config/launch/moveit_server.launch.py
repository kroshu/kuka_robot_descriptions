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
from ament_index_python.packages import get_package_share_directory
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_family = LaunchConfiguration("robot_family")

    moveit_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("kuka_resources"),
                "/launch/moveit_server_template.launch.py",
            ]
        ),
        launch_arguments={
            "robot_family": f"{robot_family.perform(context)}",
            "robot_model": f"{robot_model.perform(context)}",
            "dof": f"{6}",
            "moveit_config": "kr",
        }.items(),
    )

    return [moveit_server]


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value="kr6_r700_sixx"))
    launch_arguments.append(DeclareLaunchArgument("robot_family", default_value="agilus"))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
