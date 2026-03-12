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

import os
import unittest
import launch_testing
import launch_testing.markers
import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_testing.actions import ReadyToTest


# Parametrized test for a robot
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    robot_model = LaunchConfiguration("robot_model")
    robot_family = LaunchConfiguration("robot_family")

    launch_file_path = os.path.join(
        get_package_share_directory("kuka_gazebo"), "launch", "gazebo_startup.launch.py"
    )

    ld = LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file_path),
                launch_arguments={
                    "robot_model": robot_model,
                    "robot_family": robot_family,
                    "use_gui": "false",
                }.items(),
            ),
            ReadyToTest(),
        ]
    )
    return ld, {"robot_model": robot_model}


class TestDuringLaunch(unittest.TestCase):
    def test_robot_initialization(self, proc_output, robot_model):
        proc_output.assertWaitFor("Successful initialization of hardware ", timeout=20)
        proc_output.assertWaitFor("Successful 'configure' of hardware ", timeout=10)
        proc_output.assertWaitFor("Successful 'activate' of hardware ", timeout=10)
        proc_output.assertWaitFor("Configured and activated joint_state_broadcaster", timeout=10)
        proc_output.assertWaitFor(
            "Configured and activated joint_trajectory_controller", timeout=10
        )
        print("end of test")
