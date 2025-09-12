#!/usr/bin/env python3

# Copyright 2025 KUKA Hungary
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

import unittest

import launch
import launch.actions
import launch_testing.actions
import launch_testing.markers
import pytest

from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource,
)
from launch.actions.include_launch_description import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


# Launch driver startup
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("kuka_gazebo"),
                        "/launch/",
                        "gazebo.launch.py",
                    ]
                ),
                launch_arguments={
                    "robot_model": "kr210_r2700_2",
                    "robot_family_support": "kuka_quantec_support",
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestGazeboActivation(unittest.TestCase):
    def test_read_stdout(self, proc_output):
        # Check for successful initialization
        proc_output.assertWaitFor(
            "Successful initialization of hardware 'kr210_r2700_2'", timeout=15
        ) 
         # Check for successful configuration and activation
        proc_output.assertWaitFor(
            "Successful 'configure' of hardware 'kr210_r2700_2'", timeout=5
        )
        proc_output.assertWaitFor(
            "Successful 'activate' of hardware 'kr210_r2700_2'", timeout=5
        )
        # Check for successful configuration and activation
        proc_output.assertWaitFor("Configured and activated joint_state_broadcaster", timeout=5)
        proc_output.assertWaitFor("Configured and activated joint_trajectory_controller", timeout=5)


# This is required for Python's unittest framework
if __name__ == '__main__':
    unittest.main()