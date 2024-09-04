# Copyright 2024 Aron Svastits
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
import os

from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource,
)  # noqa: E501
from launch.actions.include_launch_description import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def list_test_launch_files():
    files = [
        f
        for f in os.listdir(get_package_share_directory("kuka_fortec_support") + "/launch/")
        if f.endswith(".py")
    ]
    return files


# Launch all of the robot visualisation launch files one by one
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
@launch_testing.parametrize("test_file", list_test_launch_files())
def generate_test_description(test_file):
    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("kuka_fortec_support"),
                        "/launch/",
                        test_file,
                    ]
                )
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestModels(unittest.TestCase):
    def test_read_stdout(self, proc_output):
        # Check for robot initialization
        proc_output.assertWaitFor("Robot initialized", timeout=5)
