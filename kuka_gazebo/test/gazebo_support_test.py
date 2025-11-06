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
    robot_family_support = LaunchConfiguration("robot_family_support")

    launch_file_path = os.path.join(
        get_package_share_directory("kuka_gazebo"), "launch", "test_gazebo.launch.py"
    )

    ld = LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file_path),
                launch_arguments={
                    "robot_model": robot_model,
                    "robot_family_support": robot_family_support,
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
