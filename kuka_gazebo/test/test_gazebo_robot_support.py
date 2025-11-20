import os
import unittest
import pytest
from launch import LaunchDescription
from launch_testing.actions import ReadyToTest
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Define a dummy node to simulate a ROS 2 node during launch
dummy_node = Node(
    package="kuka_gazebo", executable="test_keep_alive", name="dummy_node", output="screen"
)


@pytest.mark.launch_test
def generate_test_description():
    ld = LaunchDescription(
        [
            dummy_node,
            ReadyToTest(),
        ]
    )
    return ld, {}


# Check the summary file of gazebo supported robots
class TestDuringLaunch(unittest.TestCase):
    def test_robot_initialization(self):
        """
        Checks that all robot models in the test summary file passed.
        """
        directory_path = get_package_share_directory("kuka_gazebo")
        file_path = os.path.join(directory_path, "gazebo_test.txt")
        with open(file_path) as gazebo_result:
            for line in gazebo_result:
                line = line.strip()
                if "Model:" in line:
                    self.assertIn("Status: PASS", line)

    def test_always_passes(self, proc_output):
        proc_output.assertWaitFor("Remaining time: 0", timeout=30)
