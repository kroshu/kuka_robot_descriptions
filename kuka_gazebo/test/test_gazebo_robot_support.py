#!/usr/bin/env python3

import os
import unittest
import pytest
from launch import LaunchDescription
from launch_testing.actions import ReadyToTest
from launch_ros.actions import Node

# Define a dummy node to simulate a ROS 2 node during launch
dummy_node = Node(
    package="demo_nodes_cpp", executable="talker", name="dummy_node", output="screen"
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


class TestDuringLaunch(unittest.TestCase):
    def test_robot_initialization(self):
        """
        Checks that all robot models in the test summary file passed.
        """
        # Define both possible paths
        path1 = os.path.expanduser("~/ros2_ws/src/kuka_robot_descriptions/kuka_gazebo/test")
        path2 = "/root/target_ws/src/kuka_robot_descriptions/kuka_gazebo/test"
        # Choose the existing path
        log_file_path = path1 if os.path.exists(path1) else path2
        file_path = os.path.join(log_file_path, "gazebo_test.txt")
        with open(file_path) as gazebo_result:
            for line in gazebo_result:
                line = line.strip()
                if "Model:" in line:
                    self.assertIn("Status: PASS", line)

    def test_always_passes(self, proc_output):
        proc_output.assertWaitFor("Hello World: 15", timeout=30)
