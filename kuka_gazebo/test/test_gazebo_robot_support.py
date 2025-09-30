#!/usr/bin/env python3

import os
import unittest
import launch
import launch_ros.actions
import launch_testing
import launch_testing.markers
import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from launch_ros.actions import Node

dummy_node = Node(
    package='demo_nodes_cpp',
    executable='listener',
    name='dummy_node',
    output='screen'
)



@pytest.mark.launch_test
def generate_test_description():
    ld = LaunchDescription([
        #TimerAction(period=5.0, actions=[ReadyToTest()]),
        dummy_node,
        ReadyToTest(),
    ])
    return ld, {}


class TestDuringLaunch(unittest.TestCase):

    def test_robot_initialization(self):
        log_file_path = os.path.expanduser("~/ros2_ws/src/kuka_robot_descriptions/kuka_gazebo/test")
        file_path = os.path.join(log_file_path, "gazebo_test.txt")
        with open(file_path, 'r') as gazebo_result:
            for line in gazebo_result:
                line = line.strip()
                if 'Model:' in line:
                    self.assertIn('Status: PASS', line)

    def test_always_pases(self):
        self.assertTrue(True)