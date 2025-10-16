#!/bin/bash
echo "Running before run test target hook Gazebo test script..."
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 run kuka_gazebo run_gazebo_tests.py
