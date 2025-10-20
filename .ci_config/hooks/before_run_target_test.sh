#!/bin/bash
echo "Running before run test target hook Gazebo test script..."
chmod +x src/run_gazebo_tests.py
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run kuka_gazebo run_gazebo_tests.py
