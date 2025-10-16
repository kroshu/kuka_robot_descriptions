#!/bin/bash
echo "Running before run test target hook Gazebo test script..."
source /opt/ros/jazzy/setup.bash
ros2 run kuka_gazebo run_gazebo_tests.py
