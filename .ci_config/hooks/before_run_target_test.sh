#!/bin/bash
echo "Running before run test target hook Gazebo test script..."
cd ~/target_ws
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
ls
source install/setup.bash
ros2 run kuka_gazebo run_gazebo_tests.py
kill $XVFB_PID