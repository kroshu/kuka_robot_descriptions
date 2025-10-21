#!/bin/bash
echo "Running before run test target hook Gazebo test script..."
ls
cd ~/target_ws
ls
source install/setup.bash
#source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch kuka_gazebo gazebo.launch.py
ros2 run kuka_gazebo run_gazebo_tests.py
