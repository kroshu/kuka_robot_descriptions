#!/bin/bash
echo "Running before run test target hook Gazebo test script..."
ls
echo $ROS_DISTRO
cd kuka_gazebo
ls
#chmod +x src/run_gazebo_tests.py
cd ..
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run kuka_gazebo run_gazebo_tests.py
