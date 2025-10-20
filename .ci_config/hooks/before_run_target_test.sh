#!/bin/bash
echo "Running before run test target hook Gazebo test script..."
ls
echo $ROS_DISTRO
cd kuka_gazebo
echo "kuka_gazebo"
ls
cd src
echo "src"
ls
cd ..
cd launch
echo "launch"
ls
cd ..
cd test
echo "test"
ls
cd ..
#chmod +x src/run_gazebo_tests.py
cd ..
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run kuka_gazebo run_gazebo_tests.py
