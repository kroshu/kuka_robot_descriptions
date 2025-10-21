#!/bin/bash
echo "Running before run test target hook Gazebo test script..."
Xvfb :99 -screen 0 1024x768x24 &
XVFB_PID=$!
sleep 2
export DISPLAY=:99
echo "virtual dsplay set"
cd ~/target_ws
ls
source install/setup.bash
ros2 launch kuka_gazebo gazebo.launch.py
ros2 run kuka_gazebo run_gazebo_tests.py
