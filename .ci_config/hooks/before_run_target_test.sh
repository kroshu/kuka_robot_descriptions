#!/bin/bash
echo "Running before run test target hook Gazebo test script..."
# sudo apt-get install -y libxcb-xinerama0
# Xvfb :99 -screen 0 1024x768x24 &
# XVFB_PID=$!
# sleep 2
# export DISPLAY=:99
# export QT_QPA_PLATFORM=xcb
# export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/qt/plugins/platforms  # Optional
# echo "Virtual display set"
cd ~/target_ws
ls
source install/setup.bash
ros2 run kuka_gazebo run_gazebo_tests.py
kill $XVFB_PID