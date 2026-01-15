#!/bin/bash
source /opt/ros/humble/setup.bash

# Which 'ros2' is being executed?
type -a ros2
which ros2

# What verbs does 'ros2' see right now?
ros2 --help

# Check environment points to ROS 2
echo "$AMENT_PREFIX_PATH"
echo "$COLCON_CURRENT_PREFIX"
echo "$PYTHONPATH"

# Show Python being used by ros2
python3 -c "import sys; print(sys.executable); print(sys.path)"

# Is the ros2run plugin importable?
python3 -c "import importlib; print(importlib.util.find_spec('ros2run'))"

echo "Running before run test target hook Gazebo test script..."
cd ~/target_ws
source install/setup.bash
ros2 run kuka_gazebo run_gazebo_tests.py
