# kuka_simulators

This package contains resources that helps offline experiment of KUKA robots.

Github CI 
------------
[![Build Status](https://github.com/kroshu/ros2_kuka_sunrise/workflows/CI/badge.svg?branch=master)](https://github.com/kroshu/kuka_simulators/actions)

## What is included?

- **kuka_resources** contains general, common files. It is copied from [kuka_experimental](https://github.com/ros-industrial/kuka_experimental) and is ported from ROS to ROS2. 
- **kuka_iisy_support** contains urdf, config and mesh files for KUKA iisy robots.
- **kuka_iisy_moveit_config** contains configuration files for KUKA iisy robots necessary for planning with moveit.
- **kuka_kr6_support** contains urdf, config and mesh files for KUKA KR6 robots, it is copied from [kuka_experimental](https://github.com/ros-industrial/kuka_experimental) and is ported to ROS2.
- **kuka_lbr_iiwa7_support** contains urdf, config and mesh files for KUKA LBR iiwa 7 robots
- **kuka_rsi_simulator** implements a basic RSI simulator. It is also copied from [kuka_experimental](https://github.com/ros-industrial/kuka_experimental) and is ported to ROS2. It implements a UDP socket, and recieves the commands from an RSI interface and sends back the actual position of the robot (as it is a simulation, it is a simple closed loop, it sends what it recieved).

## How to run

```
ros2 launch kuka_rsi_simulator kuka_rsi_simulator_launch.py
```
