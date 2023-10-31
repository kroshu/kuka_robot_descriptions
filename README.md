# kuka_simulators

This package contains resources that helps offline experiment of KUKA robots.

Github CI 
------------
[![Build Status](https://github.com/kroshu/kuka_simulators/workflows/CI/badge.svg?branch=main)](https://github.com/kroshu/kuka_simulators/actions)

## What is included?

- **kuka_resources** contains general, common files. It is copied from [kuka_experimental](https://github.com/ros-industrial/kuka_experimental) and is ported from ROS to ROS2. 
- **kuka_cybertech_support** contains urdf, config and mesh files for KUKA cybertech robots.
- **kuka_kr_moveit_config** contains configuration files for KUKA KR robots necessary for planning with moveit.
- **kuka_lbr_iisy_support** contains urdf, config and mesh files for KUKA iisy robots.
- **kuka_lbr_iisy_moveit_config** contains configuration files for KUKA iisy robots necessary for planning with moveit.
- **kuka_agilus_support** contains urdf, config and mesh files for KUKA Agilus robots, it is copied from [kuka_experimental](https://github.com/ros-industrial/kuka_experimental) and ported to ROS2.
- **kuka_lbr_iiwa7_support** contains urdf, config and mesh files for KUKA LBR iiwa 7 robots
- **kuka_omnimove_e575_support** contains urdf, config and mesh files for the KUKA Omnimove E575 platforms
- **kuka_rsi_simulator** implements a basic RSI simulator. It is also copied from [kuka_experimental](https://github.com/ros-industrial/kuka_experimental) and is ported to ROS2. It implements a UDP socket, and recieves the commands from an RSI interface and sends back the actual position of the robot (as it is a simulation, it is a simple closed loop, it sends what it recieved).


## Running the RSI simulator

```
ros2 launch kuka_rsi_simulator kuka_rsi_simulator_launch.py
```

## Starting the move group server for KR robots

The following launch file will start the driver with fake hardware, matching robot_model and robot_family arguments can be added to the command e.g. (robot_model:=kr16_r2010-2 robot_family:=cybertech):

```
ros2 launch kuka_kr_moveit_config moveit_planning_fake_hardware.launch.py 
```

The robot_manager lifecycle node must be configured and activated to be able to see the robot in rviz, afterwards the server will be able to accept planning requests. (An example how to create such a request can be found in the (ros2_kuka_drivers/kuka_driver_examples)/eci_demo package.)
