# kuka_robot_descriptions

This repository contains support packages that can be used with real KUKA robots as well as with simulations.

Github CI
------------
[![Build Status](https://github.com/kroshu/kuka_robot_descriptions/workflows/CI/badge.svg?branch=main)](https://github.com/kroshu/kuka_robot_descriptions/actions)

## What is included?

- **kuka_resources** contains general, common files. It is copied from [kuka_experimental](https://github.com/ros-industrial/kuka_experimental) and is ported from ROS to ROS2.
- **kuka_cybertech_support** contains urdf, config and mesh files for KUKA cybertech robots.
- **kuka_kr_moveit_config** contains configuration files for KUKA KR robots necessary for planning with moveit.
- **kuka_lbr_iisy_support** contains urdf, config and mesh files for KUKA iisy robots.
- **kuka_lbr_iisy_moveit_config** contains configuration files for KUKA LBR iisy robots necessary for planning with moveit.
- **kuka_agilus_support** contains urdf, config and mesh files for KUKA Agilus robots, it is copied from [kuka_experimental](https://github.com/ros-industrial/kuka_experimental) and ported to ROS2.
- **kuka_lbr_iiwa_support** contains urdf, config and mesh files for KUKA LBR iiwa robots
- **kuka_lbr_iiwa_moveit_config** contains configuration files for KUKA LBR iiwa robots necessary for planning with moveit.
- **kuka_rsi_simulator** implements a basic RSI simulator. It is also copied from [kuka_experimental](https://github.com/ros-industrial/kuka_experimental) and is ported to ROS2. It implements a UDP socket, and receives the commands from an RSI interface and sends back the actual position of the robot (as it is a simulation, it is a simple closed loop, it sends what it received).

## Structure of the support packages

All support packages consist of 4 folders:
- config: contains joint limits, necessary for time parametrization of trajectories
- launch: contains launch files to be able to visualize the robot models
- meshes: contains collision and visual meshes for the robots
- urdf: contains the xacro files describing the robots, including ros2_control integration (with fake hardware argument)

### Xacro files
 Each robot has two specific xacro files: a macro ({*robot_name*}_macro.xacro) and another file instantiating this macro ({*robot_name*}.urdf.xacro). Additionally there is a xacro providing ros2_control integration, including the name and type of the hardware interface, hardware parameters and the supported state and command interfaces.
 Additionally a transmission xacro is provided for gazebo support, but the mechanicalReduction parameters contained within are not valid, only placeholders.

 The macro files contain the links and joints of the main serial chain, including transformations, rotation axes, inertial properties, joint position, velocity and effort limits and the location of the mesh files.

 The macro file follows the ROS-Industrial conventions:
 - link names are "link_{*i*}"
 - joint names are "joint_a{*i*}"
 - all link and joint names have a {prefix} argument
 - base frame: equivalent to the base frame defined by the industrial controller ($ROBROOT)
 - flange frame: attachment point for EEF models
 - tool0 frame: all-zeros tool frame

 The frames in the xacros follow the Denavit–Hartenberg conventions of Khalil-Dombre.
 All robots in the xacros are named according to the following pattern: {kr/lbr_iisy/lbr_iiwa}{*payload*}\_r{*reach*}\_{*version*}.
 where version is optional and comes from the official product name. (e.g. KR 120 R3100-2 is named kr120_r3100_2)

 The moveit configuration packages also contain xacros, that describe the semantic information of the robots: planning groups, default states and link-pairs, for which collision checking should not be done. The default planning group (from base_link to tool0) is named "manipulator" for all robot arm. An end effector, named "end_effector" is also defined for all robots, which enables visualising end effector paths in rviz.

 To visualise the robot models, the launch files in the launch directory of the support packages can be used. These also start a joint state publisher with default values for all joints, therefore it is not possible to move the robot, only to visualise the frames and joints of the model in default position.

### Joint limit configurations

 The support packages contain a joint limits file for every supported robot model, necessary time parametrization of moveit-planned paths. They contain the velocity limits also available in the URDF model and additional acceleration limits. Acceleration limits can never be global, these values are calculated from the worst-case ramp-up time to reach maximum velocity. The easiest way to modify the allowed velocities and accelerations is to change the velocity and acceleration scaling factors also available in the same configuration files. (The scaling factor can never be smaller than 1.)

 
### Extending the models

 In real applications, it's likely that your description will be more complex, involving multiple objects next to the robot and optionally end effectors. It is recommended to create a new, dedicated ROS2 package specifically for managing this extended description by copying and extending the base robot model.

 Example of attaching an end effector (with link name "eef_base_link") to the flange frame, which could be defined in a different xacro file:
```
<joint name="${prefix}flange-${prefix}eef" type="fixed">
 <origin xyz="0 0 0" rpy="0 0 0" />
 <parent link="${prefix}flange" />
 <child link="${prefix}eef_base_link" />
</joint>
```

## What data is verifyed?

Some of the data in the xacros might not be valid or missing, the following table shows what can be considered valid.

|Robot name | Transformations | Joint position limits | Joint velocity limits | Joint effort limits | Inertial values | Simplified collision meshes|
|---|:---:|:---:|:---:|:---:|:---:|:---:|
|lbr_iisy3_r760 | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | | <img src="doc/resources/verified.png" alt="Verified" width="25"/> |
|lbr_iisy11_r1300 | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | | |
|lbr_iisy15_r930 | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | | |
|lbr_iiwa14_r820 | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | | | |
|kr6_r700_sixx | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | | | <img src="doc/resources/verified.png" alt="Verified" width="25"/> |
|kr6_r900_sixx | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | | | <img src="doc/resources/verified.png" alt="Verified" width="25"/> |
|kr16_r2010_2 | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | | |
|kr210_r2700_2 | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | | |
|kr210_r3100_2 | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | <img src="doc/resources/verified.png" alt="Verified" width="25"/> | | |

## Starting the move group server with fake hardware

To start the driver with fake hardware and the motion planning rviz plugin, the following launch files can be used:

For KR robots (KSS):
```
ros2 launch kuka_kr_moveit_config moveit_planning_fake_hardware.launch.py
```
Matching robot_model and robot_family arguments can be added to the command e.g. (robot_model:=kr16_r2010_2 robot_family:=cybertech).

For LBR iiwa robots (Sunrise):
```
ros2 launch kuka_lbr_iisy_moveit_config moveit_planning_fake_hardware.launch.py
```

For LBR iisy robots (iiQKA):
```
ros2 launch kuka_lbr_iiwa_moveit_config moveit_planning_fake_hardware.launch.py 
```

This will launch exactly the same nodes, as the driver launch file, but with fake hardware. Therefore the robot_manager lifecycle node must be configured and activated to be able to see the robot in rviz. After activation the server will be able to accept planning requests, from the plugin or from code. (An example how to create such a request from C++ code can be found in the (kuka_drivers/examples)/iiqka_moveit_example package.)
