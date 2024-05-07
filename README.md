# kuka_robot_descriptions

This repository contains support packages that can be used with real KUKA robots as well as with simulations.

Github CI
------------
[![Build Status](https://github.com/kroshu/kuka_robot_descriptions/workflows/CI/badge.svg?branch=main)](https://github.com/kroshu/kuka_robot_descriptions/actions)

## What is included?

- `kuka_resources` contains general, common files. It is copied from [kuka_experimental](https://github.com/ros-industrial/kuka_experimental) and is ported from ROS to ROS2.
- `kuka_agilus_support` contains urdf, config and mesh files for KUKA Agilus robots, it is copied from [kuka_experimental](https://github.com/ros-industrial/kuka_experimental) and ported to ROS2.
- `kuka_cybertech_support` contains urdf, config and mesh files for KUKA cybertech robots.
- `kuka_fortec_support` contains urdf, config and mesh files for KUKA fortec robots.
- `kuka_iontec_support` contains urdf, config and mesh files for KUKA iontec robots.
- `kuka_quantec_support` contains urdf, config and mesh files for KUKA quantec robots.
- `kuka_kr_moveit_config` contains configuration files for KUKA KR robots necessary for planning with MoveIt.
- `kuka_lbr_iisy_support` contains urdf, config and mesh files for KUKA iisy robots.
- `kuka_lbr_iisy_moveit_config` contains configuration files for KUKA LBR iisy robots necessary for planning with MoveIt.
- `kuka_lbr_iiwa_support` contains urdf, config and mesh files for KUKA LBR iiwa robots
- `kuka_lbr_iiwa_moveit_config` contains configuration files for KUKA LBR iiwa robots necessary for planning with MoveIt.
- `kuka_mock_hardware_interface` contains a custom mock hardware interface for KUKA robots

## Structure of the support packages

All support packages consist of 4 folders:
- `config`: contains joint limits, necessary for time parametrization of trajectories
- `launch`: contains launch files to be able to visualize the robot models
- `meshes`: contains collision and visual meshes for the robots
- `urdf`: contains the xacro files describing the robots, including `ros2_control` integration (with fake hardware argument)

### Xacro files
Each robot has two specific xacro files: a macro (`{robot_name}_macro.xacro`) and another file instantiating this macro (`{robot_name}.urdf.xacro`). Additionally there is a xacro providing `ros2_control` integration, including the name and type of the hardware interface, hardware parameters and the supported state and command interfaces.
Additionally a transmission xacro is provided for gazebo support, but the `mechanicalReduction` parameters contained within are not valid, only placeholders.

The macro files contain the links and joints of the main serial chain, including transformations, rotation axes, inertial properties, joint position, velocity and effort limits and the location of the mesh files.

The macro file follows the ROS-Industrial conventions:
 - link names are `link_{i}`
 - joint names are `joint_{i}`
 - all link and joint names have a `prefix` argument
 - includes `base` frame: equivalent to the base frame defined by the industrial controller ($ROBROOT)
 - includes `flange` frame: attachment point for EEF models
 - includes `tool0` frame: all-zeros tool frame, identical to the tool frame defined by the industrial controller ($TOOL)

 All macros additionally contain a `world` fixed frame (without `prefix`). The transform from `world` to `base_link` can be given with the block parameter `*origin`.

All robots in the xacros are named according to the following pattern:

`{kr/lbr_iisy/lbr_iiwa}{payload}_r{reach}_{version}`,

where `version` is omitted, if the official product name does not contain it. (e.g. KR 120 R3100-2 is named `kr120_r3100_2` and LBR iisy 3 R760 is `lbr_iisy3_r760`)

The MoveIt configuration packages also contain xacros, that describe the semantic information of the robots: planning groups, default states and link-pairs, for which collision checking should not be done. The default planning group (from `base_link` to `tool0`) is named `manipulator` for all robot arms. An end effector, named `end_effector` is also defined for all robots, which enables visualising end effector paths in `rviz`.

To visualise the robot models, the launch files in the `launch` directory of the support packages can be used. These also start a `joint_state_publisher_gui` to enable visualisation of the robot meshes and frames with different joint configurations. However they have only visualisation purposes and cannot connect to real or fake hardware.

### Frame conventions

The frames of the main serial chain in the xacros (`base_link` to `link_6` or `link_7`) follow the Denavit–Hartenberg conventions of Khalil-Dombre.
The other frames, which are added to conform to ROS-Industrial follow the conventions defined there: `base` and `tool0` are defined to be identical to the frames on the controller, while `flange` follows [REP-103](https://www.ros.org/reps/rep-0103.html#coordinate-frame-conventions), meaning that in default position x+ points forwards and z+ upwards.

### Collision geometry

Collision meshes are provided for the robots to speed up collision avoidance and detection calculations. These are automatically generated from the visual meshes using the Blender python API (remesh modifier) with fixed parameter values. This generation process will be fine-tuned in the future to further optimize collision calculations.


### Joint limit configurations

The support packages contain a joint limits file for every supported robot model, necessary time parametrization of MoveIt-planned paths. They contain the velocity limits also available in the URDF model and additional acceleration limits. Acceleration limits can never be global, these values are calculated from the worst-case ramp-up time to reach maximum velocity. The easiest way to modify the allowed velocities and accelerations is to change the velocity and acceleration scaling factors also available in the same configuration files. (The scaling factor can never be smaller than 1.)


### Extending the models

In real applications, it's likely that the description will be more complex, involving multiple objects next to the robot and optionally end effectors. It is recommended to create a new, dedicated ROS2 package specifically for managing this extended description by including the xacro of the the base robot model and extending it.

Example of attaching an end effector (with link name `eef_base_link`) to the `flange` frame, which could be defined in a different xacro file:
```xml
<joint name="${prefix}flange-${prefix}eef" type="fixed">
 <origin xyz="0 0 0" rpy="0 0 0" />
 <parent link="${prefix}flange" />
 <child link="${prefix}eef_base_link" />
</joint>
```

## What data is verified?

The following table shows what data is included for each robot in the support packages:

|Robot name | Robot family | Transformations | Joint position limits | Joint velocity limits | Joint effort limits | Inertial values | Simplified collision meshes|
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|lbr_iisy3_r760| - | ✓ | ✓ | ✓ | ✓ | | ✓ |
|lbr_iisy11_r1300| - | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
|lbr_iisy15_r930| - | ✓ | ✓ | ✓ | ✓ | | ✓ |
|lbr_iiwa14_r820| - | ✓ | ✓ | ✓ | | | ✓ |
|kr6_r700_sixx| agilus | ✓ | ✓ | ✓ | | | ✓ |
|kr6_r900_sixx| agilus | ✓ | ✓ | ✓ | | | ✓ |
|kr10_r1100_2| agilus | ✓ | ✓ | ✓ | ✓ | | ✓ |
|kr16_r2010_2| cybertech | ✓ | ✓ | ✓ | ✓ | | ✓ |
|kr70_r2100| iontec | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
|kr210_r2700_2| quantec | ✓ | ✓ | ✓ | ✓ | | ✓ |
|kr210_r3100_2| quantec | ✓ | ✓ | ✓ | ✓ | | ✓ |
|kr560_r3100_2| fortec | ✓ | ✓ | ✓ | ✓ | | ✓ |

## Custom mock hardware

The repository also contains a mock hardware interface implementation, that extends the `mock_components::GenericSystem` defined in the `hardware_interface` package.
This is necessary, as the driver workflow also activates controllers, which is possible only if all of the interfaces claimed by the controller is provided by the hardware interface. This would not be the case for the default `GenericSystem`, therefore all of the custom state and command interfaces used by the drivers are exported by the `KukaMockHardwareInterface`.
Additionally two hardware parameters are added:
- To support similar timing behaviour as the actual robots, the mock hardware was extended with a blocking wait, so that the read function does not return immediately, but cyclically. The frequency of the loops is defined by the `cycle_time_ms` parameter. Default value is 4  [ms].
- To be able to test whether a specific setup would fit into the roundtrip time enforced by a real robot, the `roundtrip_time_micro` parameter can be used. If the `write()` method is not called before the given timeout is exceeded (starting from the previous `read()` function), a warning message is logged (but the return value of the `write()` will be still SUCCESS). Default value is 0 [us], which means, that the roundrip time should not be monitored.

The mock hardware was implemented in this repository to allow testing moveit capabilities for the robots without having to build the driver code.

## Starting the move group server with mock hardware

To start `rviz` with the motion planning plugin using fake hardware, the following launch files can be used:

#### KR robots (KSS):
```
ros2 launch kuka_kr_moveit_config moveit_planning_fake_hardware.launch.py
```
Matching `robot_model` and `robot_family` arguments can be added after the command (e.g. `robot_model:=kr16_r2010_2 robot_family:=cybertech`). The default robot model is `kr6_r700_sixx`

#### LBR iiwa robots (Sunrise):
```
ros2 launch kuka_lbr_iisy_moveit_config moveit_planning_fake_hardware.launch.py
```

#### LBR iisy robots (iiQKA):
```
ros2 launch kuka_lbr_iiwa_moveit_config moveit_planning_fake_hardware.launch.py
```
A `robot_model` argument can be added after the command (e.g. `robot_model:=lbr_iisy11_r1300`). The default robot model is `lbr_iisy3_r760`

These launch files are not using the actual driver implementation, they only start `rviz` the `move_group` server and a `ros2_control_node` with fake hardware and two controllers `joint_state_broadcaster` and `joint_trajectory_controller` The server will be able to accept planning requests from the plugin or from code. (An example how to create such a request from C++ code can be found in the `iiqka_moveit_example` package in the `kuka_drivers` repository.)
