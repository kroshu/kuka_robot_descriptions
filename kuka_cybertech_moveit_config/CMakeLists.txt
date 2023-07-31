cmake_minimum_required(VERSION 3.5)

project(kuka_cybertech_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files(TARGET kr16r2010-2 srdf/kr16_r2010-2_arm.srdf.xacro OUTPUT kr16_r2010-2_arm.srdf INSTALL DESTINATION urdf)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
