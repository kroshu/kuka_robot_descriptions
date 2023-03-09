cmake_minimum_required(VERSION 3.5)

project(kuka_lbr_iisy_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files(TARGET xacro srdf/iisy_arm.srdf.xacro OUTPUT iisy.srdf INSTALL DESTINATION config)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
