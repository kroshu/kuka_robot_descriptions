cmake_minimum_required(VERSION 3.5)

project(kuka_lbr_iiwa_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files(TARGET iiwa14 srdf/lbr_iiwa14_r820_arm.srdf.xacro OUTPUT lbr_iiwa14_r820.srdf INSTALL DESTINATION urdf)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
