cmake_minimum_required(VERSION 3.5)

project(kuka_lbr_iico_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# TODO: add more robots to this list
set(supported_robots
  lbr_iico7_r900
  lbr_iico12_r1260
)

foreach(model IN LISTS supported_robots)
  xacro_add_files(TARGET ${model} srdf/lbr_iico_arm.srdf.xacro REMAP robot_name:=${model} OUTPUT ${model}.srdf INSTALL DESTINATION urdf)
endforeach()

install(DIRECTORY config srdf launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
