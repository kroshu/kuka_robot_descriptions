cmake_minimum_required(VERSION 3.5)

project(kuka_kr_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# TODO: add more robots to this list
set(supported_robots
  kr6_r700_sixx
  kr6_r900_sixx
  kr10_r1100_2
  kr16_r2010_2
  kr70_r2100
  kr150_r3100_2
  kr210_r2700_2
  kr210_r3100_2
  kr240_r3330
  kr560_r3100_2
)

foreach(model IN LISTS supported_robots)
  xacro_add_files(TARGET ${model} srdf/kr_arm.srdf.xacro REMAP robot_name:=${model} OUTPUT ${model}_arm.srdf INSTALL DESTINATION urdf)
endforeach()

install(DIRECTORY config srdf launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
