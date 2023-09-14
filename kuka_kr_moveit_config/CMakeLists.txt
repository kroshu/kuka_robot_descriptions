cmake_minimum_required(VERSION 3.5)

project(kuka_kr_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# TODO: add more robots to this list
set(supported_robots
  kr6_r700_sixx
  kr6_r900_sixx
  kr10_r1100-2
  kr16_r2010-2
  kr210-r2700-2
  kr210-r3100-2
  )

foreach(model IN LISTS supported_robots)
  xacro_add_files(TARGET ${model} srdf/kr_arm.srdf.xacro REMAP robot_name:=${model} OUTPUT ${model}_arm.srdf INSTALL DESTINATION urdf)
endforeach()

install(DIRECTORY config srdf launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
