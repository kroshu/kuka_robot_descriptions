cmake_minimum_required(VERSION 3.5)

project(kuka_iisy_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
