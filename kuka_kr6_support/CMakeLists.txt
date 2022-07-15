cmake_minimum_required(VERSION 3.5)

project(kuka_kr6_support)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY config launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
