cmake_minimum_required(VERSION 3.8)
project(kuka_kl_support)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY config meshes srdf urdf DESTINATION share/${PROJECT_NAME})

ament_package()
