cmake_minimum_required(VERSION 3.8)
project(kuka_kl_support)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY config meshes srdf urdf DESTINATION share/${PROJECT_NAME})

# Required for Gazebo integration via package environment setup.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.dsv")

ament_package()
