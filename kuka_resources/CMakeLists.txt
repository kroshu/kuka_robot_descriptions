cmake_minimum_required(VERSION 3.5)
project(kuka_resources)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
