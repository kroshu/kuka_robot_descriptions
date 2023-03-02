cmake_minimum_required(VERSION 3.5)

project(kuka_iisy_support)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files(TARGET xacro urdf/iisy_arm.srdf.xacro OUTPUT iisy.srdf INSTALL DESTINATION urdf)

install(DIRECTORY launch meshes urdf config
  DESTINATION share/${PROJECT_NAME})

ament_package()
