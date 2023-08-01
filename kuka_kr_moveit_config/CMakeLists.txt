cmake_minimum_required(VERSION 3.5)

project(kuka_kr_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files(TARGET kr_arm srdf/kr_arm.srdf.xacro OUTPUT kr_arm.srdf INSTALL DESTINATION urdf)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
