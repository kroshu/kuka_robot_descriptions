cmake_minimum_required(VERSION 3.5)
project(kuka_mobile_robot_support)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config gazebo launch ros2_control urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
