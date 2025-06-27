cmake_minimum_required(VERSION 3.8)
project(kuka_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

include_directories(include)

add_executable(gazebo_moveit_example src/gazebo_lbr_iisy3_r760_moveit_example.cpp)
ament_target_dependencies(gazebo_moveit_example
  moveit_ros_planning_interface
  moveit_visual_tools
)

install(TARGETS
  gazebo_moveit_example
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY gazebo launch world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
