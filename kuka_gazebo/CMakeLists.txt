cmake_minimum_required(VERSION 3.8)
project(kuka_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(test_keep_alive test/gazebo_test_keep_alive.cpp)
ament_target_dependencies(test_keep_alive rclcpp std_msgs)

install(
  DIRECTORY gazebo launch world config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS test/gazebo_support_test.py test/run_gazebo_tests.py test/test_gazebo_robot_support.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
message(STATUS "GITHUB_ACTIONS in CMake: $ENV{GITHUB_ACTIONS}")
option(GITHUB_CI_TESTS "Enable tests for GitHub CI" OFF)

# If CI tests are disabled, do not install README.md unnecessarily
# (it could also break binary package build, where root folder is package root)
if(GITHUB_CI_TESTS)
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../README.md
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(TARGETS
  test_keep_alive
  DESTINATION lib/${PROJECT_NAME})

if(GITHUB_CI_TESTS)
  message(STATUS "Gazebo test environment variable detected, enabling gazebo tests")
  find_package(launch_testing_ament_cmake REQUIRED)
  enable_testing()
  add_launch_test(test/test_gazebo_robot_support.py)
endif()

ament_package()
