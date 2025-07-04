cmake_minimum_required(VERSION 3.5)

project(kuka_lbr_iiwa_support)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.dsv")

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/test_lbr_iiwa.py)
endif()

ament_package()
