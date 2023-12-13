cmake_minimum_required(VERSION 3.5)

project(kuka_lbr_iisy_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files(TARGET iisy3 srdf/lbr_iisy3_r760_arm.srdf.xacro OUTPUT lbr_iisy3_r760.srdf INSTALL DESTINATION urdf)
xacro_add_files(TARGET iisy11 srdf/lbr_iisy11_r1300_arm.srdf.xacro OUTPUT lbr_iisy11_r1300.srdf INSTALL DESTINATION urdf)
xacro_add_files(TARGET iisy15 srdf/lbr_iisy15_r930_arm.srdf.xacro OUTPUT lbr_iisy15_r930.srdf INSTALL DESTINATION urdf)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
