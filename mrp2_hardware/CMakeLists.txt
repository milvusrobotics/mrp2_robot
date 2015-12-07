cmake_minimum_required(VERSION 2.8.3)
project(mrp2_hardware)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  hardware_interface
  realtime_tools
  roscpp
  rospy
  tf
  urdf
  std_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/Parameters.cfg
)

catkin_package(
  CATKIN_DEPENDS
    controller_interface
    controller_manager
    hardware_interface
    realtime_tools
    roscpp
    tf
    urdf
    std_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS gazebo
)

catkin_package()

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_library(mrp2_serial src/mrp2_serial.cpp src/rs232.c)
add_executable(mrp2_hardware src/mrp2_hardware.cpp)
add_dependencies(mrp2_hardware ${PROJECT_NAME}_gencfg)

target_link_libraries(mrp2_hardware mrp2_serial	${catkin_LIBRARIES})