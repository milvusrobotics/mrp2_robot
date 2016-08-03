cmake_minimum_required(VERSION 2.8.3)
project(mrp2_display)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  gpio.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(mrp2_display_serial src/serial.c)
add_executable(mrp2_display_node src/display.cpp)
target_link_libraries(mrp2_display_node mrp2_display_serial ${catkin_LIBRARIES})
