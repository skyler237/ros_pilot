cmake_minimum_required(VERSION 2.8.3)
project(ros_pilot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  JoyCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_pilot
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs
)

add_definitions(-std=c++11)

###########
## Build ##
###########

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(joy src/joy.cpp)
add_dependencies(joy ros_pilot_generate_messages_cpp)
target_link_libraries(joy ${catkin_LIBRARIES})
