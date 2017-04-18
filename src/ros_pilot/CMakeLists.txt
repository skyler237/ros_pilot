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
  fcu_common
  dynamic_reconfigure
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

# Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
  cfg/Follower.cfg
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

add_executable(ros_pilot_joy src/joy.cpp)
add_dependencies(ros_pilot_joy ros_pilot_generate_messages_cpp)
target_link_libraries(ros_pilot_joy ${catkin_LIBRARIES})

add_executable(ros_pilot_controller
            include/ros_pilot/controller_base.h
            include/ros_pilot/controller_example.h
            src/controller_base.cpp
            src/controller_example.cpp
)
add_dependencies(ros_pilot_controller ros_pilot_generate_messages_cpp fcu_common_generate_messages_cpp)
target_link_libraries(ros_pilot_controller ${catkin_LIBRARIES})
