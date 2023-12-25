cmake_minimum_required(VERSION 3.0.2)
project(cube_petit_text_to_speech)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  omron_env_sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cube_petit_text_to_speech
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
#   message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)
