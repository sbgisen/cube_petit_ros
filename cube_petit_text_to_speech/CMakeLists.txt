cmake_minimum_required(VERSION 3.0.2)
project(cube_petit_text_to_speech)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             rospy
             std_msgs
             tf
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    tf
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
