cmake_minimum_required(VERSION 3.0.2)
project(cube_petit_hardware_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#############
## Package ##
#############
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# CPP FILE
add_executable(${PROJECT_NAME}
  src/cube_petit_hardware_interface.cpp
  src/dji_can_communication.cpp
)

# ROS.Hをインクルードするのに必要
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
 )
