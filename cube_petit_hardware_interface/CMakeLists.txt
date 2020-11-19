cmake_minimum_required(VERSION 3.0.2)
project(cube_petit_hardware_interface)


#############
## Package ##
#############

# CMake building options
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
## Find required libraries and add them into include path
# package.xml の<build_depend>と同じものをかく
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  hardware_interface
  controller_manager
)

# Macro to generate ROS node with "add_executable" and "add_library"
catkin_package()

###########
## Build ##
###########

# 自分で作ったヘッダ＋使用するライブラリのヘッダ
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# "cube_petit_hardware_interface" node's settings below
# CPP FILE
add_executable(${PROJECT_NAME}_node
  src/cube_petit_hardware_interface.cpp
  src/dji_can_communication.cpp
)

# ビルド済みのライブラリ
# ROS.Hをインクルードするのに必要
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
