cmake_minimum_required(VERSION 3.0.2)
project(jbd_battery_monitor)

#############
## Package ##
#############

# CMake building options
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
## Find required libraries and add them into include path
# package.xml の<build_depend>と同じものをかく
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             rospy
             std_msgs
)

# Macro to generate ROS node with "add_executable" and "add_library"
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
)

###########
## Build ##
###########

# 自分で作ったヘッダ＋使用するライブラリのヘッダ
include_directories(include ${catkin_INCLUDE_DIRS})

# "cube_petit_hardware_interface" node's settings below
# CPP FILE
add_executable(${PROJECT_NAME}_node src/jbd_battery_monitor.cpp src/serial_communication.cpp)

# ビルド済みのライブラリ
# ROS.Hをインクルードするのに必要
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
