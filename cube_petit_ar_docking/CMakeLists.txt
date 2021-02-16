cmake_minimum_required(VERSION 3.0.2)
project(cube_petit_ar_docking)


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
  tf2
  tf2_ros
  actionlib
  actionlib_msgs
  ar_track_alvar
  ar_track_alvar_msgs
  message_generation
  move_base_msgs
  std_srvs
)

add_action_files(
    DIRECTORY action FILES ARDocking.action
)

add_service_files(
  FILES
  CubeSpeech.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  actionlib_msgs
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

# "(cube_petit_ar_docking)" node's settings below
# CPP FILE
add_executable(${PROJECT_NAME}_node
   src/ar_docking_server.cpp
   src/ar_docking/pregoal_broadcaster.cpp
   src/ar_docking/battery_current_monitor.cpp
   src/ar_docking/geometry_util.cpp
   src/ar_docking/move_base_action_client.cpp
   src/ar_docking/ar_docking_controller.cpp
   src/ar_docking/speech_util.cpp
)

# ビルド済みのライブラリ
# ROS.Hをインクルードするのに必要
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
