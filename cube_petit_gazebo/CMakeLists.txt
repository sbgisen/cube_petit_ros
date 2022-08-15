cmake_minimum_required(VERSION 3.0.2)
project(cube_petit_gazebo)

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
  control_toolbox
  message_generation
  std_srvs
  brass_gazebo_battery
)

add_service_files(
  DIRECTORY srv
  FILES
  SetCharging.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# Macro to generate ROS node with "add_executable" and "add_library"
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} brass_gazebo_battery
  CATKIN_DEPENDS
  roscpp
  message_runtime
)


###########
## Build ##
###########

# 自分で作ったヘッダ＋使用するライブラリのヘッダ
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${GAZEBO_INCLUDE_DIRS}
)


# add_executable(${PROJECT_NAME}_node
#   src/battery_charge_simulation.cpp
#   src/docking_station_simulation.cpp
# )

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ### Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#   ${roscpp_LIBRARIES}
#   ${GAZEBO_INCLUDE_DIRS}
# )

add_executable(docking_station_simulation_node
  src/docking_station_simulation.cpp
)
add_dependencies(docking_station_simulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(docking_station_simulation_node
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(battery_charge_simulation_node
  src/battery_charge_simulation.cpp
)
add_dependencies(battery_charge_simulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(battery_charge_simulation_node
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${GAZEBO_INCLUDE_DIRS}
)
