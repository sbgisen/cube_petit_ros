cmake_minimum_required(VERSION 3.8)
project(cube_petit_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID
      MATCHES
      "Clang"
)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(class_loader REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(dji_ros_controller REQUIRED)

# Include directories
include_directories(
  include
  # ${CMAKE_CURRENT_SOURCE_DIR}/../dji_ros_controller/include
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${hardware_interface_INCLUDE_DIRS}
  ${controller_interface_INCLUDE_DIRS}
  ${rmw_INCLUDE_DIRS}
  ${rosidl_typesupport_interface_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${can_msgs_INCLUDE_DIRS}
  ${controller_manager_INCLUDE_DIRS}
  ${class_loader_INCLUDE_DIRS}
  ${controll_toolbox_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../dji_ros_controller/include
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME} src/cube_petit_hardware_interface.cpp)

# add_executable(${PROJECT_NAME}_node src/M2006_ros2.cpp)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  control_toolbox
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
