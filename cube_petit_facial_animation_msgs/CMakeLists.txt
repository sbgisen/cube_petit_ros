cmake_minimum_required(VERSION 3.8)
project(cube_petit_facial_animation_msgs)

if(
  CMAKE_COMPILER_IS_GNUCXX
  OR
    CMAKE_CXX_COMPILER_ID
    MATCHES
    "Clang"
)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files "msg/FaceExpression.msg")

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES
  std_msgs
  builtin_interfaces
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
