cmake_minimum_required(VERSION 3.10.2)
project(cube_petit_navigation_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Disable Wredundant-decls warnings since rosidl generates redundant function
#  declarations
if(
  CMAKE_COMPILER_IS_GNUCXX
  OR
    CMAKE_CXX_COMPILER_ID
    MATCHES
    "Clang"
)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-redundant-decls")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(srv_files "srv/SavePlace.srv")

set(msg_files "msg/NavigationState.msg")

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
  DEPENDENCIES
  ${${PROJECT_NAME}_BUILD_DEPENDS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
