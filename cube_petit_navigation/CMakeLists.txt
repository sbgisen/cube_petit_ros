cmake_minimum_required(VERSION 3.8)
project(cube_petit_navigation)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

install(
  PROGRAMS cube_petit_navigation/navigation_api_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  map
)
