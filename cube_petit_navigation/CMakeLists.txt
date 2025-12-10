cmake_minimum_required(VERSION 3.8)
project(cube_petit_navigation)

find_package(ament_cmake_auto REQUIRED)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
