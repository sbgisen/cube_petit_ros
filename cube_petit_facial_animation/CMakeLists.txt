cmake_minimum_required(VERSION 3.8)
project(cube_petit_facial_animation)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(
  DIRECTORY launch frontend
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS cube_petit_facial_animation/animation.py cube_petit_facial_animation/expression_operator.py
  DESTINATION lib/${PROJECT_NAME}
)
