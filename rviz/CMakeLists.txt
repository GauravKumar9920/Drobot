cmake_minimum_required(VERSION 3.5)
project(rviz)

# Find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY drobot_rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
