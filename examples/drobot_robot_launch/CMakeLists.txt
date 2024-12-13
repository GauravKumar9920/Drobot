cmake_minimum_required(VERSION 3.8)
project(drobot_robot_launch)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/drobot_robot_launch
)

ament_package()
