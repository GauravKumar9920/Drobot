cmake_minimum_required(VERSION 3.8)
project(drobot_demos)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/drobot_demos
)

ament_package()
