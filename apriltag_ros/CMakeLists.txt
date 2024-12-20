cmake_minimum_required(VERSION 3.8)
project(apriltag_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(apriltag 3.2 REQUIRED)


if(cv_bridge_VERSION VERSION_GREATER_EQUAL 3.3.0)
    add_compile_definitions(cv_bridge_HPP)
endif()

include_directories(
  include
)

add_library(tags OBJECT src/tag_functions.cpp)
target_link_libraries(tags apriltag::apriltag)
set_property(TARGET tags PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(main src/main.cpp src/apriltag_node.cpp)
ament_target_dependencies(main rclcpp sensor_msgs apriltag_msgs tf2_ros image_transport cv_bridge)
target_link_libraries(main apriltag::apriltag tags Eigen3::Eigen)

ament_target_dependencies(main
  rclcpp
  cv_bridge
  # other_dependencies
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

install(TARGETS 
  main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  params
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()