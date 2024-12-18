cmake_minimum_required(VERSION 3.8)
project(apriltag_dock)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED COMPONENTS profiler)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(apriltag_dock_msgs REQUIRED)
# find_package(gz-sim REQUIRED)

# Set version variables
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

include_directories(
  include/${PROJECT_NAME}
)

message(STATUS "Compiling against Gazebo Harmonic")
add_library(TfPlugin SHARED src/TfPlugin.cpp)
target_include_directories(TfPlugin PRIVATE include)

target_link_libraries(TfPlugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

set(CTRL_EXEC_NAME "controller")
set(CLIENT_EXEC_NAME "docking_client")
add_executable( ${CTRL_EXEC_NAME} src/controller.cpp src/service_state.cpp)
add_executable( ${CLIENT_EXEC_NAME} src/service_client_node.cpp)

set(DEPENDENCIES
  "rclcpp"           
  "std_msgs"         
  "geometry_msgs" 
  "tf2_geometry_msgs"
  "tf2"
  "tf2_ros"
  "sensor_msgs"
  "apriltag_dock_msgs"
)

ament_target_dependencies(${CTRL_EXEC_NAME} ${DEPENDENCIES})
ament_target_dependencies(${CLIENT_EXEC_NAME} ${DEPENDENCIES})

ament_target_dependencies(TfPlugin
  tf2_ros
  tf2
  sensor_msgs
  rclcpp
  geometry_msgs
  std_msgs
)

# Install targets
install(TARGETS ${CTRL_EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${CLIENT_EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS TfPlugin
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)
  
install(
  DIRECTORY launch param rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()