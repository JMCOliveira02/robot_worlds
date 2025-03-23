cmake_minimum_required(VERSION 3.5)
project(robot_worlds)

# Set C++ standard if not already specified
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#
# ─── FIND REQUIRED PACKAGES ─────────────────────────────────────
#
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(pluginlib REQUIRED)                  # For Webots plugin support
find_package(webots_ros2_driver REQUIRED)         # Webots ROS 2 driver
find_package(yaml-cpp REQUIRED)                   # YAML file parsing
find_package(ament_index_cpp REQUIRED)            # To find shared package paths

#
# ─── INCLUDE HEADERS ────────────────────────────────────────────
#
include_directories(
  include
)

#
# ─── INSTALL HEADER FILES ───────────────────────────────────────
#
install(
  DIRECTORY include/
  DESTINATION include
)

#
# ─── ROBOTCONTROLLER PLUGIN (for Webots) ────────────────────────
#

# Export the plugin XML so Webots knows how to load it
pluginlib_export_plugin_description_file(webots_ros2_driver robot_controller.xml)

# Build the RobotController as a shared library (for pluginlib)
add_library(RobotController SHARED
  src/RobotController.cpp
)

# Link dependencies to the plugin library
ament_target_dependencies(RobotController
  rclcpp
  pluginlib
  webots_ros2_driver
  nav_msgs
)

# Include headers for the plugin target
target_include_directories(RobotController
  PRIVATE
    include
)

# Install the shared library
install(TARGETS
  RobotController
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#
# ─── FAKE DETECTOR NODE ─────────────────────────────────────────
#

# Build a standalone executable for the fake feature detector
add_executable(fake_detector
  src/FakeDetector.cpp
)

# Link its dependencies
ament_target_dependencies(fake_detector
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  yaml-cpp
  ament_index_cpp
)

target_link_libraries(fake_detector yaml-cpp)

# Install the fake_detector binary
install(TARGETS
  fake_detector
  DESTINATION lib/${PROJECT_NAME}
)

#
# ─── INSTALL RESOURCE DIRECTORIES ───────────────────────────────
#

# Install useful resource directories (for launch, maps, features, etc.)
install(DIRECTORY
  launch
  urdf
  maps
  feature_maps
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

#
# ─── EXPORTS FOR DOWNSTREAM PACKAGES ────────────────────────────
#

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

#
# ─── PACKAGE DECLARATION ────────────────────────────────────────
#

ament_package()
