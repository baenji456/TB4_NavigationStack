cmake_minimum_required(VERSION 3.8)
project(mapping_nodes)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rtabmap_ros REQUIRED)  # Wichtig: RTAB-Map ROS Integration
find_package(rtabmap_conversions REQUIRED)  # Falls du Konvertierungen brauchst

# Include Directory
include_directories(
  include
)

# Create executable
add_executable(dummy_mapping_node src/dummy_mapping_node.cpp)
ament_target_dependencies(dummy_mapping_node
  rclcpp
  rtabmap_ros
)

# Install executable
install(TARGETS
  dummy_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/dummy_mapping_helper_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
