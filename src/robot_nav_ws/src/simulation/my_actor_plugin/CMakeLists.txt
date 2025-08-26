cmake_minimum_required(VERSION 3.8)
project(actor_trajectory_plugin)

# Build type
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED)

# Plugin
add_library(${PROJECT_NAME} SHARED src/actor_trajectory_plugin.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  geometry_msgs
  ignition-gazebo6
)

# Install plugin
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# Install Python script
install(PROGRAMS
  scripts/person_path_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
