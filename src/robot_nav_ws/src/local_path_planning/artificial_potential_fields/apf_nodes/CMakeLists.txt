cmake_minimum_required(VERSION 3.8)
project(apf_nodes)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# Add C++ executable(s)
add_executable(apf_planner_node src/dummy_apf_node.cpp)
ament_target_dependencies(apf_planner_node rclcpp std_msgs geometry_msgs)

# Install targets
install(TARGETS apf_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/dummy_apf_script.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
