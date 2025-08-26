cmake_minimum_required(VERSION 3.8)
project(human_trajectory_costmap_layer)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  include
)

# --------- Messages (GENERATE FIRST) ----------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PredictedPosition.msg"
  "msg/PredictedPositionArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# --------- Executable ----------
add_executable(dummy_human_prediction_node
  src/dummy_human_prediction_node.cpp
)
ament_target_dependencies(dummy_human_prediction_node
  rclcpp std_msgs geometry_msgs
)
# critical: ensure build order vs. generated msgs
rosidl_target_interfaces(dummy_human_prediction_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# --------- Plugin Library ----------
add_library(human_trajectory_costmap_layer_core SHARED
  src/human_trajectory_layer.cpp
)
ament_target_dependencies(human_trajectory_costmap_layer_core
  rclcpp std_msgs geometry_msgs nav2_costmap_2d pluginlib
)
rosidl_target_interfaces(human_trajectory_costmap_layer_core
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

pluginlib_export_plugin_description_file(
  nav2_costmap_2d human_trajectory_costmap_layer_plugins.xml
)

install(TARGETS
  dummy_human_prediction_node
  human_trajectory_costmap_layer_core
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  human_trajectory_costmap_layer_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mdn_dummy_prediction_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
