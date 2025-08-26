cmake_minimum_required(VERSION 3.8)
project(nav2_oaa_costmap_plugin)

set(lib_name ${PROJECT_NAME}_core)

# === Language / Warnings ===
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# === Build ===
add_library(${lib_name} SHARED
  src/prediction_layer.cpp
)

target_include_directories(${lib_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${lib_name}
  rclcpp
  pluginlib
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# === Plugin description ===
pluginlib_export_plugin_description_file(nav2_costmap_2d prediction_layer.xml)

# === Install ===
install(TARGETS ${lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES prediction_layer.xml
  DESTINATION share/${PROJECT_NAME}
)

# === Ament package ===
ament_package()
