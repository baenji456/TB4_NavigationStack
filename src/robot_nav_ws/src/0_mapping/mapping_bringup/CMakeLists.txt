cmake_minimum_required(VERSION 3.8)
project(mapping_bringup)

find_package(ament_cmake REQUIRED)

# Install Launch, Config, and RViz files
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
