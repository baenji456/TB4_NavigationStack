# Mapping Packages

This folder contains all mapping-related packages, divided into external libraries and custom development.

## Structure

mapping/
├── extern/
│ ├── rtabmap/ # External library (submodule)
│ ├── rtabmap_ros/ # External library (submodule)
├── mapping_nodes/ # Custom C++ and Python nodes
│ ├── src/ # C++ source files
│ ├── include/ # C++ headers
│ ├── scripts/ # Python scripts
├── mapping_bringup/ # Launch, config, and RViz setup
│ ├── launch/ # Launch files
│ ├── config/ # YAML configuration files
│ ├── rviz/ # RViz visualization setups



## Package Description

- **extern/**: External ROS2 packages added as Git submodules (not modified directly).
- **mapping_nodes/**: Custom-developed nodes in C++ and Python.
- **mapping_bringup/**: Launch files, configurations, and RViz setups to run and visualize the mapping system.

> Note: Dummy files (e.g., `dummy_mapping_node.cpp`, `dummy_mapping_launch.py`) are placeholders and should be replaced with actual implementations.
