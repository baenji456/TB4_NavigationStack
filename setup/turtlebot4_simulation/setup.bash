#!/bin/bash
# Host simulation setup script - isolated ROS 2 networking for simulation

# Source ROS 2 Jazzy environment
source /opt/ros/jazzy/setup.bash

# Unset discovery-related variables (set by turtlebot_discovery)
unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT

# Configure networking for simulation
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1

echo "[SIMULATION SETUP] Using localhost-only ROS 2 networking."
echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "[NOTE] Variables set by other setup scripts are being unset"

