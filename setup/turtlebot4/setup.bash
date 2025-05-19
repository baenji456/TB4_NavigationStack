#!/bin/bash
# Turtlebot Standalone Setup

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Ensure clean state
unset ROS_LOCALHOST_ONLY
unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "[TURTLEBOT STANDALONE] Using normal ROS 2 setup (no discovery server)."
echo "[NOTE] Variables set by other setup scripts are being unset"
