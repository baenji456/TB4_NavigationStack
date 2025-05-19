#!/bin/bash
# Turtlebot Discovery Server Setup

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Ensure clean state
unset ROS_LOCALHOST_ONLY

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
[ -t 0 ] && export ROS_SUPER_CLIENT=True || export ROS_SUPER_CLIENT=False
export ROS_DOMAIN_ID=0
export ROS_DISCOVERY_SERVER="192.168.0.223:11811;"

echo "[TURTLEBOT DISCOVERY] Using discovery server."
echo "  ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
echo "[NOTE] Variables set by other setup scripts are being unset"

