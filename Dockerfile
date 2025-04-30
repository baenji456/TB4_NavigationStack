FROM ros:humble-ros-base-jammy

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-turtlebot4-simulator \
    ros-humble-turtlebot4-navigation \
    ros-humble-irobot-create-nodes \
    ros-dev-tools \
    ros-humble-rqt* \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*


# install gazebo
RUN sudo apt-get update && sudo apt-get install wget
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo apt-get update && sudo apt-get install -y ignition-fortress

# install other dependencies
RUN apt-get update && apt-get install -y libpcl-dev

# Build workspace
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN source install/setup.bash


# Environment setup
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo '#!/usr/bin/env bash' > /ros_entrypoint.sh
RUN echo 'source /opt/ros/humble/setup.bash' >> /ros_entrypoint.sh
RUN echo 'exec "$@"' >> /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
# Run bash
CMD ["bash"]