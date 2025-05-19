FROM ros:jazzy-ros-base-noble

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-turtlebot4-simulator \
    ros-jazzy-turtlebot4-desktop \
    ros-jazzy-turtlebot4-navigation \
    ros-jazzy-irobot-create-nodes \
    ros-dev-tools \
    ros-jazzy-rqt* \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*


# install gazebo#
RUN apt-get install curl
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install gz-harmonic

# install other dependencies
RUN apt-get update && apt-get install -y libpcl-dev




# Environment setup
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
RUN echo '#!/usr/bin/env bash' > /ros_entrypoint.sh
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /ros_entrypoint.sh
#RUN echo "echo 'Container starting with entrypoint'"  >> /ros_entrypoint.sh
#RUN echo "rosdep update && rosdep install --from-paths src --ignore-src -r -y" >> /ros_entrypoint.sh
#RUN echo "colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release" >> /ros_entrypoint.sh
#RUN echo "source home install/setup.bash" >> /ros_entrypoint.sh
RUN echo 'exec "$@"' >> /ros_entrypoint.sh
#RUN echo "echo 'Entrypoint script executed'" >> /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh


# Configure Container for use with real TB4 (https://turtlebot.github.io/turtlebot4-user-manual/setup/simple_discovery.html)
RUN sudo mkdir /etc/turtlebot4/
RUN sudo touch /etc/turtlebot4/setup.bash
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/turtlebot4/setup.bash
RUN echo "export ROS_DOMAIN_ID=0" >> /etc/turtlebot4/setup.bash
RUN echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /etc/turtlebot4/setup.bash
RUN echo 'source /etc/turtlebot4/setup.bash' >> ~/.bashrc


ENTRYPOINT ["/ros_entrypoint.sh"]
# Run bash
CMD ["bash"]