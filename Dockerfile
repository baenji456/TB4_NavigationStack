FROM ros:humble-ros-base-jammy

#RUN apt-get install curl
#RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
#RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

# HOTFIX: Until GPG Keys are updated in the original image
RUN rm /etc/apt/sources.list.d/ros2-latest.list && \
    apt update && apt install curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
#    ros-humble-turtlebot4-navigation \
#    ros-humble-irobot-create-nodes \
    ros-dev-tools \
    ros-humble-rqt* \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-image-geometry \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    python3-pip \
    ros-humble-py-trees \
    ros-humble-py-trees-ros-interfaces \
    ros-humble-py-trees-ros \
    ros-humble-py-trees-ros-tutorials \
    ros-humble-py-trees-ros-viewer \
    && rm -rf /var/lib/apt/lists/*

RUN pip install opencv-python numpy<2.0 ultralytics cv_bridge


# install gazebo
RUN apt-get update && sudo apt-get install wget
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt-get update && sudo apt-get install -y ignition-fortress

# install other dependencies
RUN apt-get update && apt-get install -y libpcl-dev




# Environment setup
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo '#!/usr/bin/env bash' > /ros_entrypoint.sh
RUN echo 'source /opt/ros/humble/setup.bash' >> /ros_entrypoint.sh
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
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/turtlebot4/setup.bash
RUN echo "export ROS_DOMAIN_ID=0" >> /etc/turtlebot4/setup.bash
RUN echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /etc/turtlebot4/setup.bash
RUN echo 'source /etc/turtlebot4_simulation/setup.bash' >> ~/.bashrc

COPY setup/ /etc/

#RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y
#RUN export MAKEFLAGS="-j6" # Can be ignored if you have a lot of RAM (>16GB)
#RUN colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release




ENTRYPOINT ["/ros_entrypoint.sh"]
# Run bash
CMD ["bash"]