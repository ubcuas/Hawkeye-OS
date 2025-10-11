ARG TARGETARCH
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Source ROS setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
