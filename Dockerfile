ARG TARGETARCH
FROM ros:humble

# Build arguments for user configuration
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Create non-root user with sudo
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    libprotobuf-dev \
    protobuf-compiler \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Install development tools
RUN apt-get update && apt-get install -y \
    git \
    vim \
    wget \
    curl \
    build-essential \
    gdb \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    && /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh \
    && rm -rf /var/lib/apt/lists/*

# Create workspace with build and install directories
RUN mkdir -p /ros2_ws/src /ros2_ws/build /ros2_ws/install
WORKDIR /ros2_ws

# Install Python packages for WebRTC
RUN pip3 install aiortc av opencv-python websockets numpy

# Configure ROS environment for new user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Set workspace ownership
RUN chown -R $USERNAME:$USERNAME /ros2_ws

# Switch to non-root user
USER $USERNAME

CMD ["/bin/bash"]
