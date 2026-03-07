FROM ros:humble

# Build arguments for user configuration
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Create non-root user with sudo
RUN apt-get clean && rm -rf /var/lib/apt/lists/* \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update -o Acquire::AllowInsecureRepositories=true -o Acquire::AllowDowngradeToInsecureRepositories=true \
    && apt-get install -y --allow-unauthenticated sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    libgl1 \
    libglib2.0-0 \
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

# Install Intel RealSense ROS2 wrapper and related dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-camera-msgs \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-message-filters \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /ros2_ws/src /ros2_ws/build /ros2_ws/install 
WORKDIR /ros2_ws

COPY ./setup_env.sh /ros2_ws/setup_env.sh

# Install Python packages
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

# Install CLIP — use OpenAI's canonical package which correctly installs the 'clip' module
# (ultralytics/CLIP fork has a broken pyproject.toml that installs as UNKNOWN-0.0.0)
RUN pip3 install --no-cache-dir git+https://github.com/openai/CLIP.git

# set the Ultralytics config directory to a writable location
# disable auto-install so ultralytics doesn't try to re-install CLIP at runtime
ENV YOLO_CONFIG_DIR=/tmp/Ultralytics
ENV YOLO_AUTOINSTALL=False

# Configure ROS environment for new user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Set workspace ownership
RUN chown -R $USERNAME:$USERNAME /ros2_ws

# Switch to non-root user
USER $USERNAME

CMD ["/bin/bash"]