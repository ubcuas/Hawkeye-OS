# L4T base with CUDA — match r36.x to your JetPack 6.x version
# Check your host: cat /etc/nv_tegra_release
FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

# Build arguments for user configuration
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install ROS Humble on top of L4T base
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
       http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y ros-humble-ros-base python3-rosdep \
    && rosdep init \
    && rm -rf /var/lib/apt/lists/*

# maybe not ros base

# Create non-root user with sudo
RUN apt-get clean && rm -rf /var/lib/apt/lists/* \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG root,video,render $USERNAME \
    && apt-get update -o Acquire::AllowInsecureRepositories=true -o Acquire::AllowDowngradeToInsecureRepositories=true \
    && apt-get install -y --allow-unauthenticated sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    libprotobuf-dev \
    protobuf-compiler \
    libopencv-dev \
    libgl1 \
    libglib2.0-0 \
    libopenblas-dev \
    libopenmpi-dev \
    libomp-dev \
    git \
    vim \
    wget \
    curl \
    build-essential \
    gdb \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-camera-msgs \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-message-filters \
    && /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /ros2_ws/src /ros2_ws/build /ros2_ws/install
WORKDIR /ros2_ws

COPY ./setup_env.sh /ros2_ws/setup_env.sh

# Install Python packages
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt
RUN pip3 install --no-cache-dir git+https://github.com/openai/CLIP.git

# Use Jetson torch
RUN pip3 uninstall -y torch torchvision || true
RUN pip3 install --no-cache-dir --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 torch torchvision

# CUDA / cuDSS setup
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb \
    && apt-get update \
    && apt-get install -y cudss \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install pyrealsense2

# Set the Ultralytics config directory to a writable location
# Disable auto-install so ultralytics doesn't try to re-install CLIP at runtime
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