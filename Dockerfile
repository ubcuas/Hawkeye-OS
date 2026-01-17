# Force ARM64 architecture for Apple Silicon / Raspberry Pi
FROM --platform=linux/arm64 osrf/ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    libgeographic-dev \
    protobuf-compiler \
    libprotobuf-dev \
    geographiclib-tools \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

## IMAGING SETUP ##

# Set ArenaSDK version and architecture
ARG ARENA_SDK_VERSION=0.1.78
# Hardcode to ARM64 since we're building for that platform
RUN echo "ARM64" > /tmp/arena_arch

# Copy the appropriate ArenaSDK based on architecture
COPY src/imaging/external/ /tmp/arena_external/

# Extract and install ArenaSDK conditionally
RUN ARENA_ARCH=$(cat /tmp/arena_arch) && \
    ARENA_TAR="ArenaSDK_v${ARENA_SDK_VERSION}_Linux_${ARENA_ARCH}.tar.gz" && \
    echo "Looking for: /tmp/arena_external/$ARENA_TAR" && \
    if [ -f "/tmp/arena_external/$ARENA_TAR" ]; then \
        echo "Found ArenaSDK for $ARENA_ARCH architecture" && \
        tar -xzvf "/tmp/arena_external/$ARENA_TAR" -C /tmp && \
        mv "/tmp/ArenaSDK_Linux_${ARENA_ARCH}" /opt/arena_sdk; \
    else \
        echo "WARNING: ArenaSDK not found for $ARENA_ARCH architecture" && \
        echo "Available files:" && ls -la /tmp/arena_external/ && \
        mkdir -p /opt/arena_sdk/lib; \
    fi

# configure library paths so the arena SDK libraries can be found
# Use find to automatically discover all Linux64* lib directories (works for ARM, x64, etc.)
RUN echo "/opt/arena_sdk/lib" > /etc/ld.so.conf.d/Arena_SDK.conf && \
    echo "/opt/arena_sdk/lib64" >> /etc/ld.so.conf.d/Arena_SDK.conf && \
    find /opt/arena_sdk/GenICam/library/lib -type d -name "Linux64*" \
      -exec bash -c 'echo "{}" >> /etc/ld.so.conf.d/Arena_SDK.conf' \;

# update dynamic linker cache so system can find our libraries
RUN ldconfig

# set up metavision SDK (bundled w/ arena SDK, need symlinks for version compatibility)
RUN if [ -d /opt/arena_sdk/Metavision/lib ]; then \
      echo "/opt/arena_sdk/Metavision/lib" > /etc/ld.so.conf.d/Metavision_SDK.conf && \
      cd /opt/arena_sdk/Metavision/lib && \
      ln -sf libmetavision_sdk_core.so.4.6.2 libmetavision_sdk_core.so.4 && \
      ln -sf libmetavision_sdk_base.so.4.6.2 libmetavision_sdk_base.so.4 && \
      ldconfig; \
    else \
      echo "No Metavision SDK found for this architecture, skipping"; \
    fi

RUN ls -la ./src

# Source ROS setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
