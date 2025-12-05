ARG TARGETARCH
FROM osrf/ros:humble-desktop-full

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

# Imaging setup - copy conanfile to the WORKDIR and install dependencies
COPY ArenaSDK_v0.1.104_Linux_x64.tar.gz .
RUN tar -xzvf ArenaSDK_v0.1.104_Linux_x64.tar.gz -C /tmp
RUN mv /tmp/ArenaSDK_Linux_x64 /opt/arena_sdk

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
