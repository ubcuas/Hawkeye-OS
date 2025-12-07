ARG TARGETARCH
FROM ros:humble


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


# add build-time dependencies for imaging
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3-pip libssl-dev nghttp2 \
    libopencv-dev libcurl4-openssl-dev tzdata pkg-config && \
    rm -rf /var/lib/apt/lists/*


# ensure pip user path is available
ENV PATH="/root/.local/bin:${PATH}"


# Copy imaging source code to workspace
COPY src/imaging /ros2_ws/src/imaging

# Set up paths expected by build_imaging.sh
RUN mkdir -p /workspace/ros2_pubsub/src && \
    ln -s /ros2_ws/src/imaging /workspace/ros2_pubsub/src/imaging && \
    mkdir -p /workspace/external

# Copy Arena SDK to the location expected by build script
COPY src/imaging/external/ArenaSDK_v0.1.78_Linux_ARM64.tar.gz /workspace/external/

# Copy and run the imaging build script
COPY src/imaging/build_imaging.sh /tmp/build_imaging.sh
RUN chmod +x /tmp/build_imaging.sh && /tmp/build_imaging.sh


# Source ROS setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc


# copy compiled binaries to a known path
RUN mkdir -p /opt/hawkeye
RUN cp /ros2_ws/src/imaging/build/camerafeed /opt/hawkeye/ || true


CMD ["/bin/bash"]


