#!/bin/bash
#
# Script to run the deployed VSLAM RealSense Docker image
# Based on run_dev.sh from isaac_ros_common
#—-------------------------------------------------------------------------------------------------------
#—-------------------------------------- Author: Jingyang Cui -------------------------------------------
#—-------------------------------------------------------------------------------------------------------

set -e

source ~/Hawkeye-OS/vslam/print_color.sh

# Default image name
DEPLOY_IMAGE_NAME="${1:-vslam_realsense_deploy:latest}"

# Detect platform
PLATFORM="$(uname -m)"

# Allow X11 forwarding for rviz
xhost +local:root > /dev/null 2>&1 || true

CONTAINER_NAME="isaac_ros_visual_slam"

# Re-use existing container.
if [ "$(docker ps -a --quiet --filter status=running --filter name=$CONTAINER_NAME)" ]; then
    print_info "Attaching to running container: $CONTAINER_NAME"
    ISAAC_ROS_WS=$(docker exec $CONTAINER_NAME printenv ISAAC_ROS_WS)
    print_info "Docker workspace: $ISAAC_ROS_WS"
    docker exec -i -t -u admin --workdir $ISAAC_ROS_WS $CONTAINER_NAME /bin/bash $@
    # docker exec -u admin --workdir $ISAAC_ROS_WS $CONTAINER_NAME /bin/bash -c "source /usr/local/bin/scripts/deploy-entrypoint.sh && ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py"
    exit 0
fi

DOCKER_ARGS=()

# Map host's display socket to docker
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
DOCKER_ARGS+=("-e ROS_DOMAIN_ID")
DOCKER_ARGS+=("-e USER")
DOCKER_ARGS+=("-e ISAAC_ROS_WS=/workspaces/isaac_ros-dev")
DOCKER_ARGS+=("-e HOST_USER_UID=`id -u`")
DOCKER_ARGS+=("-e HOST_USER_GID=`id -g`")

# Forward SSH Agent to container if the ssh agent is active.
if [[ -n $SSH_AUTH_SOCK ]]; then
    DOCKER_ARGS+=("-v $SSH_AUTH_SOCK:/ssh-agent")
    DOCKER_ARGS+=("-e SSH_AUTH_SOCK=/ssh-agent")
fi

if [[ $PLATFORM == "aarch64" ]]; then
    DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=nvidia.com/gpu=all,nvidia.com/pva=all")
    DOCKER_ARGS+=("-v /usr/bin/tegrastats:/usr/bin/tegrastats")
    DOCKER_ARGS+=("-v /tmp/:/tmp/")
    DOCKER_ARGS+=("-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra")
    DOCKER_ARGS+=("-v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api")
    DOCKER_ARGS+=("--pid=host")
    DOCKER_ARGS+=("-v /usr/share/vpi3:/usr/share/vpi3")
    DOCKER_ARGS+=("-v /dev/input:/dev/input")

    # If jtop present, give the container access
    if [[ $(getent group jtop) ]]; then
        DOCKER_ARGS+=("-v /run/jtop.sock:/run/jtop.sock:ro")
    fi
fi
# Device access for I2C (IMU) and RealSense
DOCKER_ARGS+=("--device /dev/i2c-7:/dev/i2c-7")
DOCKER_ARGS+=("--group-add $(stat -c %g /dev/i2c-7)")
DOCKER_ARGS+=("-v /dev:/dev")

# Mount to local
DOCKER_ARGS+=("-v ${ISAAC_ROS_WS}/src/isaac_ros_visual_slam/isaac_ros_visual_slam/launch/isaac_ros_visual_slam_realsense.launch.py:/workspaces/isaac_ros-dev/install/share/isaac_ros_visual_slam/launch/isaac_ros_visual_slam_realsense.launch.py")

# Timezone
DOCKER_ARGS+=("-v /etc/localtime:/etc/localtime:ro")

print_info "Running deployed image: $DEPLOY_IMAGE_NAME"
print_info "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"

#docker run -it --rm \
docker run --rm \
    --runtime nvidia \
    --privileged \
    --network host \
    --ipc=host \
    --name "isaac_ros_visual_slam" \
    ${DOCKER_ARGS[@]} \
    --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \
    "$DEPLOY_IMAGE_NAME" \
    /bin/bash -c "source /usr/local/bin/scripts/deploy-entrypoint.sh && ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py"
    