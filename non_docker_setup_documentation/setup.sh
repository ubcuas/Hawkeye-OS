#!/bin/bash
set -e

echo "=== 1. Setting up system dependencies and ROS 2 Humble ==="
sudo apt-get update
sudo apt-get install -y curl gnupg2 lsb-release

# Ensure ROS 2 repo is added (if not already present on JP6)
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update
sudo apt-get install -y \
    ros-humble-desktop \
    python3-rosdep \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    libprotobuf-dev \
    protobuf-compiler \
    libopencv-dev \
    libgl1 \
    libglib2.0-0 \
    libopenblas-dev \
    libopenmpi-dev \
    libomp-dev \
    git vim wget build-essential gdb \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-camera-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-message-filters

echo "=== 2. Initializing rosdep and geographiclib ==="
sudo rosdep init || echo "rosdep already initialized"
rosdep update
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

echo "=== 3. Setting up NVIDIA CUDSS ==="
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get install -y cudss
rm cuda-keyring_1.1-1_all.deb

# echo "=== 4. Creating Virtual Environment ==="
# sudo apt install -y python3.10-venv
# if [ ! -d "$HOME/hawkeye_venv" ]; then
#     python3 -m venv --system-site-packages "$HOME/hawkeye_venv"
#     echo "Created venv at $HOME/hawkeye_venv"
# fi
# source "$HOME/hawkeye_venv/bin/activate"

echo "=== 5. Installing Pip Requirements (into VENV) ==="
# Pip upgrade
pip install --upgrade pip

# General requirements from repo
pip install --no-cache-dir -r requirements.txt
pip install --no-cache-dir git+https://github.com/openai/CLIP.git

# Jetson ML packages (JetPack 6 optimized)
pip install pyrealsense2
pip uninstall -y torch torchvision || true
pip install --no-cache-dir --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 torch torchvision



echo "=== 6. Building ROS workspace ==="
# Source base ROS
source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo "=== Setup complete! ==="
echo "To develop, run:"
echo "source /opt/ros/humble/setup.bash"
echo "source ~/hawkeye_venv/bin/activate"
echo "source install/setup.bash"