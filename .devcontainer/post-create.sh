#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

sudo pip3 install --quiet git+https://github.com/openai/CLIP.git

sudo chown -R ros:ros /ros2_ws/build /ros2_ws/install 2>/dev/null || true

sudo apt-get update
rosdep update
sudo rosdep install --from-paths src --ignore-src -y

sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

colcon build --symlink-install
