#!/bin/bash

source ~/.bashrc
if command -v docker-compose &> /dev/null; then
docker-compose build 
docker-compose up -d 
else
docker compose build
docker compose up -d
fi

SESSION=Project
SESSIONEXISTS=(tmux list-sessions | grep $SESSION)

if [ "$SESSIONEXISTS" = "" ] 
then
tmux new-session -d -s $SESSION 

tmux rename-window -t 0 'Main'

tmux new-window -t $SESSION:1 -n 'Orchestrator'
if command -v docker-compose &> /dev/null; then
tmux send-keys -t $SESSION: 'source ~/.bashrc' C-m 'PROGRAM=orchestrator' C-m '
docker-compose exec -it ros2_workspace bash -lc "
source /opt/ros/humble/setup.bash &&
cd /ros2_ws &&
colcon build && 
source install/setup.bash &&
ros2 run orchestrator orchestrator"' C-m
else
tmux send-keys -t $SESSION: 'source ~/.bashrc' C-m 'PROGRAM=orchestrator' C-m '
docker compose exec -it ros2_workspace bash -lc "
source /opt/ros/humble/setup.bash &&
cd /ros2_ws &&
colcon build && 
source install/setup.bash &&
ros2 run orchestrator $PROGRAM "' C-m
fi

tmux new-window -t $SESSION:2 -n 'Image-Capture'
if command -v docker-compose &> /dev/null; then
tmux send-keys -t $SESSION: 'source ~/.bashrc' C-m 'PROGRAM=mock_image_capture' C-m '
docker-compose exec -it ros2_workspace bash -lc "
source /opt/ros/humble/setup.bash &&
cd /ros2_ws &&
colcon build && 
source install/setup.bash &&
ros2 run orchestrator orchestrator"' C-m
else
tmux send-keys -t $SESSION: 'source ~/.bashrc' C-m 'PROGRAM=mock_image_capture' C-m '
docker compose exec -it ros2_workspace bash -lc "
source /opt/ros/humble/setup.bash &&
cd /ros2_ws &&
colcon build &&
source install/setup.bash &&
ros2 run orchestrator $PROGRAM "' C-m
fi

tmux new-window -t $SESSION:3 -n 'Object-Detection'
if command -v docker-compose &> /dev/null; then
tmux send-keys -t $SESSION: 'source ~/.bashrc' C-m 'PROGRAM=mock_object_detection' C-m '
docker-compose exec -it ros2_workspace bash -lc "
source /opt/ros/humble/setup.bash &&
cd /ros2_ws &&
colcon build && 
source install/setup.bash &&
ros2 run orchestrator orchestrator"' C-m
else
tmux send-keys -t $SESSION: 'source ~/.bashrc' C-m 'PROGRAM=mock_object_detection' C-m '
docker compose exec -it ros2_workspace bash -lc "
source /opt/ros/humble/setup.bash &&
cd /ros2_ws &&
colcon build && 
source install/setup.bash &&
ros2 run orchestrator $PROGRAM "' C-m
fi
fi

tmux attach-session -t $SESSION:0

tmux kill-sess -t Project

