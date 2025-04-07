#!/bin/bash

# Start a new tmux session named 'navigator'
tmux new-session -d -s flo_data_collection

# First window: launch the hardware launch file
tmux send-keys 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys 'source ~/flo_data_collection/install/setup.bash' C-m
tmux send-keys 'ros2 launch flo_driver cam_launch.py' C-m

# Split window horizontally and run the enable service call
tmux split-window -h
tmux send-keys 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys 'source install/setup.bash' C-m
tmux send-keys 'ros2 bag record /cam0/image_compressed /cam1/image_compressed /scan /imu' C-m

# Attach to the tmux session
tmux attach-session -t flo_data_collection
