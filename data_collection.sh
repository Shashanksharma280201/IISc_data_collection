#!/bin/bash

SESSION='data_collection'
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_DIR="/home/flo/logs/zed_recordings"

PYTHON_SCRIPT_1="ros2 launch zed_wrapper dual_zed_cameras.launch.py"
PYTHON_SCRIPT_2="ros2 launch monocamera_publisher monocamera_launch.py "
PYTHON_SCRIPT_3="ros2 launch rslidar_sdk start.py"
PYTHON_SCRIPT_4="python3 folder_recording.py"

mkdir -p "$LOG_DIR"

check_cameras() {
    local camera_count
    camera_count=$(ls /dev/video* 2>/dev/null | wc -l)
    if [ "$camera_count" -lt 2 ]; then
        echo "Error: Not enough cameras detected. Found $camera_count cameras."
        return 1
    fi
    return 0
}

main() {
    if ! check_cameras; then
        echo "Camera check failed. Exiting." >&2
        exit 1
    fi

    # Start tmux session and first window
    tmux new-session -d -s "$SESSION" -n "Python scripts"

    # Pane 0 (default)
    tmux send-keys -t "$SESSION:0.0" "bash" C-m
    tmux send-keys -t "$SESSION:0.0" "cd ~/IISc_data_collection" C-m
    tmux send-keys -t "$SESSION:0.0" "source install/setup.bash" C-m
    tmux send-keys -t "$SESSION:0.0" "$PYTHON_SCRIPT_1" C-m

    # Pane 1 (split horizontally)
    tmux split-window -h -t "$SESSION:0.0"
    tmux send-keys -t "$SESSION:0.1" "bash" C-m
    tmux send-keys -t "$SESSION:0.1" "cd ~/IISc_data_collection" C-m
    tmux send-keys -t "$SESSION:0.1" "source install/setup.bash" C-m
    tmux send-keys -t "$SESSION:0.1" "$PYTHON_SCRIPT_3" C-m

    # Pane 2 (split below pane 0)
    tmux select-pane -t "$SESSION:0.0"
    tmux split-window -v
    tmux send-keys -t "$SESSION" "bash" C-m
    tmux send-keys -t "$SESSION" "cd ~/IISc_data_collection" C-m
    tmux send-keys -t "$SESSION" "source install/setup.bash" C-m
    tmux send-keys -t "$SESSION" "$PYTHON_SCRIPT_2" C-m

    # sleep 5
    # Pane 3 (split below pane 1)
    tmux select-pane -t "$SESSION:0.1"
    tmux split-window -v
    tmux send-keys -t "$SESSION" "bash" C-m
    tmux send-keys -t "$SESSION" "cd ~/IISc_data_collection" C-m
    # tmux send-keys -t "$SESSION" "source install/setup.bash" C-m
    # tmux send-keys -t "$SESSION" "$PYTHON_SCRIPT_4" C-m

    tmux select-layout tiled \; attach-session -t "$SESSION"
}

trap 'echo "Script interrupted."; exit 1' SIGINT SIGTERM
main
