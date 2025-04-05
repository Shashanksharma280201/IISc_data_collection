#!/bin/bash

# Session and script configuration
SESSION='data_collection'
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_DIR="/home/flo/logs/zed_recordings"
PYTHON_SCRIPT_1="ros2 launch zed_wrapper dual_zed_cameras.launch.py"
PYTHON_SCRIPT_2="ros2 launch flo_driver cam_launch.py"
#TEST_SCRIPT="/home/gmr/zed-sdk/recording/recording/external_data/python/test.py"

# Create log directory if it doesn't exist
mkdir -p "$LOG_DIR"

# Function to check camera availability
check_cameras() {
    local camera_count=$(ls /dev/video* 2>/dev/null | wc -l)
    if [ "$camera_count" -lt 2 ]; then
        echo "Error: Not enough cameras detected. Found $camera_count cameras."
        return 1
    fi
    return 0
}

# Main recording script
main() {
    # Check camera availability before starting
    if ! check_cameras; then
        echo "Camera check failed. Exiting." >&2
        exit 1
    fi

    # Create tmux session
    tmux new-session -d -s "$SESSION"
    tmux rename-window -t "$SESSION" "Python scripts"
    tmux split-window -h

    # Start recording with logging
    tmux send-keys -t "$SESSION:0.0" "cd ~/zed_ros_ws" C-m
    tmux send-keys -t "$SESSION:0.0" "source install/setup.bash" C-m
    tmux send-keys -t "$SESSION:0.0" "$PYTHON_SCRIPT_1" C-m
    #tmux send-keys -t "$SESSION:0.1" "python3 $PYTHON_SCRIPT_2 --output_svo_file cam2_${TIMESTAMP}.svo 2>&1 | tee $LOG_DIR/cam2_${TIMESTAMP}.log" C-m

    # Wait for 10 seconds before launching test.py
    sleep 10
    tmux send-keys -t "$SESSION:0.1" "cd ~/flo_data_collection" C-m
    tmux send-keys -t "$SESSION:0.1" "source install/setup.bash" C-m
    tmux send-keys -t "$SESSION:0.1" "$PYTHON_SCRIPT_2" C-m

    # Attach to the session
    tmux attach-session -t "$SESSION"
}

# Error handling
trap 'echo "Script interrupted."; exit 1' SIGINT SIGTERM

# Run the main function
main

