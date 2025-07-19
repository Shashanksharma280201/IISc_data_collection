#!/bin/bash

SESSION='video_recording'
VIDEO_DIR="/home/flo/videos"
SCRIPT_DIR="$HOME/IISc_data_collection"

mkdir -p "$VIDEO_DIR/zed" "$VIDEO_DIR/cam1" "$VIDEO_DIR/cam2"

check_cameras() {
    local camera_count
    camera_count=$(ls /dev/video* 2>/dev/null | wc -l)
    if [ "$camera_count" -lt 2 ]; then
        echo "Error: Not enough cameras detected. Found $camera_count cameras."
        return 1
    fi
    return 0
}

start_zed_recording() {
    gst-launch-1.0 -e zedsrc camera-resolution=1 camera-fps=30 ! \
    videoconvert ! x264enc ! h264parse ! \
    splitmuxsink location="/home/flo/videos/zed/zed_%05d.mp4" max-size-time=600000000000
}

start_cam1_recording() {
    gst-launch-1.0 -e v4l2src device=/dev/video1 ! \
    videoconvert ! x264enc ! h264parse ! \
    splitmuxsink location="/home/flo/videos/cam1/cam1_%05d.mp4" max-size-time=600000000000
}

start_cam2_recording() {
    gst-launch-1.0 -e v4l2src device=/dev/video2 ! \
    videoconvert ! x264enc ! h264parse ! \
    splitmuxsink location="/home/flo/videos/cam2/cam2_%05d.mp4" max-size-time=600000000000
}

main() {
    if ! check_cameras; then
        echo "Camera check failed. Exiting." >&2
        exit 1
    fi

    tmux new-session -d -s "$SESSION" -n "Recordings"

    # Pane 0: ZED Recording
    tmux send-keys -t "$SESSION:0.0" "cd $SCRIPT_DIR" C-m
    tmux send-keys -t "$SESSION:0.0" "bash" C-m
    tmux send-keys -t "$SESSION:0.0" "gst-launch-1.0 -e zedsrc camera-resolution=1 camera-fps=30 ! \
                                        videoconvert ! x264enc ! h264parse ! \
                                        splitmuxsink location="/home/flo/videos/zed/zed_%05d.mp4" max-size-time=600000000000" C-m

    # Pane 1: Cam1 Recording
    tmux split-window -h -t "$SESSION:0.0"
    tmux send-keys -t "$SESSION:0.1" "cd $SCRIPT_DIR" C-m
    tmux send-keys -t "$SESSION:0.0" "gst-launch-1.0 -e v4l2src device=/dev/video1 ! \
                                        videoconvert ! x264enc ! h264parse ! \
                                        splitmuxsink location="/home/flo/videos/cam1/cam1_%05d.mp4" max-size-time=600000000000" C-m

    # Pane 2: Cam2 Recording
    tmux select-pane -t "$SESSION:0.0"
    tmux split-window -v
    tmux send-keys -t "$SESSION" "cd $SCRIPT_DIR" C-m
    tmux send-keys -t "$SESSION:0.0" "bash" C-m
    tmux send-keys -t "$SESSION" "gst-launch-1.0 -e v4l2src device=/dev/video2 ! \
                                    videoconvert ! x264enc ! h264parse ! \
                                    splitmuxsink location="/home/flo/videos/cam2/cam2_%05d.mp4" max-size-time=600000000000" C-m

    tmux select-layout tiled \; attach-session -t "$SESSION"
}

trap 'echo "Script interrupted."; exit 1' SIGINT SIGTERM
main
