#!/bin/bash
robot=(
    /odom
    /cmd_vel
)

mobile_phone=(
    /mobile_phone/imu
)

arducam=(
    /arducam/camera_info
    /arducam/image_raw/compressed
)

zed2_miscellaneous=(
    /zed2/zed_node/atm_press
    /zed2/zed_node/temperature/imu
    /zed2/zed_node/temperature/left
    /zed2/zed_node/temperature/right
)

zed2_cams=(
    /zed2/zed_node/left/camera_info
    /zed2/zed_node/left/image_rect_color/compressed
    /zed2/zed_node/right/camera_info
    /zed2/zed_node/right/image_rect_color/compressed
    /zed2/zed_node/stereo/image_rect_color/compressed
)

zed2_motion=(
    /zed2/zed_node/imu/data
    /zed2/zed_node/odom
    /zed2/zed_node/pose_with_covariance
)

zed2_depth=(
    /zed2/zed_node/disparity/disparity_image
    /zed2/zed_node/point_cloud/cloud_registered
)

zed2=(
    ${zed2_miscellaneous[@]}
    ${zed2_cams[@]}
    ${zed2_motion[@]}
    ${zed2_depth[@]}
)

rosbag record ${robot[@]} ${mobile_phone[@]} ${arducam[@]} ${zed2[@]}
