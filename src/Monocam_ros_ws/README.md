# Monocamera Publisher Package

A ROS2 Humble package for publishing monocamera topics with RViz visualization.

## Package Structure

```
monocamera_publisher/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── monocamera_launch.py
├── scripts/
│   └── camera_info_publisher.py
├── config/
│   └── monocamera_rviz.rviz
└── README.md
```

## Features

- USB camera integration using `usb_cam` package
- Camera info publishing with calibration parameters
- Static transform publishing for camera frame
- Pre-configured RViz2 visualization with camera view and image display
- Configurable camera parameters (resolution, frame rate, device path)

## Dependencies

Make sure you have the following packages installed:

```bash
sudo apt update
sudo apt install ros-humble-usb-cam ros-humble-image-transport ros-humble-cv-bridge ros-humble-rviz2
```

## Installation

1. Create a ROS2 workspace if you don't have one:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone or create this package in your workspace:
```bash
# If cloning from repository
git clone <repository_url> monocamera_publisher

# Or create the package structure manually and copy the files
mkdir -p monocamera_publisher/{launch,scripts,config}
# Copy all the provided files to their respective directories
```

3. Make the Python script executable:
```bash
chmod +x monocamera_publisher/scripts/camera_info_publisher.py
```

4. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select monocamera_publisher
source install/setup.bash
```

## Usage

### Launch the complete system:
```bash
ros2 launch monocamera_publisher monocamera_launch.py
```

This will start:
- USB camera node
- Camera info publisher
- Static transform publisher
- RViz2 with camera visualization

### Launch components individually:

**USB Camera only:**
```bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p camera_name:=mono_camera -p video_device:=/dev/video0
```

**Camera info publisher only:**
```bash
ros2 run monocamera_publisher camera_info_publisher.py
```

**RViz2 with custom config:**
```bash
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/monocamera_publisher/config/monocamera_rviz.rviz
```

## Configuration

### Camera Parameters

Edit the `camera_config` dictionary in `launch/monocamera_launch.py` to match your camera:

```python
camera_config = {
    "camera_name": "mono_camera",
    "framerate": 30.0,
    "frame_id": "camera_link",
    "image_width": 1920,
    "image_height": 1080,
    "video_device": "/dev/video0",  # Change this to your camera device
    "pixel_format": "mjpeg2rgb",
    "io_method": "mmap",
    "brightness": 0,
    "contrast": 48
}
```

### Camera Device Detection

Find your camera device:
```bash
ls /dev/video*
v4l2-ctl --list-devices
```

### Troubleshooting

**Camera not found:**
- Check if camera is connected: `lsusb`
- Check video devices: `ls /dev/video*`
- Test camera with: `ffplay /dev/video0`

**Permission denied:**
- Add user to video group: `sudo usermod -a -G video $USER`
- Logout and login again

**No image in RViz:**
- Check topic list: `ros2 topic list`
- Check image topic: `ros2 topic echo /mono_camera/image_raw --field data`
- Verify camera info: `ros2 topic echo /mono_camera/camera_info`

## Topics Published

- `/mono_camera/image_raw` - Raw camera images
- `/mono_camera/image_compressed` - Compressed images
- `/mono_camera/camera_info` - Camera calibration information
- `/tf_static` - Static transform for camera frame

## Customization

### Adding Multiple Cameras

To add more cameras, modify the launch file to include additional camera configurations similar to your reference code:

```python
cam_names = ["cam0", "cam1", "cam2"]
configs = {
    "cam0": {...},
    "cam1": {...},
    "cam2": {...}
}
```

### Camera Calibration

For better results, calibrate your camera using:
```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/mono_camera/image_raw camera:=/mono_camera
```

Then update the camera matrix values in `scripts/camera_info_publisher.py`.

## License

Apache-2.0