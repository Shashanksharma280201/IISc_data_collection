# IISc_data_collection

## Tmux Binding keys: 
- ```Ctrl + A``` is the binding key
- ``` Ctrl + A + H ``` is to split the window Vertically
- ![Screenshot from 2025-04-05 12-44-58](https://github.com/user-attachments/assets/ef593e14-fa33-4a09-a924-6ed4e79b1b4d)

-  ``` Ctrl + A + V ``` is to split the window Horizontally
-  ![Screenshot from 2025-04-05 12-45-22](https://github.com/user-attachments/assets/9be3f45b-577d-4df9-b4df-56a27228092d)

- ``` Ctrl + A + X ``` This is to deletet the pane
- ``` Ctrl + A + D ``` This is to close the session but this wont delete the session 


## Run all the cameras: 
Use the data_colection.sh file 

### First check the cameras are connected:
 ``` 
 v4l2-ctl --list-devices
 ```

### Change device names for mono camera only 
Change the device name in the monocamera_publisher.py file
```
    # Single camera configuration - Based on your working script
    camera_config = {
        "camera_name": "mono_camera",
        "frame_id": "camera_frame",
        "video_device": "/dev/video2", ----------------------------------------> Change this to /dev/videoX
        "framerate": 30.0,
        "image_width": 1920,
        "image_height": 1080,
        "pixel_format": "mjpeg2rgb",  # Must be in supported list
        "io_method": "mmap",
        "brightness": 0,
        "contrast": 48,
    }
```
## We need not change the device name for the ZED cameras 
The ZED SDK assigns the device name automatically to the ZED_Wrapper node.


## LiDAR Setup with Jetson Board (Headless Mode)

This guide explains how to connect and configure a LiDAR sensor (e.g., Hesai, Ouster, or RoboSense) to a Jetson board running in **headless mode** (no monitor, keyboard, or mouse).

## 1. Hardware Connections

* Connect the **LiDAR Ethernet port** to the **Jetson board’s `eth1` port** using a direct Ethernet cable (or via a switch if needed).
* Power up the LiDAR and Jetson board.

## 2. Configure Jetson Ethernet (`eth1`)

Assign a static IP address to the Jetson board so it matches the LiDAR’s subnet.

1. Check available network interfaces:

   ```bash
   ip a
   ```

2. Set static IP for `eth1`:

   ```bash
   sudo ip addr flush dev eth1
   sudo ip addr add 192.168.1.102/24 dev eth1
   sudo ip link set eth1 up
   ```

3. Verify:

   ```bash
   ip a show eth1
   ```

   You should see:

   ```
   inet 192.168.1.102/24
   ```


## 3. LiDAR Default IP

* Most LiDARs ship with default IP **192.168.1.200** (check your device manual).
* Ensure the LiDAR is in the same subnet (`192.168.1.xxx`).


## 4. Test Connectivity

1. Ping the LiDAR:

   ```bash
   ping 192.168.1.200
   ```

   If successful, you will see responses with `0% packet loss`.

2. Check for incoming LiDAR UDP packets (commonly ports `6699`, `7788`, or `2368` depending on the model):

   ```bash
   sudo tcpdump -i eth1 udp
   ```

   If the LiDAR is streaming, you should see UDP packets arriving.


## 5. Headless Access to Jetson

Since the Jetson is headless, connect via **SSH** from your laptop/PC:

```bash
ssh username@192.168.1.102
```

Replace `username` with your Jetson user (e.g., `IISC`).


## 6. Troubleshooting

* If you cannot see packets:

  * Double-check LiDAR IP (`192.168.1.200`) using manufacturer tools.
  * Make sure Jetson and LiDAR are in the same subnet (`192.168.1.xxx`).
  * Disable firewall (if installed):

    ```bash
    sudo systemctl stop ufw
    ```
  * Ensure correct Ethernet cable (use straight or cross cable depending on LiDAR).
* If you want the Jetson IP to persist after reboot, add a **Netplan configuration** or edit `/etc/network/interfaces`.


