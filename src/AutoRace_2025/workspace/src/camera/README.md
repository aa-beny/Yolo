# FishEyeCamera120 ROS 2 Package

This package allows users to interface with a fisheye camera, configure its parameters, and publish its image stream to a ROS 2 topic. The configuration is managed via a JSON file for flexibility.

---

### 1. Configure Camera Device with `udevadm` (Local Machine Setup)

This step must be performed on the local machine (not inside Docker) to ensure proper device recognition and mapping:

1. **Find Your Camera's Device Name**:
   Plug in your camera and check the system-assigned name:

   ```bash
   v4l2-ctl --list-devices
   ```

2. **Create a Udev Rule**:
   Write a custom rule to link your camera device to `/dev/mycamera`. Replace `ATTR{idVendor}` and `ATTR{idProduct}` with your camera's vendor and product IDs:

   ```bash
   sudo nano /etc/udev/rules.d/99-custom-camera.rules
   ```

   Add the following line to the file:

   ```bash
   KERNEL=="video*", ATTR{idVendor}=="1234", ATTR{idProduct}=="5678", SYMLINK+="mycamera"
   ```

3. **Reload Udev Rules**:
   Apply the changes and verify the device:

   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ls -l /dev/mycamera
   ```

   Example output:

   ```bash
   lrwxrwxrwx 1 root root 6 Jan 18 00:45 /dev/mycamera -> video0
   ```

4. **Inspect Camera Capabilities**:
   To further inspect the camera's supported formats and resolutions, use the following command:

   ```bash
   v4l2-ctl --list-formats-ext -d /dev/video0
   ```

   Example output:

   ```plaintext
   ioctl: VIDIOC_ENUM_FMT
   Type: Video Capture

   [0]: 'MJPG' (Motion-JPEG, compressed)
       Size: Discrete 1920x1080
           Interval: Discrete 0.033s (30.000 fps)
       Size: Discrete 1280x720
           Interval: Discrete 0.033s (30.000 fps)
       Size: Discrete 1280x1024
           Interval: Discrete 0.033s (30.000 fps)
       Size: Discrete 320x240
           Interval: Discrete 0.033s (30.000 fps)
       Size: Discrete 640x480
           Interval: Discrete 0.033s (30.000 fps)
       Size: Discrete 800x600
           Interval: Discrete 0.033s (30.000 fps)
       Size: Discrete 1024x768
           Interval: Discrete 0.033s (30.000 fps)
   [1]: 'YUYV' (YUYV 4:2:2)
       Size: Discrete 1920x1080
           Interval: Discrete 0.200s (5.000 fps)
       Size: Discrete 1280x720
           Interval: Discrete 0.111s (9.000 fps)
       Size: Discrete 1280x1024
           Interval: Discrete 0.167s (6.000 fps)
       Size: Discrete 320x240
           Interval: Discrete 0.033s (30.000 fps)
       Size: Discrete 640x480
           Interval: Discrete 0.033s (30.000 fps)
       Size: Discrete 800x600
           Interval: Discrete 0.048s (21.000 fps)
       Size: Discrete 1024x768
           Interval: Discrete 0.077s (13.000 fps)
   ```

   Use this information to configure your camera parameters accordingly.

---

### 2. Configure Camera Parameters

Edit the JSON configuration file (`camera/json/camera_config.json`) to set the desired camera parameters:

```json
{
    "camera_device": "/dev/mycamera",
    "frame_width": 1920,
    "frame_height": 1080,
    "fps": 30,
    "fourcc": "MJPG",
    "topic_name": "rgb",
    "node_name": "fish_eye_camera_120"
}
```

| **Key**         | **Description**                                      |
| --------------- | ---------------------------------------------------- |
| `camera_device` | Path to your camera device (e.g., `/dev/mycamera`).  |
| `frame_width`   | Width of the captured video frames.                  |
| `frame_height`  | Height of the captured video frames.                 |
| `fps`           | Frames per second to capture.                        |
| `fourcc`        | Video codec (e.g., `MJPG`).                          |
| `topic_name`    | ROS 2 topic to publish the image frames.             |
| `node_name`     | Name of the ROS 2 node running the camera publisher. |

---

### 3. Run the Node

Launch the node to start publishing image frames:

```bash
ros2 run camera fish_eye_camera_120
```

Verify that the node is publishing on the specified topic:

```bash
ros2 topic echo /rgb
```

