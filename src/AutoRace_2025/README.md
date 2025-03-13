# AutoRace 2025 Repository

Welcome to the AutoRace 2025 repository! This project is part of the AutoRace autonomous driving challenge, aiming to develop and showcase advanced autonomous driving capabilities using ROS 2 and Docker.

---

## Repository Structure

This repository is structured to modularize various components required for the autonomous driving challenge. Below is an overview of the key directories and their purposes:

```plaintext
AutoRace_2025/
├── docker/                # Docker setup scripts for the development environment
│   ├── build.sh           # Build the Docker image
│   ├── run.sh             # Run the Docker container
├── workspace/             # ROS 2 workspace containing source code and build outputs
│   └── src/               # Source code for ROS 2 packages
│       ├── camera/        # Fisheye camera package
│       └── ld08_driver/   # LDS-02 LiDAR
└── README.md              # This file
```

---

## Setting Up the Environment

### 1. Build and Run the Docker Environment

Navigate to the `docker` directory and execute the following commands:

```bash
cd AutoRace_2025/docker/
./build.sh
./run.sh
```

This will build and run the Docker environment for development and testing.

### 2. Build the ROS 2 Workspace

Navigate to the `workspace` directory and build the ROS 2 workspace:

```bash
cd workspace/
colcon build
source install/setup.bash
```

This ensures all ROS 2 packages are built and ready for execution.

---

## Fisheye Camera Package

### Quick Start

To start using the fisheye camera package, follow these steps:

1. **Run the Camera Node**:
   ```bash
   ros2 run camera fish_eye_camera_120
   ```

2. **Run the LiDAR Node**:
   ```bash
   ros2 launch ld08_driver ld08.launch.py
   ```

3. **Verify Output**:
   Check the image stream being published to the `/rgb` topic:
   ```bash
   ros2 topic echo /rgb
   ```

   Check the LiDAR being published to the `/scan` topic:
   ```bash
   ros2 topic echo /scan
   ```

### Optional Configuration

To customize camera parameters such as resolution, frame rate, or device path, modify the JSON configuration file:

```bash
workspace/src/camera/json/camera_config.json
```

Example:
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

For more detailed setup instructions, including how to configure your camera device with `udevadm` or inspect its capabilities, refer to the package-level README in the `camera/` directory.
