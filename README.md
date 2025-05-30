# Kinefly Docker Container

This repository contains a Dockerized version of Kinefly, a ROS-based system for real-time wing tracking. The container is built on Ubuntu 16.04 with ROS Kinetic.

## Prerequisites

- Docker installed on your system
- X11 server (for GUI applications)
- Git (for cloning the repository)

## Quick Start Guide

### 1. Clone the Repository

```bash
git clone https://github.com/pvnkmrksk/Kinefly_docker.git
cd Kinefly_docker
```

### 2. Build the Docker Image

```bash
docker build -t kinefly .
```



Before running the container, allow X server connections:

```bash
xhost +local:docker
```

### 3. Run the Container

There are two ways to run the container:

#### Option 1: Using the Default Entry Point

This will run the container with the default ROS launch configuration:

```bash
docker run -it \
    --privileged \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    kinefly
```

#### Option 2: Interactive Shell Access

If you want to access the container's shell instead of running the default ROS launch:

```bash
docker run -it \
    --privileged \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --entrypoint /bin/bash \
    kinefly
```

### 4. Enable X11 Display (Required for GUI)

## ROS ZMQ Bridge Usage

The `ros_zmq_bridge.py` script bridges ROS messages from Kinefly to ZMQ for external applications (like Unity). This allows real-time communication between the ROS-based Kinefly system and other applications.

### ✨ New Improved Message Format

The bridge now outputs a well-structured JSON message with meaningful headers:

```json
{
  "message_info": {
    "source": "kinefly",
    "message_type": "wing_tracking",
    "frame_number": 445,
    "timestamp_ros": 1748610967.524399,
    "timestamp_unix": 1748610967.524399,
    "frame_id": "Fly"
  },
  "wing_tracking": {
    "left_wing": {
      "angle_radians": -0.2618,
      "angle_degrees": -15.00,
      "beat_frequency_hz": 0.0,
      "tracking_confidence": 0.309,
      "gradient": 0.0
    },
    "right_wing": {
      "angle_radians": 0.0654,
      "angle_degrees": 3.75,
      "beat_frequency_hz": 0.0,
      "tracking_confidence": 0.478,
      "gradient": 0.0
    }
  },
  "body_parts": {
    "head": {
      "angle_radians": 0.0,
      "tracking_confidence": 0.573,
      "radius": 0.0
    },
    "abdomen": {
      "angle_radians": 0.0,
      "tracking_confidence": 0.263,
      "radius": 0.0
    }
  },
  "legacy_unity_format": {
    "x": -0.2618,
    "y": 0.0654,
    "z": 0.0,
    "yaw": 0.0,
    "pitch": 0.0,
    "roll": 0.0
  }
}
```

### Running Inside Docker (Recommended)

The easiest approach is to run the bridge inside the Docker container where ROS and Kinefly are already configured:

**Note**: You'll need to rebuild the Docker image to include the latest bridge scripts and dependencies:

```bash
# Rebuild the Docker image with bridge scripts included
docker build -t kinefly .
```

#### Method 1: Using the Helper Script (Easiest)

```bash
# Start container
docker run -it --privileged --net=host --name kinefly_container kinefly

# Inside the container, use the helper script
cd /opt/Kinefly_docker
./start_bridge.sh

# Or with custom port
./start_bridge.sh --port 9872

# Or with custom topic
./start_bridge.sh --topic /custom/flystate
```

#### Method 2: Manual Setup

```bash
# Start container with shell access and network host mode for ZMQ
docker run -it \
    --privileged \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --entrypoint /bin/bash \
    kinefly
```

#### 2. Inside the Container, Run the Bridge

```bash
# Navigate to the bridge script location
cd /opt/Kinefly_docker

# Set up environment
source /opt/ros/kinetic/setup.bash
source /root/catkin/devel/setup.bash
export PYTHONPATH=/root/catkin/src/Kinefly/src:$PYTHONPATH

# Run the bridge with default settings
python2 ros_zmq_bridge.py

# Or with custom settings
python2 ros_zmq_bridge.py --zmq-url "tcp://*:9872" --topic "/custom/flystate"
```

### Testing the Bridge

Test the bridge connection from your host system:

```bash
# Copy the test script to your host (optional)
docker cp kinefly_container:/opt/Kinefly_docker/test_zmq_client.py .

# Run the test (requires Python 3 with pyzmq)
python3 test_zmq_client.py
```

Expected output with new format:
```
📊 === Message #1 ===
🔖 Source: kinefly
📋 Type: wing_tracking
🎬 Frame: 445
🕒 Timestamp: 1748610967.524399
🦋 Left Wing:
   📐 Angle: -0.2618 rad (-15.00°)
   🎯 Confidence: 0.309
   🎵 Frequency: 0.00 Hz
🦋 Right Wing:
   📐 Angle: 0.0654 rad (3.75°)
   🎯 Confidence: 0.478
   🎵 Frequency: 0.00 Hz
🗣️  Head: confidence=0.573
🔵 Abdomen: confidence=0.263
🎮 Legacy Unity Format: x=-0.2618, y=0.0654
```

## Socket ZMQ Republisher (FicTrac Bridge)

The `socket_zmq_republisher.py` script bridges UDP data from FicTrac to ZMQ. This is useful when you have FicTrac running separately and want to republish its data over ZMQ for Unity or other applications.

### Running the FicTrac Bridge

#### Basic Usage

```bash
# Run with default settings (UDP on 127.0.0.1:1111, ZMQ on port 9871)
python3 socket_zmq_republisher.py

# Or with Python 2 (if needed)
python socket_zmq_republisher.py
```

#### Advanced Usage

```bash
# Custom settings
python3 socket_zmq_republisher.py \
    --zmq-url "tcp://*:9872" \
    --sock-host "0.0.0.0" \
    --sock-port 2222 \
    --timeout 5

# See all available options
python3 socket_zmq_republisher.py --help
```

#### Command Line Options

- `--zmq-url`: ZMQ binding URL (default: `tcp://*:9871`)
- `--sock-host`: UDP socket host IP (default: `127.0.0.1`)
- `--sock-port`: UDP socket port (default: `1111`)
- `--timeout`: Timeout for receiving data from FicTrac (default: `1` second)

### FicTrac Bridge Output Format

The bridge converts FicTrac UDP data to JSON format over ZMQ:

```json
{
    "x": 0.0,      // X position from FicTrac
    "y": 0.0,      // Y position from FicTrac
    "z": 0.0,      // Fixed to 0.0
    "yaw": 0.0,    // Yaw angle in degrees (converted from radians)
    "pitch": 0.0,  // Fixed to 0.0
    "roll": 0.0    // Fixed to 0.0
}
```

### Troubleshooting FicTrac Bridge

1. **No data received**:
   - Verify FicTrac is sending UDP data: `nc -u -l 1111` (listen on port)
   - Check FicTrac configuration for UDP output
   - Ensure firewall allows UDP traffic on the specified port

2. **Connection issues**:
   - Try binding to `0.0.0.0` instead of `127.0.0.1` for external connections
   - Check if the UDP port is already in use: `netstat -an | grep 1111`

3. **Data format errors**:
   - Ensure FicTrac is outputting the expected format (starts with "FT", comma-separated)
   - Check FicTrac version compatibility

## Container Features

- ROS Kinetic with full desktop installation
- OpenCV with Python bindings
- Phidgets support
- LED panels support
- USB camera support
- GUI tools (guvcview, gedit)

## Troubleshooting

1. If you see X11 connection errors:
   - Make sure you've run the `xhost` command
   - Verify your DISPLAY environment variable is set correctly

2. If USB devices are not detected:
   - Ensure the container is run with `--privileged` flag
   - Check USB device permissions on the host system

## Development

To modify the launch files or make changes to the configuration:

1. The custom launch files are located in `launch_files/rhag/`
2. After making changes, rebuild the Docker image

## Notes

- The container uses ROS Kinetic which is based on Ubuntu 16.04
- GUI applications are supported through X11 forwarding
- USB devices are accessible through the `--privileged` flag
- The container includes all necessary dependencies for Kinefly operation

## License


Please refer to the original Kinefly repository for license information.

## Camera Configuration

Kinefly supports both USB cameras (via `usb_cam`) and FireWire cameras (via `camera1394`). The default configuration in this repository uses USB cameras with optimized settings for wing tracking.

### Current Camera Configuration

The camera is configured with the following optimized settings for high-quality wing tracking:

**Camera Parameters:**
- **Video Device**: `/dev/video4` (adjustable)
- **Resolution**: 800x600 pixels (higher than default)
- **Format**: MJPEG (better compression than YUYV)
- **Framerate**: 30 FPS
- **Namespace**: `rhag_camera`

**Image Processing:**
- **Auto-exposure**: Disabled (manual control)
- **Auto-gain**: Disabled (manual control)
- **Auto-white-balance**: Disabled (manual control)
- **Manual settings**: Optimized for consistent lighting

### Testing Your Camera

#### Method 1: Quick Camera Test

```bash
# Test camera availability and settings
./test_camera.sh

# Or inside Docker container
docker exec -it kinefly_container /opt/Kinefly_docker/test_camera.sh
```

#### Method 2: Manual Camera Testing

```bash
# Check available video devices
ls -la /dev/video*

# Test camera with v4l2 tools
v4l2-ctl --device=/dev/video4 --all
v4l2-ctl --device=/dev/video4 --list-formats-ext

# Test camera with guvcview (GUI)
guvcview -d /dev/video4
```

### Customizing Camera Settings

#### Changing Video Device

If your camera is on a different device (e.g., `/dev/video0`):

1. **Edit the launch file**:
   ```xml
   <!-- In launch_files/rhag/camera_1394.launch -->
   <param name="video_device" type="string" value="/dev/video0"/>
   ```

2. **Rebuild Docker image** (for persistence):
   ```bash
   docker build -t kinefly .
   ```

#### Adjusting Camera Parameters

Common parameters you might want to adjust:

```xml
<!-- Resolution -->
<param name="image_width" type="int" value="640"/>    <!-- Lower for speed -->
<param name="image_height" type="int" value="480"/>   <!-- Lower for speed -->

<!-- Framerate -->
<param name="framerate" type="int" value="60"/>       <!-- Higher for tracking -->

<!-- Manual exposure control -->
<param name="exposure_absolute" value="100"/>         <!-- Adjust for lighting -->
<param name="gain" value="20"/>                       <!-- Adjust for brightness -->

<!-- Pixel format -->
<param name="pixel_format" value="yuyv"/>             <!-- Alternative format -->
```

### Camera Launch Commands

#### Starting the Camera Only

```bash
# Inside Docker container
source /opt/ros/kinetic/setup.bash
source /root/catkin/devel/setup.bash

# Launch camera
roslaunch Kinefly camera_1394.launch rig:=rhag

# Check camera topics
rostopic list | grep camera
rostopic echo /rhag_camera/camera/image_raw --once
```

#### Starting Full Kinefly with Camera

```bash
# Start complete system
roslaunch Kinefly main_1394.launch rig:=rhag

# Or with custom camera parameters
roslaunch Kinefly main_1394.launch rig:=rhag camera_device:=/dev/video0
```

### Camera Troubleshooting

#### 1. Camera Not Detected

```bash
# Check USB connections
lsusb | grep -i camera

# Check video devices
ls -la /dev/video*

# Check permissions
sudo usermod -a -G video $USER  # On host
# Or run container with --privileged (recommended)
```

#### 2. Permission Denied Errors

```bash
# Run Docker with proper permissions
docker run -it --privileged --net=host kinefly

# Or on host, check device permissions
sudo chmod 666 /dev/video4
```

#### 3. Camera Format Issues

```bash
# Check supported formats
v4l2-ctl --device=/dev/video4 --list-formats-ext

# Test different pixel formats in launch file:
# mjpeg (best compression, higher quality)
# yuyv (raw format, lower CPU usage)
# rgb24 (full color, highest CPU usage)
```

#### 4. Poor Image Quality

1. **Adjust manual exposure**:
   ```xml
   <param name="exposure_absolute" value="200"/>  <!-- Increase for brighter -->
   ```

2. **Adjust gain**:
   ```xml
   <param name="gain" value="15"/>  <!-- Increase for sensitivity -->
   ```

3. **Enable auto-exposure temporarily**:
   ```xml
   <param name="auto_exposure" value="1"/>  <!-- Let camera auto-adjust -->
   ```

4. **Check physical setup**:
   - Ensure adequate lighting
   - Clean camera lens
   - Minimize motion blur with faster shutter

#### 5. Performance Issues

1. **Lower resolution**:
   ```xml
   <param name="image_width" type="int" value="640"/>
   <param name="image_height" type="int" value="480"/>
   ```

2. **Lower framerate**:
   ```xml
   <param name="framerate" type="int" value="15"/>
   ```

3. **Change pixel format**:
   ```xml
   <param name="pixel_format" value="yuyv"/>  <!-- Less CPU intensive -->
   ```

### Advanced Camera Configuration

#### Multiple Cameras

For multi-camera setups, duplicate the camera launch file with different namespaces:

```xml
<!-- camera_1394_cam2.launch -->
<launch>
    <node name="camera2" pkg="usb_cam" type="usb_cam_node" ns="rhag_camera2" output="screen">
        <param name="video_device" type="string" value="/dev/video1"/>
        <!-- ... other parameters ... -->
    </node>
</launch>
```

#### Camera Calibration

For accurate tracking, calibrate your camera:

```bash
# Create calibration directory
mkdir -p ~/.ros/camera_info

# Run ROS camera calibration
rosrun camera_calibration cameracalibrator.py \
    --size 8x6 \
    --square 0.108 \
    image:=/rhag_camera/camera/image_raw \
    camera:=/rhag_camera/camera
```

### Camera Specifications

**Recommended USB Cameras:**
- **Logitech C920/C922**: Excellent MJPEG support, good low-light performance
- **Microsoft LifeCam**: Reliable drivers, good auto-focus
- **Generic UVC cameras**: Wide compatibility, various price points

**Requirements:**
- **UVC (USB Video Class) compatibility**: For driver-free operation
- **Manual exposure control**: Essential for consistent tracking
- **MJPEG support**: For efficient compression
- **High frame rate capability**: 30+ FPS at desired resolution 