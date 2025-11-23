# Kinefly Multi-Camera Setup

This setup allows you to run multiple Kinefly instances simultaneously, each processing a different video source and publishing data on different ZMQ ports.

## Overview

The multi-camera setup includes:
- **Camera 1**: `/dev/video4` → ZMQ Port `9871` (default)
- **Camera 2**: `/dev/video6` → ZMQ Port `9872` (default)

Each camera instance runs in its own ROS namespace to prevent conflicts.

## Directory Structure

```
launch/
├── rhag/           # Original single camera setup
├── rhag_cam1/      # Camera 1 configuration
└── rhag_cam2/      # Camera 2 configuration

Scripts:
├── dev-kinefly.sh          # Original single camera
├── dev-kinefly-cam1.sh     # Camera 1 Docker wrapper
├── dev-kinefly-cam2.sh     # Camera 2 Docker wrapper
├── dev-kinefly-dual.sh     # Both cameras in same container
├── start-kinefly-cam1.sh   # Camera 1 startup script (inside container)
├── start-kinefly-cam2.sh   # Camera 2 startup script (inside container)
└── start-kinefly-dual.sh   # Dual camera startup script (inside container)
```

## Quick Start

### Option 1: Run Single Camera (Original)
```bash
./dev-kinefly.sh [PORT]
```

### Option 2: Run Camera 1 Only
```bash
./dev-kinefly-cam1.sh [PORT]
```

### Option 3: Run Camera 2 Only
```bash
./dev-kinefly-cam2.sh [PORT]
```

### Option 4: Run Both Cameras in Separate Containers

**Terminal 1:**
```bash
./dev-kinefly-cam1.sh 9871
```

**Terminal 2:**
```bash
./dev-kinefly-cam2.sh 9872
```

### Option 5: Run Both Cameras in Same Container (NEW!)
```bash
./dev-kinefly-dual.sh [CAM1_PORT] [CAM2_PORT]
# Example:
./dev-kinefly-dual.sh 9871 9872
# Or with defaults:
./dev-kinefly-dual.sh
```

**Inside the container, you can also run:**
```bash
kinefly-dual [CAM1_PORT] [CAM2_PORT]
```

## Configuration Details

### Camera 1 (`rhag_cam1`)
- **Video Device**: `/dev/video4`
- **ROS Namespace**: `kinefly_cam1`
- **Camera Namespace**: `rhag_cam1`
- **Topic**: `/kinefly_cam1/kinefly_cam1/flystate`
- **Default ZMQ Port**: `9871`

### Camera 2 (`rhag_cam2`)
- **Video Device**: `/dev/video6`
- **ROS Namespace**: `kinefly_cam2`
- **Camera Namespace**: `rhag_cam2`
- **Topic**: `/kinefly_cam2/kinefly_cam2/flystate`
- **Default ZMQ Port**: `9872`

## Dual Camera Startup Sequence

When using the dual camera option, the script:

1. 🚀 **Starts ROS Master**
2. 🎬 **Launches Camera 1** (`/dev/video4`)
3. ⏳ **Waits for Camera 1** to initialize (~8 seconds)
4. 🌉 **Starts Camera 1 ZMQ Bridge** (port 9871)
5. 🎬 **Launches Camera 2** (`/dev/video6`)  
6. ⏳ **Waits for Camera 2** to initialize (~8 seconds)
7. 🌉 **Starts Camera 2 ZMQ Bridge** (port 9872)
8. 🎉 **Both cameras running!**

## Testing Connections

### Test Camera 1
```bash
python3 /opt/Kinefly_docker/test_zmq_client.py --zmq-url tcp://localhost:9871
```

### Test Camera 2
```bash
python3 /opt/Kinefly_docker/test_zmq_client.py --zmq-url tcp://localhost:9872
```

## ROS Topics

When both cameras are running, you'll see these topics:

```bash
# Camera 1 topics
/kinefly_cam1/kinefly_cam1/flystate
/rhag_cam1/camera/image_raw
/rhag_cam1/camera/image_mono

# Camera 2 topics
/kinefly_cam2/kinefly_cam2/flystate
/rhag_cam2/camera/image_raw
/rhag_cam2/camera/image_mono
```

## Container Aliases

Inside any Kinefly container, you can use:

```bash
kinefly [PORT]         # Original single camera
kinefly-cam1 [PORT]    # Camera 1 only  
kinefly-cam2 [PORT]    # Camera 2 only
kinefly-dual [P1] [P2] # Both cameras with custom ports
status                 # Show active topics
test-data              # Test original camera data
test-cam1              # Test Camera 1 data
test-cam2              # Test Camera 2 data
```

## Troubleshooting

### Check Available Video Devices
```bash
ls -la /dev/video*
```

### Check if Ports are in Use
```bash
netstat -tlnp | grep :9871
netstat -tlnp | grep :9872
```

### View Logs (Dual Camera Setup)
```bash
# Inside container
tail -f /tmp/kinefly_cam1.log
tail -f /tmp/zmq_bridge_cam1.log
tail -f /tmp/kinefly_cam2.log
tail -f /tmp/zmq_bridge_cam2.log
```

### Kill All Kinefly Processes
```bash
# Kill all ROS processes
pkill -f roslaunch
pkill -f roscore
pkill -f rosmaster

# Kill ZMQ bridges
pkill -f ros_zmq_bridge
```

## Customizing Video Sources

To use different video devices, edit the camera configuration files:

**For Camera 1:**
```bash
# Edit: launch/rhag_cam1/camera_1394.launch
<param name="video_device" type="string" value="/dev/video4"/>
```

**For Camera 2:**
```bash
# Edit: launch/rhag_cam2/camera_1394.launch
<param name="video_device" type="string" value="/dev/video6"/>
```

## Adding More Cameras

To add Camera 3:

1. **Create launch directory:**
   ```bash
   mkdir launch/rhag_cam3
   cp -r launch/rhag/* launch/rhag_cam3/
   ```

2. **Update configuration:**
   - Edit `launch/rhag_cam3/camera_1394.launch` (change video device and namespace)
   - Edit `launch/rhag_cam3/_main.launch` (change namespace and topic)
   - Edit `launch/rhag_cam3/source_live.launch` (change namespaces)

3. **Create startup scripts:**
   ```bash
   cp start-kinefly-cam2.sh start-kinefly-cam3.sh
   cp dev-kinefly-cam2.sh dev-kinefly-cam3.sh
   ```

4. **Update the scripts** to use `rhag_cam3`, `/dev/video8`, port `9873`, etc.

## Integration with External Applications

Each camera publishes data independently:

```python
import zmq
import json

# Connect to Camera 1
context1 = zmq.Context()
socket1 = context1.socket(zmq.SUB)
socket1.connect("tcp://localhost:9871")
socket1.setsockopt(zmq.SUBSCRIBE, b"")

# Connect to Camera 2
context2 = zmq.Context()
socket2 = context2.socket(zmq.SUB)
socket2.connect("tcp://localhost:9872")
socket2.setsockopt(zmq.SUBSCRIBE, b"")

while True:
    # Non-blocking receive from both cameras
    try:
        data1 = json.loads(socket1.recv_string(zmq.NOBLOCK))
        print(f"Camera 1: {data1}")
    except zmq.Again:
        pass
        
    try:
        data2 = json.loads(socket2.recv_string(zmq.NOBLOCK))
        print(f"Camera 2: {data2}")
    except zmq.Again:
        pass
```

## Performance Considerations

- Each camera instance uses significant CPU/GPU resources
- Monitor system resources when running multiple instances
- The dual camera setup uses **less resources** than separate containers
- Consider reducing frame rates if needed:
  ```xml
  <param name="framerate" type="int" value="60"/>  <!-- Reduce from 120 -->
  ```

- Disable GUI for headless operation:
  ```xml
  <param name="use_gui" value="false" />
  ```

## Comparison: Separate vs Same Container

| Method | Resource Usage | Complexity | Use Case |
|--------|----------------|------------|----------|
| **Separate Containers** | Higher (2x Docker overhead) | Simple to manage | Independent camera control |
| **Same Container** | Lower (shared ROS master) | Single point of control | Synchronized dual camera setup |

**Recommendation:** Use the **dual camera setup** (`./dev-kinefly-dual.sh`) for most use cases as it's more efficient and easier to manage. 