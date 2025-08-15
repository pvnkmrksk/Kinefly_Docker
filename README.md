# Kinefly ZMQ Bridge

Simple bridge connecting Kinefly ROS topics to ZMQ for real-time wing tracking data.

## ⚡ One Command to Start Everything

```bash
./dev-kinefly.sh          # Start Kinefly + ZMQ bridge on port 9871
./dev-kinefly.sh 9872     # Custom port
```

That's it! Press Ctrl+C to stop everything gracefully.

## Container Commands

Once inside the container, you can also use:
```bash
kinefly           # Start everything on port 9871
kinefly 9872      # Start everything on custom port
status            # Check ROS topics and cameras  
test-data         # Test if Kinefly is publishing data
```

## Configuration Management

### Quick Configuration Changes
1. **Edit in container** (recommended for testing):
   ```bash
   ./dev-kinefly.sh
   # Once inside container, edit files:
   nano /root/kinefly.yaml                    # Kinefly config
   nano /root/catkin/src/Kinefly/launch/main.launch  # Launch config
   ```

2. **Copy changes back to host**:
   ```bash
   ./copy-config.sh  # Copies all configs from container
   ```

3. **Rebuild container** to apply changes:
   ```bash
   docker build -t kinefly .
   ```

### Configuration Files
- `config/kinefly.yaml` - Main Kinefly configuration
- `launch/` - ROS launch files
- `config/README.md` - Detailed configuration guide

## ZMQ Output

Simple JSON format:
```json
{
  "x": 0.123,     // left wing angle (radians)
  "y": -0.456,    // right wing angle (radians) 
  "z": 0.0,       // always 0
  "yaw": 0.579,   // difference (x - y)
  "pitch": 0.0,   // always 0
  "roll": 0.0     // always 0
}
```

## Features

✅ **One command startup** - Everything starts together  
✅ **Configurable port** - No more port conflicts  
✅ **Graceful shutdown** - Ctrl+C stops everything cleanly  
✅ **Automatic cleanup** - Handles process management  
✅ **Port validation** - Prevents conflicts before starting  
✅ **Error handling** - Clear error messages and recovery  
✅ **Easy configuration** - Edit in container, copy back to host

## Project Structure

```
Kinefly_docker/
├── dev-kinefly.sh           # Main startup script
├── start-kinefly-all.sh     # Container startup script
├── copy-config.sh           # Copy configs from container
├── config/                  # Configuration files
│   ├── kinefly.yaml
│   └── README.md
├── launch/                  # ROS launch files
├── tests/                   # Test files
│   ├── test_zmq_client.py
│   ├── test_camera.sh
│   ├── test_flystate_publisher.py
│   └── README.md
├── ros_zmq_bridge.py        # Core ZMQ bridge
└── Dockerfile               # Container definition
```

## Testing

Test ZMQ connection:
```bash
python3 tests/test_zmq_client.py --zmq-url tcp://localhost:9871
```

## Multi-Camera Support

The system now supports multiple cameras with correct topic subscriptions:

### Camera 1
```bash
./start-kinefly-cam1.sh [PORT]  # Uses topic: /kinefly_cam1/kinefly_cam1/flystate
```

### Camera 2  
```bash
./start-kinefly-cam2.sh [PORT]  # Uses topic: /kinefly_cam2/kinefly_cam2/flystate
```

### Both Cameras
```bash
./start-kinefly-dual.sh [CAM1_PORT] [CAM2_PORT]
```

### Topic Verification
```bash
./test_topics.sh              # Check available topics
./test_bridge.sh [PORT] [TOPIC]  # Test bridge with custom config
./verify_bridge.py --zmq-url tcp://localhost:PORT  # Verify ZMQ data
```

**Note**: Each camera uses a different ROS namespace, so the topics are:
- Vanilla: `/kinefly/flystate`
- Camera 1: `/kinefly_cam1/kinefly_cam1/flystate` 
- Camera 2: `/kinefly_cam2/kinefly_cam2/flystate`

### Topic Structure

The double namespace structure (e.g., `/kinefly_cam1/kinefly_cam1/flystate`) is **intentional and correct** for the multi-camera setup. This ensures each camera has a unique topic name.

**Verify the topics:**
```bash
./test_topics.sh  # Check that all expected topics exist
```