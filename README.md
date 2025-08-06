# Kinefly Docker Setup

A complete Docker-based setup for running Kinefly (fly wing tracking) with ROS-to-ZMQ bridge functionality.

## 🚀 Quick Start

### Option 1: Run Complete Demo (Recommended)
```bash
# Run the complete demo with simulated data
./run_kinefly_demo.sh
```

This will:
- Build the Docker image
- Start ROS core
- Launch a test flystate publisher (simulated wing data)
- Start the ZMQ bridge
- Test the connection
- Optionally show live data

### Option 2: Manual Setup

```bash
# Build the Docker image
docker build -t kinefly .

# Run with camera access and host networking
docker run -it --name kinefly_container --net=host -v /dev:/dev --privileged kinefly

# Inside the container, start the bridge
cd /opt/Kinefly_docker
python ros_zmq_bridge.py
```

## 📋 What's Included

### Core Components
- **Kinefly**: Fly wing tracking software
- **ROS Kinetic**: Robot Operating System
- **ZMQ Bridge**: Converts ROS messages to ZMQ for external applications
- **Test Publisher**: Simulates realistic wing tracking data

### Bridge Scripts
- `ros_zmq_bridge.py` - Main ROS-to-ZMQ bridge
- `socket_zmq_republisher.py` - Alternative socket-based republisher
- `test_flystate_publisher.py` - Generates test data for development
- `test_zmq_client.py` - Client to view live data

### Helper Scripts
- `run_kinefly_demo.sh` - Complete demo runner
- `setup.sh` - Automated dependency installation
- `start_bridge.sh` - Bridge startup script
- `test_camera.sh` - Camera testing utilities

## 🔧 Configuration

### ZMQ Bridge Settings
- **Default Port**: 9871
- **Protocol**: TCP
- **Host**: localhost (when using `--net=host`)
- **Message Format**: JSON

### ROS Topics
- **Input**: `/kinefly/flystate` (Kinefly wing tracking data)
- **Output**: ZMQ socket on `tcp://*:9871`

## 📊 Data Format

The bridge outputs structured JSON messages:

```json
{
  "message_info": {
    "source": "kinefly",
    "message_type": "wing_tracking",
    "frame_number": 123,
    "timestamp_ros": 1234567890.123,
    "frame_id": "camera"
  },
  "wing_tracking": {
    "left_wing": {
      "angle_radians": 0.5,
      "angle_degrees": 28.65,
      "beat_frequency_hz": 200.0,
      "tracking_confidence": 0.8,
      "gradient": 0.1
    },
    "right_wing": {
      "angle_radians": -0.3,
      "angle_degrees": -17.19,
      "beat_frequency_hz": 205.0,
      "tracking_confidence": 0.9,
      "gradient": -0.05
    }
  },
  "body_parts": {
    "head": {
      "angle_radians": 0.0,
      "tracking_confidence": 0.7,
      "radius": 5.0
    },
    "abdomen": {
      "angle_radians": 0.1,
      "tracking_confidence": 0.6,
      "radius": 3.0
    }
  },
  "legacy_unity_format": {
    "x": 0.5,
    "y": -0.3,
    "z": 0.0,
    "yaw": 0.0,
    "pitch": 0.0,
    "roll": 0.0
  }
}
```

## 🧪 Testing

### View Live Data
```bash
# Connect to the ZMQ stream and view formatted output
python test_zmq_client.py --host localhost --port 9871

# Limit to specific number of messages
python test_zmq_client.py --host localhost --port 9871 --max-messages 10
```

### Test with Simulated Data
```bash
# In the container, start the test publisher
docker exec -it kinefly_container /bin/bash -c "
  source /opt/ros/kinetic/setup.bash && 
  source /root/catkin/devel/setup.bash && 
  cd /opt/Kinefly_docker && 
  python test_flystate_publisher.py
"
```

### Check ROS Topics
```bash
# List available topics
docker exec -it kinefly_container /bin/bash -c "
  source /opt/ros/kinetic/setup.bash && 
  rostopic list
"

# Monitor flystate messages
docker exec -it kinefly_container /bin/bash -c "
  source /opt/ros/kinetic/setup.bash && 
  rostopic echo /kinefly/flystate
"
```

## 🛠️ Development

### Container Access
```bash
# Access running container
docker exec -it kinefly_container /bin/bash

# Start new interactive container
docker run -it --name kinefly_dev --net=host -v /dev:/dev --privileged kinefly /bin/bash
```

### Bridge Development
```bash
# Test bridge with custom settings
python ros_zmq_bridge.py --zmq-url "tcp://*:9872" --topic "/custom/flystate"

# Debug mode with verbose output
python ros_zmq_bridge.py --help
```

### Adding Custom Processing
The bridge script can be modified to add custom data processing:

```python
def process_ros_message(msg, socket_zmq):
    # Add your custom processing here
    # msg contains the Kinefly flystate data
    # socket_zmq is the ZMQ publisher socket
    pass
```

## 🔍 Troubleshooting

### Common Issues

#### 1. "Cannot open display" Error
**Problem**: Kinefly tries to open GUI even with `use_gui:=false`
**Solution**: Use the test publisher instead of real Kinefly for headless operation

#### 2. "No module named Kinefly.msg"
**Problem**: Python can't find Kinefly message definitions
**Solution**: Ensure `PYTHONPATH` includes `/root/catkin/src/Kinefly/src`

#### 3. ZMQ Connection Failed
**Problem**: Can't connect to ZMQ socket
**Solutions**:
- Check if bridge is running: `docker logs kinefly_container`
- Verify port 9871 is not blocked
- Ensure `--net=host` is used when running container

#### 4. No ROS Topics
**Problem**: `rostopic list` shows only `/rosout`
**Solutions**:
- Check if ROS core is running: `ps aux | grep roscore`
- Verify ROS environment: `echo $ROS_MASTER_URI`
- Restart the container

#### 5. Permission Denied
**Problem**: Can't access camera or devices
**Solution**: Run container with `--privileged` and `-v /dev:/dev`

### Debug Commands

```bash
# Check container status
docker ps -a

# View container logs
docker logs kinefly_container

# Check ROS environment
docker exec -it kinefly_container /bin/bash -c "env | grep ROS"

# Test ZMQ connectivity
timeout 5 python test_zmq_client.py --host localhost --port 9871 --max-messages 1

# Check Python path
docker exec -it kinefly_container /bin/bash -c "python -c 'import sys; print(sys.path)'"
```

### Performance Tuning

#### ZMQ Buffer Settings
```python
# In ros_zmq_bridge.py, add:
socket_zmq.setsockopt(zmq.SNDHWM, 1000)  # Send buffer
socket_zmq.setsockopt(zmq.LINGER, 0)     # No linger on close
```

#### ROS Message Queue
```python
# Adjust queue size in subscriber
sub = rospy.Subscriber(topic, MsgFlystate, callback, queue_size=1)
```

## 📚 Dependencies

### System Requirements
- Docker
- Linux (tested on Ubuntu)
- Camera device (for real Kinefly operation)

### Python Packages (installed in container)
- `click==6.7` - Command line interface
- `pyzmq==17.1.2` - ZeroMQ Python bindings
- `rospkg==1.1.10` - ROS package utilities

### ROS Packages
- `ros-kinetic-desktop-full` - Complete ROS installation
- `ros-kinetic-uvc-camera` - USB camera support
- `ros-kinetic-usb-cam` - USB camera driver

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Test your changes with the demo script
4. Submit a pull request

## 📄 License

This project builds upon Kinefly and ROS, which have their own licenses. Please refer to their respective documentation for licensing information.

## 🆘 Support

If you encounter issues:

1. Run the demo script: `./run_kinefly_demo.sh`
2. Check the troubleshooting section above
3. View container logs: `docker logs kinefly_container`
4. Create an issue with:
   - Your system information
   - Complete error messages
   - Steps to reproduce

---

**Note**: This setup is designed for development and testing. For production use, consider additional security measures and performance optimizations. 