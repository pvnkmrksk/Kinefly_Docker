# Manual Kinefly Setup Guide

This guide shows how to manually start Kinefly and ZMQ bridge in separate terminals for debugging or manual control.

## Quick Start (Automated)
```bash
./dev-kinefly.sh [PORT]    # One command to start everything
```

## Manual Setup (Step-by-Step)

### Terminal 1: Start Docker Container
```bash
# Start interactive Docker container
docker run -it \
    --privileged \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --name kinefly_manual \
    kinefly \
    /bin/bash

# Or use existing container
docker exec -it kinefly_manual /bin/bash
```

### Inside Docker Container:

#### Setup ROS Environment (in container)
```bash
# Source ROS environment
source /opt/ros/kinetic/setup.bash
source /root/catkin/devel/setup.bash
export RIG=rhag
export PYTHONPATH=/root/catkin/src/Kinefly/src:$PYTHONPATH

# Start ROS master (if not running)
roscore &
```

### Terminal 2: Launch Kinefly (new container terminal)
```bash
# Open new terminal in same container
docker exec -it kinefly_manual /bin/bash

# Source environment
source /opt/ros/kinetic/setup.bash
source /root/catkin/devel/setup.bash
export RIG=rhag

# Launch Kinefly
cd /root/catkin/src/Kinefly/launch
roslaunch Kinefly main.launch
```

### Terminal 3: Run ZMQ Bridge (new container terminal)
```bash
# Open new terminal in same container  
docker exec -it kinefly_manual /bin/bash

# Source environment
source /opt/ros/kinetic/setup.bash
source /root/catkin/devel/setup.bash
export PYTHONPATH=/root/catkin/src/Kinefly/src:$PYTHONPATH

# Wait for Kinefly topic to be available
rostopic list | grep kinefly

# Start ZMQ bridge on desired port (e.g., 9871)
cd /root/catkin/src/Kinefly/launch
python2 ros_zmq_bridge.py
```

### Terminal 4: Test ZMQ Connection (host)
```bash
# Test from host machine
python3 test_zmq_client.py --zmq-url tcp://localhost:9871
```

## Manual Commands Reference

### ROS Commands (inside container)
```bash
# Check ROS topics
rostopic list

# Monitor Kinefly data
rostopic echo /kinefly/flystate

# Check ROS nodes
rosnode list

# Kill all ROS processes
pkill -f ros
```

### Docker Commands (host)
```bash
# List running containers
docker ps

# Stop container
docker stop kinefly_manual

# Remove container
docker rm kinefly_manual

# View container logs
docker logs kinefly_manual
```

### ZMQ Bridge Commands (inside container)
```bash
# Start bridge with custom port
python2 -c "
import sys
sys.path.append('/root/catkin/src/Kinefly/launch')
from ros_zmq_bridge import main
main(zmq_url='tcp://*:9872', topic='/kinefly/flystate')
"

# Check if port is in use
netstat -tlnp | grep :9871
```

### Debugging Commands
```bash
# Check Kinefly logs
tail -f /tmp/kinefly.log

# Check ZMQ bridge logs  
tail -f /tmp/zmq_bridge.log

# Check ROS master logs
tail -f /tmp/roscore.log

# Monitor system processes
ps aux | grep ros
ps aux | grep python
```

## Troubleshooting

### Kinefly Won't Start
```bash
# Check camera permissions
ls -la /dev/video*

# Check ROS environment
echo $ROS_ROOT
echo $ROS_PACKAGE_PATH

# Verify Kinefly package
rospack find Kinefly
```

### ZMQ Bridge Issues
```bash
# Check Python path
python2 -c "import sys; print('\n'.join(sys.path))"

# Test ROS message import
python2 -c "from Kinefly.msg import MsgFlystate; print('Success')"

# Check topic data
rostopic hz /kinefly/flystate
```

### Port Conflicts
```bash
# Find process using port
netstat -tlnp | grep :9871
lsof -i :9871

# Kill process using port
kill -9 $(lsof -t -i:9871)
```

## Container Aliases (available inside container)
```bash
kinefly [PORT]     # Start everything with optional port
status             # Show running processes  
test-data          # Test ZMQ connection
zmq-bridge         # Start just the ZMQ bridge
```

## File Locations
- **Kinefly source**: `/root/catkin/src/Kinefly/`
- **Launch files**: `/root/catkin/src/Kinefly/launch/`
- **ZMQ bridge**: `/root/catkin/src/Kinefly/launch/ros_zmq_bridge.py`
- **Test client**: `/opt/Kinefly_docker/test_zmq_client.py`
- **Logs**: `/tmp/*.log` 