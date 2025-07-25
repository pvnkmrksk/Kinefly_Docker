# Manual Kinefly Setup Guide

**Note: Use `./dev-kinefly.sh` for normal operation. This guide is for debugging only.**

## Quick Start (Recommended)
```bash
./dev-kinefly.sh [PORT]    # One command to start everything
```

## Manual Setup (For Debugging)

### Start Container Manually
```bash
docker run -it \
    --privileged \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --name kinefly_debug \
    kinefly \
    /bin/bash
```

### Inside Container - Manual Steps

#### 1. Start ROS Master
```bash
source /opt/ros/kinetic/setup.bash
source /root/catkin/devel/setup.bash
export RIG=rhag
roscore &
```

#### 2. Start Kinefly
```bash
# In new terminal
docker exec -it kinefly_debug /bin/bash
source /opt/ros/kinetic/setup.bash
source /root/catkin/devel/setup.bash
export RIG=rhag
roslaunch Kinefly main.launch
```

#### 3. Start ZMQ Bridge
```bash
# In new terminal
docker exec -it kinefly_debug /bin/bash
source /opt/ros/kinetic/setup.bash
source /root/catkin/devel/setup.bash
export PYTHONPATH=/root/catkin/src/Kinefly/src:$PYTHONPATH
cd /root/catkin/src/Kinefly/launch
python2 ros_zmq_bridge.py
```

## Debugging Commands

### Check ROS Status
```bash
rostopic list | grep kinefly
rostopic echo /kinefly/flystate --once
rosnode list
```

### Check ZMQ Connection
```bash
# From host
python3 test_zmq_client.py --zmq-url tcp://localhost:9871
```

### Kill All Processes
```bash
pkill -f ros
pkill -f ros_zmq_bridge
```

## Configuration Files Location
- Kinefly config: `/root/kinefly.yaml`
- Launch files: `/root/catkin/src/Kinefly/launch/`
- ZMQ bridge: `/root/catkin/src/Kinefly/launch/ros_zmq_bridge.py` 