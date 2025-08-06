#!/bin/bash

# Kinefly Dual Camera Startup Script
# Starts both Camera 1 and Camera 2 in the same container with proper sequencing

# Source ROS environment
source /opt/ros/kinetic/setup.bash
source /root/catkin/devel/setup.bash
export PYTHONPATH=/root/catkin/src/Kinefly/src:$PYTHONPATH

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default ports
CAM1_PORT=${1:-9871}
CAM2_PORT=${2:-9872}

echo -e "${BLUE}🚀 Kinefly Dual Camera Startup${NC}"
echo -e "${YELLOW}📹 Camera 1: /dev/video4 → Port ${CAM1_PORT}${NC}"
echo -e "${YELLOW}📹 Camera 2: /dev/video6 → Port ${CAM2_PORT}${NC}"
echo -e "${YELLOW}⏹️  Press Ctrl+C to stop everything${NC}"
echo

# Arrays to track PIDs
CAM1_PIDS=()
CAM2_PIDS=()

# Function to cleanup all processes
cleanup() {
    echo -e "\n${YELLOW}🛑 Shutting down both cameras...${NC}"
    
    # Kill Camera 1 processes
    for pid in "${CAM1_PIDS[@]}"; do
        if [ ! -z "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo -e "${YELLOW}📦 Stopping Camera 1 process (PID: $pid)${NC}"
            kill "$pid" 2>/dev/null
        fi
    done
    
    # Kill Camera 2 processes
    for pid in "${CAM2_PIDS[@]}"; do
        if [ ! -z "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo -e "${YELLOW}📦 Stopping Camera 2 process (PID: $pid)${NC}"
            kill "$pid" 2>/dev/null
        fi
    done
    
    # Clean up any remaining ROS processes
    pkill -f "roslaunch.*rhag_cam1" 2>/dev/null
    pkill -f "roslaunch.*rhag_cam2" 2>/dev/null
    pkill -f "ros_zmq_bridge" 2>/dev/null
    pkill -f "rosmaster" 2>/dev/null
    pkill -f "roscore" 2>/dev/null
    
    echo -e "${GREEN}✅ Dual camera cleanup complete${NC}"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Check if ports are available
if netstat -an 2>/dev/null | grep -q ":${CAM1_PORT} "; then
    echo -e "${RED}❌ Port ${CAM1_PORT} is already in use${NC}"
    exit 1
fi

if netstat -an 2>/dev/null | grep -q ":${CAM2_PORT} "; then
    echo -e "${RED}❌ Port ${CAM2_PORT} is already in use${NC}"
    exit 1
fi

echo -e "${GREEN}🎬 Starting ROS master...${NC}"
# Ensure ROS master is available
if ! pgrep -f "rosmaster" > /dev/null; then
    roscore > /tmp/roscore.log 2>&1 &
    ROSCORE_PID=$!
    sleep 3
fi

echo -e "${GREEN}🎬 Starting Camera 1 (/dev/video4)...${NC}"
# Start Camera 1 Kinefly
export RIG=rhag_cam1
roslaunch Kinefly main.launch > /tmp/kinefly_cam1.log 2>&1 &
CAM1_KINEFLY_PID=$!
CAM1_PIDS+=($CAM1_KINEFLY_PID)

# Wait for Camera 1 to initialize
echo -e "${YELLOW}⏳ Waiting for Camera 1 to initialize...${NC}"
sleep 8

# Check if Camera 1 started successfully
if ! kill -0 $CAM1_KINEFLY_PID 2>/dev/null; then
    echo -e "${RED}❌ Camera 1 failed to start${NC}"
    echo "Check log: tail /tmp/kinefly_cam1.log"
    cleanup
    exit 1
fi

# Wait for Camera 1 topic
timeout=30
counter=0
while [ $counter -lt $timeout ]; do
    if rostopic list 2>/dev/null | grep -q "/kinefly_cam1/flystate"; then
        break
    fi
    sleep 1
    counter=$((counter + 1))
done

if [ $counter -eq $timeout ]; then
    echo -e "${RED}❌ Camera 1 topic not found${NC}"
    cleanup
    exit 1
fi

echo -e "${GREEN}✅ Camera 1 ready! Starting ZMQ Bridge...${NC}"
# Start Camera 1 ZMQ bridge
cd /root/catkin/src/Kinefly/launch/
python2 -c "
import sys
sys.path.append('/root/catkin/src/Kinefly/launch')
from ros_zmq_bridge import main
main(zmq_url='tcp://*:${CAM1_PORT}', topic='/kinefly_cam1/flystate')
" > /tmp/zmq_bridge_cam1.log 2>&1 &
CAM1_BRIDGE_PID=$!
CAM1_PIDS+=($CAM1_BRIDGE_PID)

sleep 2

echo -e "${GREEN}🎬 Starting Camera 2 (/dev/video6)...${NC}"
# Start Camera 2 Kinefly
export RIG=rhag_cam2
roslaunch Kinefly main.launch > /tmp/kinefly_cam2.log 2>&1 &
CAM2_KINEFLY_PID=$!
CAM2_PIDS+=($CAM2_KINEFLY_PID)

# Wait for Camera 2 to initialize
echo -e "${YELLOW}⏳ Waiting for Camera 2 to initialize...${NC}"
sleep 8

# Check if Camera 2 started successfully
if ! kill -0 $CAM2_KINEFLY_PID 2>/dev/null; then
    echo -e "${RED}❌ Camera 2 failed to start${NC}"
    echo "Check log: tail /tmp/kinefly_cam2.log"
    cleanup
    exit 1
fi

# Wait for Camera 2 topic
counter=0
while [ $counter -lt $timeout ]; do
    if rostopic list 2>/dev/null | grep -q "/kinefly_cam2/flystate"; then
        break
    fi
    sleep 1
    counter=$((counter + 1))
done

if [ $counter -eq $timeout ]; then
    echo -e "${RED}❌ Camera 2 topic not found${NC}"
    cleanup
    exit 1
fi

echo -e "${GREEN}✅ Camera 2 ready! Starting ZMQ Bridge...${NC}"
# Start Camera 2 ZMQ bridge
python2 -c "
import sys
sys.path.append('/root/catkin/src/Kinefly/launch')
from ros_zmq_bridge import main
main(zmq_url='tcp://*:${CAM2_PORT}', topic='/kinefly_cam2/flystate')
" > /tmp/zmq_bridge_cam2.log 2>&1 &
CAM2_BRIDGE_PID=$!
CAM2_PIDS+=($CAM2_BRIDGE_PID)

sleep 2

echo -e "${GREEN}🎉 Both cameras are running!${NC}"
echo -e "${BLUE}📊 Status:${NC}"
echo -e "  Camera 1: /dev/video4 → tcp://*:${CAM1_PORT} → /kinefly_cam1/flystate"
echo -e "  Camera 2: /dev/video6 → tcp://*:${CAM2_PORT} → /kinefly_cam2/flystate"
echo -e "  Kinefly PIDs: ${CAM1_KINEFLY_PID}, ${CAM2_KINEFLY_PID}"
echo -e "  Bridge PIDs:  ${CAM1_BRIDGE_PID}, ${CAM2_BRIDGE_PID}"
echo

echo -e "${YELLOW}💡 Test connections:${NC}"
echo -e "  python3 /opt/Kinefly_docker/test_zmq_client.py --zmq-url tcp://localhost:${CAM1_PORT}"
echo -e "  python3 /opt/Kinefly_docker/test_zmq_client.py --zmq-url tcp://localhost:${CAM2_PORT}"
echo

# Monitor all processes
while true; do
    # Check Camera 1 processes
    if ! kill -0 $CAM1_KINEFLY_PID 2>/dev/null; then
        echo -e "${RED}❌ Camera 1 Kinefly stopped unexpectedly${NC}"
        cleanup
        exit 1
    fi
    
    if ! kill -0 $CAM1_BRIDGE_PID 2>/dev/null; then
        echo -e "${RED}❌ Camera 1 ZMQ Bridge stopped unexpectedly${NC}"
        cleanup
        exit 1
    fi
    
    # Check Camera 2 processes
    if ! kill -0 $CAM2_KINEFLY_PID 2>/dev/null; then
        echo -e "${RED}❌ Camera 2 Kinefly stopped unexpectedly${NC}"
        cleanup
        exit 1
    fi
    
    if ! kill -0 $CAM2_BRIDGE_PID 2>/dev/null; then
        echo -e "${RED}❌ Camera 2 ZMQ Bridge stopped unexpectedly${NC}"
        cleanup
        exit 1
    fi
    
    sleep 5
done 