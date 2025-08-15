#!/bin/bash

# Kinefly Camera 2 Startup Script
# Starts Kinefly + ZMQ Bridge for Camera 2 with configurable port

# Source ROS environment
source /opt/ros/kinetic/setup.bash
source /root/catkin/devel/setup.bash
export RIG=rhag_cam2
export PYTHONPATH=/root/catkin/src/Kinefly/src:$PYTHONPATH

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default port
DEFAULT_PORT="9872"
PORT=${1:-$DEFAULT_PORT}

# Validate port number
if ! [[ "$PORT" =~ ^[0-9]+$ ]] || [ "$PORT" -lt 1024 ] || [ "$PORT" -gt 65535 ]; then
    echo -e "${RED}❌ Invalid port: $PORT${NC}"
    echo "Usage: $0 [PORT]"
    echo "Port must be between 1024-65535. Default: $DEFAULT_PORT"
    exit 1
fi

echo -e "${BLUE}🚀 Kinefly Camera 2 Startup${NC}"
echo -e "${YELLOW}📡 ZMQ Port: ${PORT}${NC}"
echo -e "${YELLOW}📹 Video Device: /dev/video6${NC}"
echo -e "${YELLOW}⏹️  Press Ctrl+C to stop everything${NC}"
echo

# Function to cleanup processes
cleanup() {
    echo -e "\n${YELLOW}🛑 Shutting down Camera 2...${NC}"
    
    # Kill background processes
    if [ ! -z "$KINEFLY_PID" ]; then
        echo -e "${YELLOW}📦 Stopping Kinefly Cam2 (PID: $KINEFLY_PID)${NC}"
        kill $KINEFLY_PID 2>/dev/null
        wait $KINEFLY_PID 2>/dev/null
    fi
    
    if [ ! -z "$BRIDGE_PID" ]; then
        echo -e "${YELLOW}🌉 Stopping ZMQ Bridge Cam2 (PID: $BRIDGE_PID)${NC}"
        kill $BRIDGE_PID 2>/dev/null
        wait $BRIDGE_PID 2>/dev/null
    fi
    
    # Clean up any remaining ROS processes for cam2
    pkill -f "roslaunch.*rhag_cam2" 2>/dev/null
    pkill -f "ros_zmq_bridge.*cam2" 2>/dev/null
    
    echo -e "${GREEN}✅ Camera 2 cleanup complete${NC}"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Check if port is already in use
if netstat -an 2>/dev/null | grep -q ":${PORT} "; then
    echo -e "${RED}❌ Port ${PORT} is already in use${NC}"
    echo "Try a different port or kill the process using the port:"
    echo "  netstat -tlnp | grep :${PORT}"
    exit 1
fi

echo -e "${GREEN}🎬 Starting Kinefly Camera 2...${NC}"

# Ensure ROS master is available (start if needed)
if ! pgrep -f "rosmaster" > /dev/null; then
    echo -e "${YELLOW}⚡ Starting ROS master...${NC}"
    roscore > /tmp/roscore_cam2.log 2>&1 &
    sleep 3
fi

# Start Kinefly in background
roslaunch Kinefly main.launch > /tmp/kinefly_cam2.log 2>&1 &
KINEFLY_PID=$!

# Wait for ROS to initialize
echo -e "${YELLOW}⏳ Waiting for ROS to initialize...${NC}"
sleep 5

# Check if Kinefly started successfully
if ! kill -0 $KINEFLY_PID 2>/dev/null; then
    echo -e "${RED}❌ Kinefly Camera 2 failed to start${NC}"
    echo "Check log: tail /tmp/kinefly_cam2.log"
    exit 1
fi

# Wait for Kinefly topic to be available
echo -e "${YELLOW}⏳ Waiting for Kinefly Camera 2 topic...${NC}"
timeout=30
counter=0
while [ $counter -lt $timeout ]; do
    if rostopic list 2>/dev/null | grep -q "/kinefly_cam2/kinefly_cam2/flystate"; then
        break
    fi
    sleep 1
    counter=$((counter + 1))
done

if [ $counter -eq $timeout ]; then
    echo -e "${RED}❌ Kinefly Camera 2 topic not found after ${timeout}s${NC}"
    cleanup
    exit 1
fi

echo -e "${GREEN}🌉 Starting ZMQ Bridge Camera 2 on port ${PORT}...${NC}"

# Start ZMQ bridge in background
cd /root/catkin/src/Kinefly/launch/
python2 ros_zmq_bridge.py --zmq-url "tcp://*:${PORT}" --topic "/kinefly_cam2/kinefly_cam2/flystate" > /tmp/zmq_bridge_cam2.log 2>&1 &
BRIDGE_PID=$!

# Wait for bridge to start
sleep 2

# Check if bridge started successfully
if ! kill -0 $BRIDGE_PID 2>/dev/null; then
    echo -e "${RED}❌ ZMQ Bridge Camera 2 failed to start${NC}"
    echo -e "${YELLOW}📋 Check log: tail /tmp/zmq_bridge_cam2.log${NC}"
    cleanup
    exit 1
fi

echo -e "${GREEN}✅ Camera 2 services running!${NC}"
echo -e "${BLUE}📊 Status:${NC}"
echo -e "  Kinefly PID: ${KINEFLY_PID}"
echo -e "  Bridge PID:  ${BRIDGE_PID}"
echo -e "  ZMQ Port:    ${PORT}"
echo -e "  Topic:       /kinefly_cam2/kinefly_cam2/flystate"
echo -e "  Video:       /dev/video6"
echo

# Monitor processes
while true; do
    # Check if Kinefly is still running
    if ! kill -0 $KINEFLY_PID 2>/dev/null; then
        echo -e "${RED}❌ Kinefly Camera 2 stopped unexpectedly${NC}"
        cleanup
        exit 1
    fi
    
    # Check if bridge is still running
    if ! kill -0 $BRIDGE_PID 2>/dev/null; then
        echo -e "${RED}❌ ZMQ Bridge Camera 2 stopped unexpectedly${NC}"
        cleanup
        exit 1
    fi
    
    sleep 5
done 