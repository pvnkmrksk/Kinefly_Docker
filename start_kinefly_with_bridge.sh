#!/bin/bash

# Kinefly + ZMQ Bridge Startup Script
# This script starts both Kinefly and the ZMQ bridge in the same container

echo "🚀 Starting Kinefly with ZMQ Bridge..."

# Setup ROS environment
source /opt/ros/kinetic/setup.bash
source /root/catkin/devel/setup.bash
export RIG=rhag
export PYTHONPATH=/root/catkin/src/Kinefly/src:$PYTHONPATH

# Parse command line arguments
ZMQ_PORT="9871"
ZMQ_URL="tcp://*:$ZMQ_PORT"
KINEFLY_RIG="rhag"

while [[ $# -gt 0 ]]; do
    case $1 in
        --zmq-port)
            ZMQ_PORT="$2"
            ZMQ_URL="tcp://*:$ZMQ_PORT"
            shift 2
            ;;
        --zmq-url)
            ZMQ_URL="$2"
            shift 2
            ;;
        --rig)
            KINEFLY_RIG="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  --zmq-port PORT    Set ZMQ port (default: 9871)"
            echo "  --zmq-url URL      Set full ZMQ URL (default: tcp://*:9871)"
            echo "  --rig RIG_NAME     Set Kinefly rig name (default: rhag)"
            echo "  --help             Show this help"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

echo "📋 Configuration:"
echo "   🔧 Kinefly Rig: $KINEFLY_RIG"
echo "   🌐 ZMQ URL: $ZMQ_URL"
echo ""

# Function to cleanup on exit
cleanup() {
    echo "🛑 Shutting down processes..."
    kill $KINEFLY_PID 2>/dev/null
    kill $ZMQ_PID 2>/dev/null
    exit 0
}

# Trap SIGINT and SIGTERM
trap cleanup SIGINT SIGTERM

# Start Kinefly in background
echo "🎬 Starting Kinefly..."
roslaunch Kinefly main_1394.launch rig:=$KINEFLY_RIG > kinefly.log 2>&1 &
KINEFLY_PID=$!

# Wait for ROS to initialize
echo "⏳ Waiting for ROS to initialize..."
sleep 15

# Check if Kinefly is still running
if ! kill -0 $KINEFLY_PID 2>/dev/null; then
    echo "❌ Kinefly failed to start. Check kinefly.log for details."
    tail -20 kinefly.log
    exit 1
fi

echo "✅ Kinefly started successfully (PID: $KINEFLY_PID)"

# Start ZMQ bridge in background
echo "🌉 Starting ZMQ Bridge..."
cd /root/catkin/src/Kinefly/launch/
python2 ros_zmq_bridge.py --zmq-url "$ZMQ_URL" > zmq_bridge.log 2>&1 &
ZMQ_PID=$!

# Wait a moment for ZMQ bridge to initialize
sleep 3

# Check if ZMQ bridge is running
if ! kill -0 $ZMQ_PID 2>/dev/null; then
    echo "❌ ZMQ Bridge failed to start. Check zmq_bridge.log for details."
    tail -20 zmq_bridge.log
    cleanup
    exit 1
fi

echo "✅ ZMQ Bridge started successfully (PID: $ZMQ_PID)"
echo ""
echo "🎉 Both services are running!"
echo ""
echo "📊 Status:"
echo "   🎬 Kinefly:    PID $KINEFLY_PID (log: kinefly.log)"
echo "   🌉 ZMQ Bridge: PID $ZMQ_PID (log: zmq_bridge.log)"
echo ""
echo "📋 Useful commands:"
echo "   tail -f kinefly.log      # Monitor Kinefly"
echo "   tail -f zmq_bridge.log   # Monitor ZMQ Bridge"
echo "   ps aux | grep ros        # Check ROS processes"
echo "   netstat -an | grep $ZMQ_PORT  # Check ZMQ port"
echo ""
echo "🛑 Press Ctrl+C to stop both services"

# Wait for processes to finish
wait 