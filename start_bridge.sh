#!/bin/bash
# Kinefly ZMQ Bridge Startup Script
# This script sets up the proper environment and starts the bridge

set -e

echo "🚀 Starting Kinefly ZMQ Bridge..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_colored() {
    echo -e "${2}${1}${NC}"
}

# Check if we're inside the Docker container
if [ ! -f /.dockerenv ]; then
    print_colored "❌ This script should be run inside the Kinefly Docker container" $RED
    print_colored "💡 Start the container with:" $YELLOW
    print_colored "   docker run -it --privileged --net=host kinefly" $BLUE
    exit 1
fi

# Check if ROS is sourced
if [ -z "$ROS_ROOT" ]; then
    print_colored "🔧 Sourcing ROS environment..." $YELLOW
    source /opt/ros/kinetic/setup.bash
    source /root/catkin/devel/setup.bash
fi

# Set up Python path for Kinefly messages
export PYTHONPATH=/root/catkin/src/Kinefly/src:$PYTHONPATH

# Change to bridge directory
cd /opt/Kinefly_docker

print_colored "✅ Environment set up successfully" $GREEN
print_colored "📡 Starting ZMQ bridge on port 9871..." $GREEN

# Parse command line arguments
ZMQ_URL="tcp://*:9871"
TOPIC="/kinefly/flystate"

while [[ $# -gt 0 ]]; do
    case $1 in
        --port)
            ZMQ_URL="tcp://*:$2"
            shift 2
            ;;
        --topic)
            TOPIC="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [--port PORT] [--topic TOPIC]"
            echo "  --port PORT    ZMQ port to bind to (default: 9871)"
            echo "  --topic TOPIC  ROS topic to subscribe to (default: /kinefly/flystate)"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

print_colored "🎯 Binding to: $ZMQ_URL" $BLUE
print_colored "📺 Subscribing to: $TOPIC" $BLUE
print_colored "" $NC
print_colored "🔗 External applications can connect to: tcp://localhost:$(echo $ZMQ_URL | cut -d':' -f3)" $GREEN
print_colored "🛑 Press Ctrl+C to stop" $YELLOW
print_colored "" $NC

# Start the bridge
python2 ros_zmq_bridge.py --zmq-url "$ZMQ_URL" --topic "$TOPIC" 