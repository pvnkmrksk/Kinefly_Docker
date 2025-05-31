#!/bin/bash

# Kinefly Demo Runner
# This script demonstrates the complete Kinefly -> ROS -> ZMQ pipeline

set -e

echo "🚀 Starting Kinefly Demo..."
echo "================================"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}=== $1 ===${NC}"
}

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    print_error "Docker is not running. Please start Docker first."
    exit 1
fi

# Build the Docker image
print_header "Building Kinefly Docker Image"
docker build -t kinefly .

# Stop and remove any existing containers
print_status "Cleaning up existing containers..."
docker stop kinefly_demo 2>/dev/null || true
docker rm kinefly_demo 2>/dev/null || true

# Start the container
print_header "Starting Kinefly Container"
print_status "Starting container with device access and host networking..."
docker run -d --name kinefly_demo --net=host -v /dev:/dev --privileged kinefly /bin/bash -c "
    source /opt/ros/kinetic/setup.bash && 
    source /root/catkin/devel/setup.bash && 
    export PYTHONPATH=/root/catkin/src/Kinefly/src:\$PYTHONPATH &&
    roscore &
    sleep 3 &&
    cd /opt/Kinefly_docker &&
    python test_flystate_publisher.py &
    sleep 2 &&
    python ros_zmq_bridge.py &
    wait
"

print_status "Container started. Waiting for services to initialize..."
sleep 5

# Check if services are running
print_header "Checking Services"
if docker exec kinefly_demo /bin/bash -c "source /opt/ros/kinetic/setup.bash && rostopic list | grep -q flystate"; then
    print_status "✅ ROS flystate topic is active"
else
    print_warning "⚠️  ROS flystate topic not found"
fi

# Test ZMQ connection
print_header "Testing ZMQ Bridge"
print_status "Testing ZMQ connection on localhost:9871..."

if timeout 5 python test_zmq_client.py --host localhost --port 9871 --max-messages 3 > /dev/null 2>&1; then
    print_status "✅ ZMQ bridge is working!"
else
    print_warning "⚠️  ZMQ bridge test failed"
fi

print_header "Demo Ready!"
echo ""
echo "🎯 The Kinefly demo is now running with:"
echo "   📡 ROS Core"
echo "   🦋 Test Flystate Publisher (simulated wing data)"
echo "   🌉 ZMQ Bridge (ROS → ZMQ)"
echo ""
echo "📊 To view live data:"
echo "   python test_zmq_client.py --host localhost --port 9871"
echo ""
echo "🔧 To access the container:"
echo "   docker exec -it kinefly_demo /bin/bash"
echo ""
echo "🛑 To stop the demo:"
echo "   docker stop kinefly_demo"
echo ""
echo "📋 Container logs:"
echo "   docker logs kinefly_demo"
echo ""

# Optionally start the client
read -p "🤔 Would you like to view live data now? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    print_status "Starting ZMQ client..."
    python test_zmq_client.py --host localhost --port 9871
fi 