#!/bin/bash

# Test Camera Configuration Script for Kinefly
# This script helps verify your camera is working correctly

echo "🎥 Testing Kinefly Camera Configuration..."
echo "==========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if running in Docker
if [ -f /.dockerenv ]; then
    echo -e "${BLUE}📦 Running inside Docker container${NC}"
    IN_DOCKER=true
else
    echo -e "${YELLOW}🖥️  Running on host system${NC}"
    IN_DOCKER=false
fi

echo ""
echo "🔍 Step 1: Checking video devices..."
echo "Available video devices:"
ls -la /dev/video* 2>/dev/null || echo -e "${RED}❌ No video devices found${NC}"

echo ""
echo "🔍 Step 2: Checking camera permissions..."
if [ -c "/dev/video4" ]; then
    echo -e "${GREEN}✅ /dev/video4 exists${NC}"
    ls -la /dev/video4
else
    echo -e "${YELLOW}⚠️  /dev/video4 not found, checking other devices...${NC}"
    ls -la /dev/video* 2>/dev/null
fi

echo ""
echo "🔍 Step 3: Testing camera with v4l2..."
if command -v v4l2-ctl &> /dev/null; then
    if [ -c "/dev/video4" ]; then
        echo "Camera capabilities for /dev/video4:"
        v4l2-ctl --device=/dev/video4 --all | head -20
        echo ""
        echo "Supported formats:"
        v4l2-ctl --device=/dev/video4 --list-formats-ext | grep -E "(Format|Size|Interval)"
    else
        echo -e "${YELLOW}⚠️  /dev/video4 not available for testing${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  v4l2-ctl not available (install v4l-utils)${NC}"
fi

echo ""
echo "🔍 Step 4: Testing ROS camera launch..."
if $IN_DOCKER; then
    echo "Setting up ROS environment..."
    source /opt/ros/kinetic/setup.bash
    source /root/catkin/devel/setup.bash
    
    echo "Launch file location:"
    find /root -name "camera_1394.launch" -type f 2>/dev/null
    
    echo ""
    echo "Launch file content:"
    cat /root/catkin/src/Kinefly/launch/rhag/camera_1394.launch 2>/dev/null || echo -e "${RED}❌ Launch file not found${NC}"
    
    echo ""
    echo "🚀 Testing camera launch (5 second test)..."
    echo "Press Ctrl+C if camera starts successfully"
    timeout 10s roslaunch Kinefly camera_1394.launch rig:=rhag 2>/dev/null || echo -e "${BLUE}ℹ️  Camera test completed${NC}"
fi

echo ""
echo "📋 Camera Configuration Summary:"
echo "================================"
echo "Video Device: /dev/video4"
echo "Resolution: 800x600"
echo "Format: mjpeg"
echo "Framerate: 30 fps"
echo "Namespace: rhag_camera"
echo ""
echo "🔧 To use this camera with Kinefly:"
echo "   1. Start container with: docker run -it --privileged --net=host [...]"
echo "   2. Launch camera: roslaunch Kinefly camera_1394.launch rig:=rhag"
echo "   3. Start tracking: roslaunch Kinefly main_1394.launch rig:=rhag"
echo ""
echo "🎯 Camera testing complete!" 