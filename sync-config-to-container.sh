#!/bin/bash

# Sync Configuration to Container Script
# Copies launch configs and config files from host to container at startup

CONTAINER_NAME=$1

if [ -z "$CONTAINER_NAME" ]; then
    echo "Usage: $0 <container_name>"
    exit 1
fi

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Copy launch files
if [ -d "$SCRIPT_DIR/launch" ]; then
    docker cp "$SCRIPT_DIR/launch/" "$CONTAINER_NAME:/root/catkin/src/Kinefly/" > /dev/null 2>&1
    docker exec $CONTAINER_NAME chmod -R 755 /root/catkin/src/Kinefly/launch/ 2>/dev/null || true
fi

# Copy config files
if [ -d "$SCRIPT_DIR/config" ]; then
    if [ -f "$SCRIPT_DIR/config/kinefly.yaml" ]; then
        docker cp "$SCRIPT_DIR/config/kinefly.yaml" "$CONTAINER_NAME:/root/" > /dev/null 2>&1
        docker cp "$SCRIPT_DIR/config/kinefly.yaml" "$CONTAINER_NAME:/root/kinefly_cam1/kinefly_cam1.yaml" 2>/dev/null || true
        docker cp "$SCRIPT_DIR/config/kinefly.yaml" "$CONTAINER_NAME:/root/kinefly_cam2/kinefly_cam2.yaml" 2>/dev/null || true
    fi
fi

# Copy ros_zmq_bridge.py if it exists
if [ -f "$SCRIPT_DIR/ros_zmq_bridge.py" ]; then
    docker cp "$SCRIPT_DIR/ros_zmq_bridge.py" "$CONTAINER_NAME:/root/catkin/src/Kinefly/launch/" > /dev/null 2>&1
fi


