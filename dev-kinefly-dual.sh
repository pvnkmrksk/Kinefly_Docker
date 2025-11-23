#!/bin/bash

# Kinefly Dual Camera Development Script
# One command to start both cameras in the same Docker container

CONTAINER_NAME="kinefly_dual"
CAM1_PORT=${1:-9871}
CAM2_PORT=${2:-9872}

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Function to sync configs on exit
cleanup_and_sync() {
    echo -e "\n${YELLOW}🔄 Syncing configurations from container to host (on exit)...${NC}"
    "$SCRIPT_DIR/sync-config-from-container.sh" "$CONTAINER_NAME"
    # Stop and remove container
    docker stop "$CONTAINER_NAME" > /dev/null 2>&1
    docker rm "$CONTAINER_NAME" > /dev/null 2>&1
    echo -e "${GREEN}✅ Cleanup complete${NC}"
}

# Set up trap to sync on exit
trap cleanup_and_sync EXIT INT TERM

echo -e "${YELLOW}🚀 Starting Kinefly Dual Camera Setup${NC}"
echo -e "${YELLOW}📹 Camera 1: /dev/video4 → Port ${CAM1_PORT}${NC}"
echo -e "${YELLOW}📹 Camera 2: /dev/video6 → Port ${CAM2_PORT}${NC}"

# Setup X11 for Linux
xhost +local:docker

# Cleanup any existing container
docker rm -f ${CONTAINER_NAME} > /dev/null 2>&1

# Start container in background first
echo -e "${YELLOW}🚀 Starting container...${NC}"
docker run -d \
    --privileged \
    --net=host \
    --env="DISPLAY=${DISPLAY:-:0}" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd)/start-kinefly-dual.sh:/opt/Kinefly_docker/start-kinefly-dual.sh:ro" \
    --name ${CONTAINER_NAME} \
    kinefly \
    sleep infinity > /dev/null 2>&1

# Wait for container to be ready
sleep 2

# Sync configurations FROM host TO container at startup
echo -e "${YELLOW}📋 Syncing configurations from host to container (at startup)...${NC}"
"$SCRIPT_DIR/sync-config-to-container.sh" "$CONTAINER_NAME"

# Now attach and run the startup script
echo -e "${GREEN}✅ Configurations synced. Starting dual camera setup...${NC}"
docker exec -it ${CONTAINER_NAME} /bin/bash -c "/opt/Kinefly_docker/start-kinefly-dual.sh ${CAM1_PORT} ${CAM2_PORT}; echo; echo 'Dual camera script ended. You are now in the container for debugging.'; echo 'Commands: kinefly-cam1 [PORT] | kinefly-cam2 [PORT] | kinefly-dual [CAM1_PORT] [CAM2_PORT]'; bash"

# Sync will happen automatically via trap on exit 