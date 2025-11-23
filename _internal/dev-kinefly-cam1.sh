#!/bin/bash

# Kinefly Camera 1 Development Script
# One command to start Camera 1 with configurable port

CONTAINER_NAME="kinefly_cam1"
PORT=${1:-9871}

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

echo -e "${YELLOW}🚀 Starting Kinefly Camera 1 (Port: ${PORT})${NC}"
echo -e "${YELLOW}📹 Video Device: /dev/video4${NC}"

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
    --volume="$(pwd)/start-kinefly-cam1.sh:/opt/Kinefly_docker/start-kinefly-cam1.sh:ro" \
    --name ${CONTAINER_NAME} \
    kinefly \
    sleep infinity > /dev/null 2>&1

# Wait for container to be ready
sleep 2

# Sync configurations FROM host TO container at startup
echo -e "${YELLOW}📋 Syncing configurations from host to container (at startup)...${NC}"
"$SCRIPT_DIR/sync-config-to-container.sh" "$CONTAINER_NAME"

# Now attach and run the startup script
echo -e "${GREEN}✅ Configurations synced. Starting Camera 1...${NC}"
docker exec -it ${CONTAINER_NAME} /bin/bash -c "/opt/Kinefly_docker/start-kinefly-cam1.sh ${PORT}; echo; echo 'Camera 1 script ended. You are now in the container for debugging.'; echo 'Commands: kinefly-cam1 [PORT] | kinefly-cam2 [PORT] | status | test-data'; bash"

# Sync will happen automatically via trap on exit 