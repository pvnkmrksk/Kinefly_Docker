#!/bin/bash

# Unified Kinefly VR Development Script
# One command to start VR(s) with configurable selection
# Usage: ./dev-kinefly-vr.sh [VR_NUMBER] [PORT]
#   - No arguments: starts all VRs (VR1-VR4)
#   - VR_NUMBER (1-4): starts specific VR
#   - PORT: optional custom ZMQ port

CONTAINER_NAME="kinefly_vr"
VR_NUMBER=$1
CUSTOM_PORT=$2

# Constants
MAX_VR=4

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Validate VR number if provided
if [ ! -z "$VR_NUMBER" ]; then
    if ! [[ "$VR_NUMBER" =~ ^[0-9]+$ ]] || [ "$VR_NUMBER" -lt 1 ] || [ "$VR_NUMBER" -gt $MAX_VR ]; then
        echo -e "${RED}❌ Invalid VR number: $VR_NUMBER${NC}"
        echo "Usage: $0 [VR_NUMBER] [PORT]"
        echo "VR_NUMBER must be between 1 and $MAX_VR"
        echo "If VR_NUMBER is not specified, all VRs (1-$MAX_VR) will be started"
        exit 1
    fi
    echo -e "${YELLOW}🚀 Starting Kinefly VR${VR_NUMBER}${NC}"
else
    echo -e "${YELLOW}🚀 Starting Kinefly All VRs (VR1-VR${MAX_VR})${NC}"
fi

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
    --volume="$(pwd)/start-kinefly-vr.sh:/opt/Kinefly_docker/start-kinefly-vr.sh:ro" \
    --name ${CONTAINER_NAME} \
    kinefly \
    sleep infinity > /dev/null 2>&1

# Wait for container to be ready
sleep 2

# Sync configurations FROM host TO container at startup
echo -e "${YELLOW}📋 Syncing configurations from host to container (at startup)...${NC}"
"$SCRIPT_DIR/sync-config-to-container.sh" "$CONTAINER_NAME"

# Now attach and run the startup script
echo -e "${GREEN}✅ Configurations synced. Starting Kinefly VR...${NC}"
if [ -z "$VR_NUMBER" ]; then
    docker exec -it ${CONTAINER_NAME} /bin/bash -c "/opt/Kinefly_docker/start-kinefly-vr.sh; echo; echo 'Script ended. You are now in the container for debugging.'; bash"
else
    docker exec -it ${CONTAINER_NAME} /bin/bash -c "/opt/Kinefly_docker/start-kinefly-vr.sh ${VR_NUMBER} ${CUSTOM_PORT}; echo; echo 'Script ended. You are now in the container for debugging.'; bash"
fi

# Sync will happen automatically via trap on exit

