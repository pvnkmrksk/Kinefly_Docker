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

echo -e "${YELLOW}🚀 Starting Kinefly Dual Camera Setup${NC}"
echo -e "${YELLOW}📹 Camera 1: /dev/video4 → Port ${CAM1_PORT}${NC}"
echo -e "${YELLOW}📹 Camera 2: /dev/video6 → Port ${CAM2_PORT}${NC}"

xhost +local:docker

# Cleanup any existing container
docker rm -f ${CONTAINER_NAME} > /dev/null 2>&1

# Start container and run dual camera setup, then drop to shell
docker run -it \
    --privileged \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --name ${CONTAINER_NAME} \
    --entrypoint /bin/bash \
    kinefly \
    -c "/opt/Kinefly_docker/start-kinefly-dual.sh ${CAM1_PORT} ${CAM2_PORT}; echo; echo 'Dual camera script ended. You are now in the container for debugging.'; echo 'Commands: kinefly-cam1 [PORT] | kinefly-cam2 [PORT] | kinefly-dual [CAM1_PORT] [CAM2_PORT]'; bash" 