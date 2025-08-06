#!/bin/bash

# Kinefly Camera 2 Development Script
# One command to start Camera 2 with configurable port

CONTAINER_NAME="kinefly_cam2"
PORT=${1:-9872}

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}🚀 Starting Kinefly Camera 2 (Port: ${PORT})${NC}"
echo -e "${YELLOW}📹 Video Device: /dev/video6${NC}"

xhost +local:docker

# Cleanup any existing container
docker rm -f ${CONTAINER_NAME} > /dev/null 2>&1

# Start container and run camera 2, then drop to shell
docker run -it \
    --privileged \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --name ${CONTAINER_NAME} \
    --entrypoint /bin/bash \
    kinefly \
    -c "/opt/Kinefly_docker/start-kinefly-cam2.sh ${PORT}; echo; echo 'Camera 2 script ended. You are now in the container for debugging.'; echo 'Commands: kinefly-cam1 [PORT] | kinefly-cam2 [PORT] | status | test-data'; bash" 