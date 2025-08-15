#!/bin/bash
#!/bin/bash

# Simple Kinefly Development Script
# One command to start everything with configurable port

CONTAINER_NAME="kinefly_dev"
PORT=${1:-9871}

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}🚀 Starting Kinefly All-in-One (Port: ${PORT})${NC}"

xhost +local:docker

# Cleanup any existing container
docker rm -f ${CONTAINER_NAME} > /dev/null 2>&1

# Start container and run everything, then drop to shell
docker run -it \
    --privileged \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd)/start-kinefly-all.sh:/opt/Kinefly_docker/start-kinefly-all.sh:ro" \
    --name ${CONTAINER_NAME} \
    --entrypoint /bin/bash \
    kinefly \
    -c "/opt/Kinefly_docker/start-kinefly-all.sh ${PORT}; echo; echo 'Script ended. You are now in the container for debugging.'; echo 'Commands: kinefly [PORT] | status | test-data'; bash" 
