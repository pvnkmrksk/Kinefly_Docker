#!/bin/bash

# Copy Configuration from Container Script
# Helps copy modified configurations from the running container back to host

CONTAINER_NAME="kinefly_dev"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}📋 Copying configurations from container...${NC}"

# Check if container is running
if ! docker ps | grep -q $CONTAINER_NAME; then
    echo -e "${RED}❌ Container $CONTAINER_NAME is not running${NC}"
    echo "Start the container first: ./dev-kinefly.sh"
    exit 1
fi

# Create backup directory
BACKUP_DIR="config/backup/$(date +%Y%m%d_%H%M%S)"
mkdir -p $BACKUP_DIR

echo -e "${YELLOW}📦 Creating backup in $BACKUP_DIR${NC}"

# Copy Kinefly config
echo -e "${GREEN}📄 Copying kinefly.yaml...${NC}"
docker cp $CONTAINER_NAME:/root/kinefly.yaml $BACKUP_DIR/
cp $BACKUP_DIR/kinefly.yaml config/

# Copy launch files
echo -e "${GREEN}🚀 Copying launch files...${NC}"
docker cp $CONTAINER_NAME:/root/catkin/src/Kinefly/launch/ $BACKUP_DIR/

# Copy ZMQ bridge
echo -e "${GREEN}🌉 Copying ZMQ bridge...${NC}"
docker cp $CONTAINER_NAME:/root/catkin/src/Kinefly/launch/ros_zmq_bridge.py $BACKUP_DIR/

echo -e "${GREEN}✅ Configuration copied successfully!${NC}"
echo -e "${YELLOW}📁 Backup location: $BACKUP_DIR${NC}"
echo -e "${YELLOW}📁 Updated files: config/kinefly.yaml, launch/${NC}"
echo
echo -e "${GREEN}💡 To apply changes, rebuild the container:${NC}"
echo -e "   docker build -t kinefly ." 