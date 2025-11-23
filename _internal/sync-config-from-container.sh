#!/bin/bash

# Sync Configuration from Container Script
# Copies launch configs and config files from container back to host on exit

CONTAINER_NAME=$1

if [ -z "$CONTAINER_NAME" ]; then
    echo "Usage: $0 <container_name>"
    exit 1
fi

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Check if container exists (might be stopped)
if ! docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo -e "${YELLOW}⚠️  Container $CONTAINER_NAME not found, skipping sync${NC}"
    exit 0
fi

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo -e "${YELLOW}⚠️  Container $CONTAINER_NAME is stopped, attempting to sync from stopped container...${NC}"
    # For stopped containers, we can still copy files
fi

echo -e "${YELLOW}📋 Syncing configurations from container to host (on exit)...${NC}"

# Get the directory where this script is located (go up one level from _internal/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Create backup directory
BACKUP_DIR="$SCRIPT_DIR/config/backup/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"

echo -e "${YELLOW}📦 Creating backup in $BACKUP_DIR${NC}"

# Copy launch files from container
echo -e "${GREEN}🚀 Copying launch files...${NC}"
# Try to copy launch directory (works for both running and stopped containers)
if docker cp "$CONTAINER_NAME:/root/catkin/src/Kinefly/launch/" "$BACKUP_DIR/" 2>/dev/null; then
    # Copy to main launch directory (overwrite)
    if [ -d "$BACKUP_DIR/launch" ]; then
        cp -r "$BACKUP_DIR/launch/"* "$SCRIPT_DIR/launch/" 2>/dev/null || true
    fi
else
    echo -e "${YELLOW}⚠️  Could not copy launch files (container may be stopped)${NC}"
fi

# Copy config files from container
echo -e "${GREEN}📄 Copying config files...${NC}"
if docker cp "$CONTAINER_NAME:/root/kinefly.yaml" "$BACKUP_DIR/" 2>/dev/null; then
    cp "$BACKUP_DIR/kinefly.yaml" "$SCRIPT_DIR/config/kinefly.yaml" 2>/dev/null || true
else
    echo -e "${YELLOW}⚠️  Could not copy kinefly.yaml${NC}"
fi

# Copy VR-specific YAML files (VR1/VR1.yaml, VR2/VR2.yaml, etc.)
for vr in VR1 VR2 VR3 VR4; do
    if docker cp "$CONTAINER_NAME:/root/${vr}/${vr}.yaml" "$BACKUP_DIR/${vr}.yaml" 2>/dev/null; then
        # Create VR directory in config if it doesn't exist
        mkdir -p "$SCRIPT_DIR/config/${vr}"
        cp "$BACKUP_DIR/${vr}.yaml" "$SCRIPT_DIR/config/${vr}/${vr}.yaml" 2>/dev/null || true
    fi
done

# Copy ros_zmq_bridge.py if it exists (to _internal/)
echo -e "${GREEN}🌉 Copying ZMQ bridge...${NC}"
if docker cp "$CONTAINER_NAME:/root/catkin/src/Kinefly/launch/ros_zmq_bridge.py" "$BACKUP_DIR/" 2>/dev/null; then
    mkdir -p "$SCRIPT_DIR/_internal" 2>/dev/null || true
    cp "$BACKUP_DIR/ros_zmq_bridge.py" "$SCRIPT_DIR/_internal/ros_zmq_bridge.py" 2>/dev/null || true
else
    echo -e "${YELLOW}⚠️  Could not copy ros_zmq_bridge.py${NC}"
fi

echo -e "${GREEN}✅ Configuration sync complete${NC}"
echo -e "${YELLOW}📁 Backup location: $BACKUP_DIR${NC}"

