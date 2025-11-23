#!/bin/bash

# Copy Configuration from Container Script
# Helps copy modified configurations from the running container back to host
# This is a convenience wrapper around sync-config-from-container.sh

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Try to detect container name from running containers
CONTAINER_NAME=""

# Check common container names
for name in "kinefly_dev" "kinefly_cam1" "kinefly_cam2" "kinefly_dual"; do
    if docker ps --format '{{.Names}}' | grep -q "^${name}$"; then
        CONTAINER_NAME="$name"
        break
    fi
done

# If no container found, ask user or use default
if [ -z "$CONTAINER_NAME" ]; then
    if [ ! -z "$1" ]; then
        CONTAINER_NAME="$1"
    else
        echo -e "${YELLOW}⚠️  No running Kinefly container detected${NC}"
        echo "Usage: $0 [CONTAINER_NAME]"
        echo "Or specify container name: $0 kinefly_dev"
        exit 1
    fi
fi

echo -e "${YELLOW}📋 Copying configurations from container: $CONTAINER_NAME${NC}"

# Use the sync script
"$SCRIPT_DIR/sync-config-from-container.sh" "$CONTAINER_NAME"

echo
echo -e "${GREEN}💡 Note: Changes are automatically synced on container exit.${NC}"
echo -e "${GREEN}💡 This script is useful for manual syncs while container is running.${NC}" 