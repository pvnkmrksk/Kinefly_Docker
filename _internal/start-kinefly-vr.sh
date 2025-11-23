#!/bin/bash

# Unified Kinefly VR Startup Script
# Starts Kinefly for VR1-VR4 with configurable selection
# Usage: start-kinefly-vr.sh [VR_NUMBER] [PORT]
#   - No arguments: starts all VRs (VR1-VR4)
#   - VR_NUMBER (1-4): starts specific VR
#   - PORT: optional custom ZMQ port (default: 9871 + VR_NUMBER - 1)

# Source ROS environment
source /opt/ros/kinetic/setup.bash
source /root/catkin/devel/setup.bash
export PYTHONPATH=/root/catkin/src/Kinefly/src:$PYTHONPATH

# Constants
MAX_VR=4
BASE_PORT=9871

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Parse arguments
VR_NUMBER=$1
CUSTOM_PORT=$2

# Validate VR number if provided
if [ ! -z "$VR_NUMBER" ]; then
    if ! [[ "$VR_NUMBER" =~ ^[0-9]+$ ]] || [ "$VR_NUMBER" -lt 1 ] || [ "$VR_NUMBER" -gt $MAX_VR ]; then
        echo -e "${RED}❌ Invalid VR number: $VR_NUMBER${NC}"
        echo "Usage: $0 [VR_NUMBER] [PORT]"
        echo "VR_NUMBER must be between 1 and $MAX_VR"
        echo "If VR_NUMBER is not specified, all VRs (1-$MAX_VR) will be started"
        exit 1
    fi
fi

# Function to start a single VR
start_vr() {
    local vr_num=$1
    local vr_name="VR${vr_num}"
    local port=${2:-$((BASE_PORT + vr_num - 1))}
    
    # Starting ${vr_name} on port ${port}
    
    # Create directory for VR config files (kinefly.py writes to /root/VR1/VR1.yaml, etc.)
    mkdir -p /root/${vr_name}
    
    # Unset any existing RIG variable and set it explicitly for this VR
    # This prevents inheritance from .bashrc or other sources (like "rhag")
    unset RIG
    export RIG=$vr_name
    
    # Verify RIG is set correctly
    if [ "$RIG" != "$vr_name" ]; then
        echo -e "${RED}❌ Failed to set RIG environment variable${NC}"
        echo -e "${RED}   Expected: $vr_name, Got: $RIG${NC}"
        return 1
    fi
    
    # Start Kinefly for this VR
    # Explicitly pass RIG in the environment to ensure it's available to roslaunch
    RIG=$vr_name roslaunch Kinefly main.launch > /tmp/kinefly_${vr_name}.log 2>&1 &
    local kinefly_pid=$!
    
    # Quick check if process started
    sleep 0.5
    if ! kill -0 $kinefly_pid 2>/dev/null; then
        echo -e "${RED}❌ ${vr_name} failed to start${NC}"
        echo "Check log: tail /tmp/kinefly_${vr_name}.log"
        return 1
    fi
    
    # Wait for topic to be available (reduced timeout and faster polling)
    local topic="/${vr_name}/${vr_name}/flystate"
    timeout=10
    counter=0
    while [ $counter -lt $timeout ]; do
        if rostopic list 2>/dev/null | grep -q "$topic"; then
            break
        fi
        sleep 0.2
        counter=$((counter + 1))
    done
    
    if [ $counter -eq $timeout ]; then
        echo -e "${RED}❌ ${vr_name} topic not found after ${timeout}s${NC}"
        return 1
    fi
    
    # Start ZMQ bridge
    cd /root/catkin/src/Kinefly/launch/
    python2 ros_zmq_bridge.py --zmq-url "tcp://*:${port}" --topic "$topic" > /tmp/zmq_bridge_${vr_name}.log 2>&1 &
    local bridge_pid=$!
    
    sleep 0.5
    
    if ! kill -0 $bridge_pid 2>/dev/null; then
        echo -e "${RED}❌ ZMQ Bridge for ${vr_name} failed to start${NC}"
        echo "Check log: tail /tmp/zmq_bridge_${vr_name}.log"
        return 1
    fi
    
    echo -e "${GREEN}✅ ${vr_name} running (Port: ${port})${NC}"
    
    # Store PIDs with VR name for isolated cleanup
    echo "${vr_name}:${kinefly_pid}:${bridge_pid}" >> /tmp/kinefly_vr_pids.txt
}

# Global flag to exit monitoring loop
CLEANUP_REQUESTED=0

# Function to cleanup a specific VR
cleanup_vr() {
    local vr_name=$1
    local kinefly_pid=$2
    local bridge_pid=$3
    
    echo -e "${YELLOW}🛑 Shutting down ${vr_name}...${NC}"
    
    # Kill only this VR's processes
    if [ ! -z "$kinefly_pid" ] && kill -0 "$kinefly_pid" 2>/dev/null; then
        echo -e "${YELLOW}   Stopping ${vr_name} Kinefly (PID: $kinefly_pid)${NC}"
        kill "$kinefly_pid" 2>/dev/null
        wait "$kinefly_pid" 2>/dev/null || true
    fi
    
    if [ ! -z "$bridge_pid" ] && kill -0 "$bridge_pid" 2>/dev/null; then
        echo -e "${YELLOW}   Stopping ${vr_name} ZMQ Bridge (PID: $bridge_pid)${NC}"
        kill "$bridge_pid" 2>/dev/null
        wait "$bridge_pid" 2>/dev/null || true
    fi
    
    # Kill only this VR's roslaunch (by namespace)
    pkill -f "roslaunch.*${vr_name}" 2>/dev/null || true
    
    echo -e "${GREEN}✅ ${vr_name} stopped${NC}"
}

# Function to cleanup all processes (on Ctrl+C or script exit)
cleanup_all() {
    CLEANUP_REQUESTED=1
    echo -e "\n${YELLOW}🛑 Shutting down all VRs...${NC}"
    
    # Kill all stored VRs individually
    if [ -f /tmp/kinefly_vr_pids.txt ]; then
        while IFS=':' read -r vr_name kinefly_pid bridge_pid; do
            if [ ! -z "$vr_name" ]; then
                cleanup_vr "$vr_name" "$kinefly_pid" "$bridge_pid"
            fi
        done < /tmp/kinefly_vr_pids.txt
        rm -f /tmp/kinefly_vr_pids.txt
    fi
    
    echo -e "${GREEN}✅ All VRs stopped${NC}"
}

# Set up signal handlers
trap cleanup_all SIGINT SIGTERM

# Initialize PID file
rm -f /tmp/kinefly_vr_pids.txt
touch /tmp/kinefly_vr_pids.txt

# Ensure ROS master is available
if ! pgrep -f "rosmaster" > /dev/null; then
    roscore > /tmp/roscore.log 2>&1 &
    sleep 1
fi

# Start VR(s)
if [ -z "$VR_NUMBER" ]; then
    # Start all VRs
    for i in $(seq 1 $MAX_VR); do
        start_vr $i
    done
else
    # Start specific VR
    start_vr $VR_NUMBER $CUSTOM_PORT
fi


# Monitor processes
while [ $CLEANUP_REQUESTED -eq 0 ]; do
    # Check if any process died (isolated per VR)
    if [ -f /tmp/kinefly_vr_pids.txt ]; then
        # Create temp file for remaining PIDs
        temp_file=$(mktemp)
        
        while IFS=':' read -r vr_name kinefly_pid bridge_pid; do
            if [ -z "$vr_name" ]; then
                continue
            fi
            
            vr_failed=false
            
            # Check Kinefly process
            if [ ! -z "$kinefly_pid" ] && ! kill -0 "$kinefly_pid" 2>/dev/null; then
                echo -e "${RED}❌ ${vr_name} Kinefly (PID: $kinefly_pid) stopped unexpectedly${NC}"
                vr_failed=true
            fi
            
            # Check Bridge process
            if [ ! -z "$bridge_pid" ] && ! kill -0 "$bridge_pid" 2>/dev/null; then
                echo -e "${RED}❌ ${vr_name} ZMQ Bridge (PID: $bridge_pid) stopped unexpectedly${NC}"
                vr_failed=true
            fi
            
            if [ "$vr_failed" = true ]; then
                # Clean up only this VR, don't kill others
                cleanup_vr "$vr_name" "$kinefly_pid" "$bridge_pid"
                echo -e "${YELLOW}⚠️  ${vr_name} stopped, but other VRs continue running${NC}"
            else
                # VR is still running, keep it in the list
                echo "${vr_name}:${kinefly_pid}:${bridge_pid}" >> "$temp_file"
            fi
        done < /tmp/kinefly_vr_pids.txt
        
        # Replace PID file with remaining VRs
        mv "$temp_file" /tmp/kinefly_vr_pids.txt 2>/dev/null || true
    fi
    sleep 2
done

echo -e "${GREEN}✅ Monitoring stopped${NC}"

# Script ends here - caller (dev-kinefly-vr.sh) will drop to console

