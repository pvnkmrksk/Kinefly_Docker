# Configuration Management

This directory contains configuration files for Kinefly.

## Current Files
- `kinefly.yaml` - Main Kinefly configuration file

## How to Modify Configurations

### Method 1: Edit in Container (Recommended for Testing)
1. Start the container: `./dev-kinefly.sh`
2. Once inside the container, edit the configuration files:
   ```bash
   # Edit Kinefly config
   nano /root/kinefly.yaml
   
   # Edit launch files
   nano /root/catkin/src/Kinefly/launch/main.launch
   ```
3. Test your changes
4. When satisfied, copy the files back to your host:

### Method 2: Copy from Container to Host
```bash
# Copy Kinefly config
docker cp kinefly_dev:/root/kinefly.yaml ./config/

# Copy launch files
docker cp kinefly_dev:/root/catkin/src/Kinefly/launch/ ./launch/
```

### Method 3: Rebuild Container
After copying files, rebuild the container to apply changes:
```bash
docker build -t kinefly .
```

## File Locations in Container
- Kinefly config: `/root/kinefly.yaml`
- Launch files: `/root/catkin/src/Kinefly/launch/`
- ZMQ bridge: `/root/catkin/src/Kinefly/launch/ros_zmq_bridge.py` 