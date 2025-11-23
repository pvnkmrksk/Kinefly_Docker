# Kinefly Docker Setup

A Dockerized setup for running Kinefly (fly wing tracking system) with ROS Kinetic and ZMQ bridge for real-time data streaming.

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Usage](#usage)
  - [VR Mode (VR1-VR4)](#vr-mode-vr1-vr4)
  - [Legacy Single Camera Mode](#legacy-single-camera-mode)
- [Configuration](#configuration)
- [Multi-Camera Setup](#multi-camera-setup)
- [Testing](#testing)
- [Troubleshooting](#troubleshooting)
- [Project Structure](#project-structure)
- [ZMQ Data Format](#zmq-data-format)

## Features

✅ **One-command startup** - Start everything with `./kinefly`  
✅ **VR support** - Run VR1-VR4 individually or all at once  
✅ **Multi-camera support** - Legacy single/dual camera setups  
✅ **Automatic config sync** - Launch configs automatically copied from host to container  
✅ **Change preservation** - Container changes automatically synced back to host on exit  
✅ **ZMQ bridge** - Real-time data streaming via ZeroMQ  
✅ **Configurable ports** - Customize ZMQ ports to avoid conflicts  
✅ **Fast startup** - Optimized with minimal delays  

## Prerequisites

### All Platforms

- **Docker** (version 20.10 or later)
- **Git** (for cloning the repository)

### Ubuntu/Debian

- X11 server (usually pre-installed)
- `xhost` command (for X11 forwarding)

### Windows

- Docker Desktop for Windows
- X11 server (VcXsrv or Xming)

## Installation

### Ubuntu/Debian

1. **Install Docker:**
   ```bash
   sudo apt-get update
   sudo apt-get install -y \
       apt-transport-https \
       ca-certificates \
       curl \
       gnupg \
       lsb-release
   
   curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
   
   echo \
     "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
     $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
   
   sudo apt-get update
   sudo apt-get install -y docker-ce docker-ce-cli containerd.io
   
   sudo usermod -aG docker $USER
   # Log out and back in for this to take effect
   ```

2. **Install X11 utilities:**
   ```bash
   sudo apt-get install -y x11-xserver-utils
   ```

3. **Clone and build:**
   ```bash
   git clone <repository-url>
   cd Kinefly_docker
   docker build -t kinefly .
   ```

### Windows

1. **Install Docker Desktop:**
   - Download from: https://www.docker.com/products/docker-desktop
   - Install and start Docker Desktop

2. **Install X11 server (VcXsrv):**
   - Download from: https://sourceforge.net/projects/vcxsrv/
   - Install and run XLaunch
   - Select "Multiple windows" → "Start no client"
   - Check "Disable access control"

3. **Configure DISPLAY:**
   ```bash
   export DISPLAY=localhost:0.0
   ```

4. **Clone and build:**
   ```bash
   git clone <repository-url>
   cd Kinefly_docker
   docker build -t kinefly .
   ```

## Quick Start

### VR Mode (Recommended)

```bash
# Start all VRs (VR1-VR4)
./kinefly

# Start specific VR
./kinefly 1              # VR1 on port 9871
./kinefly 2              # VR2 on port 9872
./kinefly 1 9999         # VR1 on custom port 9999
```

### Legacy Single Camera Mode

```bash
# Start with default port (9871)
./kinefly 9871

# Or use legacy aliases inside container
# (after starting with ./kinefly 9871)
kinefly-cam1 [PORT]      # Camera 1 only
kinefly-cam2 [PORT]      # Camera 2 only
kinefly-dual [P1] [P2]   # Both cameras
```

## Usage

### VR Mode (VR1-VR4)

The main `kinefly` script supports VR mode:

```bash
# From host
./kinefly              # Start all VRs (VR1-VR4)
./kinefly 1            # Start VR1 only
./kinefly 2 9999       # Start VR2 with custom port 9999

# Inside container (after starting)
kinefly                # Start all VRs
kinefly 1              # Start VR1 only
kinefly 2 9999         # Start VR2 with custom port
```

**VR Configuration:**
- VR1: Port 9871, Video device `/dev/video4`
- VR2: Port 9872, Video device `/dev/video5`
- VR3: Port 9873, Video device `/dev/video6`
- VR4: Port 9874, Video device `/dev/video7`

**Topics:**
- VR1: `/VR1/VR1/flystate`
- VR2: `/VR2/VR2/flystate`
- VR3: `/VR3/VR3/flystate`
- VR4: `/VR4/VR4/flystate`

### Legacy Single Camera Mode

For backward compatibility, you can still use single camera mode:

```bash
# Start with port number (>=1024) to use legacy mode
./kinefly 9871

# Inside container
kinefly 9871            # Single camera on port 9871
kinefly-cam1 [PORT]     # Camera 1 only
kinefly-cam2 [PORT]     # Camera 2 only
kinefly-dual [P1] [P2]  # Both cameras
```

### Inside the Container

Once inside the container, you can use these commands:

```bash
kinefly [VR_ID] [PORT]  # VR mode (VR_ID 1-4)
kinefly [PORT]          # Legacy single camera mode
kinefly-cam1 [PORT]     # Camera 1 only
kinefly-cam2 [PORT]     # Camera 2 only
kinefly-dual [P1] [P2]  # Both cameras
status                  # List active ROS topics
test-data               # Test single camera data
test-cam1               # Test camera 1 data
test-cam2               # Test camera 2 data
```

## Configuration

### Automatic Configuration Sync ⚡

**The setup automatically handles configuration synchronization:**

1. **On Startup**: Launch configs and configuration files are automatically copied from the host to the container
2. **On Exit**: Any changes made inside the container are automatically synced back to the host

**You don't need to do anything** - just edit files either on the host or inside the container, and they'll stay in sync!

**How it works:**
- Edit files on the host → Changes are copied to container on next startup
- Edit files inside container → Changes are copied back to host on exit
- All changes are backed up in `config/backup/` with timestamps

### Manual Configuration

#### Edit Configuration Files

1. **On the host:**
   ```bash
   # Edit launch files
   nano launch/main.launch
   nano launch/VR1/params_kinefly.launch
   
   # Edit Kinefly config
   nano config/kinefly.yaml
   ```

2. **Changes are automatically synced** to the container on next startup

#### Edit Inside Container

1. **Start container:**
   ```bash
   ./kinefly 1
   ```

2. **Edit files inside container:**
   ```bash
   # Inside container
   nano /root/kinefly.yaml
   nano /root/catkin/src/Kinefly/launch/main.launch
   ```

3. **Exit container** - changes are automatically copied back to host

### Configuration Files

- **Kinefly Config**: `config/kinefly.yaml`
- **Launch Files**: `launch/` directory
  - `main.launch` - Main launch file
  - `VR1/` - VR1 configuration
  - `VR2/` - VR2 configuration
  - `VR3/` - VR3 configuration
  - `VR4/` - VR4 configuration

## Multi-Camera Setup

### Quick Reference

| Setup | Command | Default Ports | Video Device |
|-------|---------|---------------|--------------|
| All VRs | `./kinefly` | 9871-9874 | `/dev/video4-7` |
| VR1 | `./kinefly 1` | 9871 | `/dev/video4` |
| VR2 | `./kinefly 2` | 9872 | `/dev/video5` |
| VR3 | `./kinefly 3` | 9873 | `/dev/video6` |
| VR4 | `./kinefly 4` | 9874 | `/dev/video7` |
| Legacy Single | `./kinefly 9871` | 9871 | `/dev/video0` |
| Legacy Cam1 | `kinefly-cam1` | 9871 | `/dev/video4` |
| Legacy Cam2 | `kinefly-cam2` | 9872 | `/dev/video6` |
| Legacy Dual | `kinefly-dual` | 9871, 9872 | `/dev/video4,6` |

### Running Multiple VRs

Each VR runs independently with its own:
- ROS namespace (`/VR1`, `/VR2`, etc.)
- ZMQ port (9871, 9872, etc.)
- Video device (`/dev/video4`, `/dev/video5`, etc.)
- Topic (`/VR1/VR1/flystate`, `/VR2/VR2/flystate`, etc.)

### Customizing Video Sources

Edit the launch files to change video devices:

```bash
# Edit VR1 video device
nano launch/VR1/source_live.launch
# Change: <param name="video_device" value="/dev/video4"/>

# Edit VR2 video device
nano launch/VR2/source_live.launch
# Change: <param name="video_device" value="/dev/video5"/>
```

## Testing

### Test ZMQ Connection

From the host machine:

```bash
# Test VR1
python3 tests/test_zmq_client.py --zmq-url tcp://localhost:9871

# Test VR2
python3 tests/test_zmq_client.py --zmq-url tcp://localhost:9872

# Test legacy single camera
python3 tests/test_zmq_client.py --zmq-url tcp://localhost:9871
```

### Test Without Camera

1. Start container: `./kinefly 1`
2. In container, start test publisher:
   ```bash
   python2 /opt/Kinefly_docker/test_flystate_publisher.py
   ```
3. Test ZMQ connection from host:
   ```bash
   python3 tests/test_zmq_client.py --zmq-url tcp://localhost:9871
   ```

### Test Camera

1. Start container: `./kinefly 1`
2. In container, run camera test:
   ```bash
   /opt/Kinefly_docker/test_camera.sh
   ```

## Troubleshooting

### Docker Issues

**Problem: Permission denied**
```bash
# Linux: Add user to docker group
sudo usermod -aG docker $USER
# Log out and back in
```

**Problem: Docker daemon not running**
```bash
# Ubuntu/Debian
sudo systemctl start docker

# Windows: Start Docker Desktop application
```

### X11 Display Issues

**Problem: Cannot connect to X server**

**Linux:**
```bash
xhost +local:docker
```

**Windows (WSL2):**
```bash
# Make sure VcXsrv is running
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0
```

### Port Already in Use

```bash
# Check what's using the port
netstat -tlnp | grep :9871

# Use a different port
./kinefly 1 9999
```

### Camera Not Found

**Linux:**
```bash
# List available video devices
ls -la /dev/video*

# Check camera permissions
sudo chmod 666 /dev/video4
```

### Container Changes Not Preserved

If changes aren't being preserved:
1. Make sure you're exiting the container normally (not killing it)
2. Check that the sync script ran on exit
3. The sync happens automatically on container exit

### VR Not Starting

**Problem: "Permission denied" on start-kinefly-vr.sh**
```bash
# Make sure scripts are executable
chmod +x _internal/*.sh
```

**Problem: "RIG environment variable" errors**
- The script automatically sets RIG to VR1-VR4
- If you see "rhag" errors, rebuild the container: `docker build -t kinefly .`

## Project Structure

```
Kinefly_docker/
├── Dockerfile              # Docker image definition
├── README.md              # This file
├── kinefly                # Main entry script (VR mode + legacy)
├── launch/                # ROS launch files
│   ├── main.launch        # Main launch file
│   ├── VR1/               # VR1 configuration
│   ├── VR2/               # VR2 configuration
│   ├── VR3/               # VR3 configuration
│   └── VR4/               # VR4 configuration
├── config/                # Configuration files
│   ├── kinefly.yaml       # Main Kinefly config
│   └── backup/            # Automatic backups
└── tests/                 # Test scripts
    ├── test_zmq_client.py
    ├── test_camera.sh
    └── test_flystate_publisher.py
```

**Internal files (in `_internal/`):**
- All `dev-*` and `start-*` scripts
- Utility scripts (`copy-config.sh`, `sync-config-*.sh`)
- Additional documentation
- Python dependencies

## ZMQ Data Format

The ZMQ bridge publishes JSON data in the following format:

```json
{
  "x": 0.123,     // left wing angle (radians)
  "y": -0.456,    // right wing angle (radians)
  "z": 0.0,       // always 0
  "yaw": 0.579,   // difference (x - y)
  "pitch": 0.0,   // always 0
  "roll": 0.0     // always 0
}
```

### Connecting to ZMQ

**Python example:**
```python
import zmq
import json

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:9871")  # VR1 port
socket.setsockopt(zmq.SUBSCRIBE, b"")

while True:
    data = json.loads(socket.recv_string())
    print(f"VR1 data: {data}")
```

**Multiple VRs:**
```python
import zmq
import json

# Connect to multiple VRs
vr1 = zmq.Context().socket(zmq.SUB)
vr1.connect("tcp://localhost:9871")
vr1.setsockopt(zmq.SUBSCRIBE, b"")

vr2 = zmq.Context().socket(zmq.SUB)
vr2.connect("tcp://localhost:9872")
vr2.setsockopt(zmq.SUBSCRIBE, b"")

# Poll for messages
poller = zmq.Poller()
poller.register(vr1, zmq.POLLIN)
poller.register(vr2, zmq.POLLIN)

while True:
    socks = dict(poller.poll(100))
    if vr1 in socks:
        data = json.loads(vr1.recv_string())
        print(f"VR1: {data}")
    if vr2 in socks:
        data = json.loads(vr2.recv_string())
        print(f"VR2: {data}")
```

## License

[Add your license information here]

## Support

For issues and questions, please [open an issue](repository-url/issues) or contact the maintainers.
