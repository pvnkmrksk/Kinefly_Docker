# Kinefly Docker Setup

A Dockerized setup for running Kinefly (fly wing tracking system) with ROS Kinetic and ZMQ bridge for real-time data streaming.

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [Ubuntu/Debian](#ubuntudebian)
  - [Windows](#windows)
- [Quick Start](#quick-start)
- [Usage](#usage)
- [Configuration](#configuration)
- [Multi-Camera Setup](#multi-camera-setup)
- [Troubleshooting](#troubleshooting)
- [Project Structure](#project-structure)

## Features

✅ **One-command startup** - Start everything with a single command  
✅ **Multi-camera support** - Run single or dual camera setups  
✅ **Automatic config sync** - Launch configs automatically copied from host to container  
✅ **Change preservation** - Container changes automatically synced back to host on exit  
✅ **ZMQ bridge** - Real-time data streaming via ZeroMQ  
✅ **Configurable ports** - Customize ZMQ ports to avoid conflicts  
✅ **Graceful shutdown** - Clean process management with Ctrl+C  

## Prerequisites

### All Platforms

- **Docker** (version 20.10 or later)
- **Docker Compose** (optional, for advanced setups)
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
   # Update package index
   sudo apt-get update
   
   # Install prerequisites
   sudo apt-get install -y \
       apt-transport-https \
       ca-certificates \
       curl \
       gnupg \
       lsb-release
   
   # Add Docker's official GPG key
   curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
   
   # Set up stable repository
   echo \
     "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
     $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
   
   # Install Docker Engine
   sudo apt-get update
   sudo apt-get install -y docker-ce docker-ce-cli containerd.io
   
   # Add your user to docker group (to run without sudo)
   sudo usermod -aG docker $USER
   # Log out and back in for this to take effect
   ```

2. **Install X11 utilities (if not already installed):**
   ```bash
   sudo apt-get install -y x11-xserver-utils
   ```

3. **Clone and build:**
   ```bash
   git clone <repository-url>
   cd Kinefly_Docker
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

3. **Configure DISPLAY in Git Bash:**
   ```bash
   # Get Windows host IP (usually from /etc/resolv.conf or use localhost)
   export DISPLAY=localhost:0.0
   # Or if using WSL2 IP:
   # export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0
   ```

4. **Clone and build:**
   ```bash
   git clone <repository-url>
   cd Kinefly_Docker
   docker build -t kinefly .
   ```

## Quick Start

### Single Camera Setup

```bash
# Start with default port (9871)
./dev-kinefly.sh

# Start with custom port
./dev-kinefly.sh 9872
```

### Multi-Camera Setup

```bash
# Camera 1 only
./dev-kinefly-cam1.sh [PORT]

# Camera 2 only
./dev-kinefly-cam2.sh [PORT]

# Both cameras in same container
./dev-kinefly-dual.sh [CAM1_PORT] [CAM2_PORT]
```

## Usage

### Inside the Container

Once inside the container, you can use these aliases:

```bash
kinefly [PORT]         # Start single camera (default: 9871)
kinefly-cam1 [PORT]    # Start camera 1 (default: 9871)
kinefly-cam2 [PORT]    # Start camera 2 (default: 9872)
kinefly-dual [P1] [P2] # Start both cameras
status                 # List active ROS topics
test-data              # Test single camera data
test-cam1              # Test camera 1 data
test-cam2              # Test camera 2 data
```

### Testing ZMQ Connection

From the host machine:

```bash
# Test single camera
python3 tests/test_zmq_client.py --zmq-url tcp://localhost:9871

# Test camera 1
python3 tests/test_zmq_client.py --zmq-url tcp://localhost:9871

# Test camera 2
python3 tests/test_zmq_client.py --zmq-url tcp://localhost:9872
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
   nano launch/rhag/params_kinefly.launch
   
   # Edit Kinefly config
   nano config/kinefly.yaml
   ```

2. **Changes are automatically synced** to the container on next startup

#### Edit Inside Container

1. **Start container:**
   ```bash
   ./dev-kinefly.sh
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
  - `rhag/` - Single camera configuration
  - `rhag_cam1/` - Camera 1 configuration
  - `rhag_cam2/` - Camera 2 configuration

## Multi-Camera Setup

See [MULTI_CAMERA_SETUP.md](MULTI_CAMERA_SETUP.md) for detailed multi-camera documentation.

### Quick Reference

| Setup | Script | Default Ports | Video Device |
|-------|--------|---------------|--------------|
| Single | `./dev-kinefly.sh` | 9871 | `/dev/video0` |
| Camera 1 | `./dev-kinefly-cam1.sh` | 9871 | `/dev/video4` |
| Camera 2 | `./dev-kinefly-cam2.sh` | 9872 | `/dev/video6` |
| Dual | `./dev-kinefly-dual.sh` | 9871, 9872 | `/dev/video4`, `/dev/video6` |

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
./dev-kinefly.sh 9873
```

### Camera Not Found

**Linux:**
```bash
# List available video devices
ls -la /dev/video*

# Check camera permissions
sudo chmod 666 /dev/video0
```


### Container Changes Not Preserved

If changes aren't being preserved:
1. Make sure you're exiting the container normally (not killing it)
2. Check that the sync script ran on exit
3. Manually copy configs: `./copy-config.sh`

## Project Structure

```
Kinefly_Docker/
├── Dockerfile                  # Docker image definition
├── README.md                  # This file
├── QUICK_START.md            # Quick start guide
├── MANUAL_SETUP.md           # Manual setup instructions
├── MULTI_CAMERA_SETUP.md     # Multi-camera guide
├── dev-kinefly.sh            # Main startup script (single camera)
├── dev-kinefly-cam1.sh       # Camera 1 startup script
├── dev-kinefly-cam2.sh       # Camera 2 startup script
├── dev-kinefly-dual.sh       # Dual camera startup script
├── copy-config.sh            # Manual config copy script
├── start-kinefly-all.sh      # Container startup (single)
├── start-kinefly-cam1.sh     # Container startup (cam1)
├── start-kinefly-cam2.sh     # Container startup (cam2)
├── start-kinefly-dual.sh     # Container startup (dual)
├── ros_zmq_bridge.py         # ZMQ bridge script
├── requirements.txt          # Python dependencies
├── config/                   # Configuration files
│   ├── kinefly.yaml         # Main Kinefly config
│   └── README.md            # Config documentation
├── launch/                   # ROS launch files
│   ├── main.launch          # Main launch file
│   ├── rhag/                # Single camera config
│   ├── rhag_cam1/           # Camera 1 config
│   └── rhag_cam2/           # Camera 2 config
└── tests/                    # Test scripts
    ├── test_zmq_client.py    # ZMQ client test
    ├── test_camera.sh        # Camera test
    ├── test_flystate_publisher.py  # Test publisher
    └── README.md             # Test documentation
```

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

## Additional Resources

- [Quick Start Guide](QUICK_START.md)
- [Manual Setup Guide](MANUAL_SETUP.md)
- [Multi-Camera Setup](MULTI_CAMERA_SETUP.md)
- [Configuration Guide](config/README.md)
- [Test Documentation](tests/README.md)

## License

[Add your license information here]

## Support

For issues and questions, please [open an issue](repository-url/issues) or contact the maintainers.
