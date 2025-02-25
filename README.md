# Kinefly Docker Container

This repository contains a Dockerized version of Kinefly, a ROS-based system for real-time wing tracking. The container is built on Ubuntu 16.04 with ROS Kinetic.

## Prerequisites

- Docker installed on your system
- X11 server (for GUI applications)
- Git (for cloning the repository)

## Quick Start Guide

### 1. Clone the Repository

```bash
git clone https://github.com/pvnkmrksk/Kinefly_docker.git
cd Kinefly_docker
```

### 2. Build the Docker Image

```bash
docker build -t kinefly .
```

### 3. Run the Container

There are two ways to run the container:

#### Option 1: Using the Default Entry Point

This will run the container with the default ROS launch configuration:

```bash
docker run -it \
    --privileged \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    kinefly
```

#### Option 2: Interactive Shell Access

If you want to access the container's shell instead of running the default ROS launch:

```bash
docker run -it \
    --privileged \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --entrypoint /bin/bash \
    kinefly
```

### 4. Enable X11 Display (Required for GUI)

Before running the container, allow X server connections:

```bash
xhost +local:docker
```

## Container Features

- ROS Kinetic with full desktop installation
- OpenCV with Python bindings
- Phidgets support
- LED panels support
- USB camera support
- GUI tools (guvcview, gedit)

## Troubleshooting

1. If you see X11 connection errors:
   - Make sure you've run the `xhost` command
   - Verify your DISPLAY environment variable is set correctly

2. If USB devices are not detected:
   - Ensure the container is run with `--privileged` flag
   - Check USB device permissions on the host system

## Development

To modify the launch files or make changes to the configuration:

1. The custom launch files are located in `launch_files/rhag/`
2. After making changes, rebuild the Docker image

## Notes

- The container uses ROS Kinetic which is based on Ubuntu 16.04
- GUI applications are supported through X11 forwarding
- USB devices are accessible through the `--privileged` flag
- The container includes all necessary dependencies for Kinefly operation

## License

Please refer to the original Kinefly repository for license information. 