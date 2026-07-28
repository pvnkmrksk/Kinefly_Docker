# Quick Start Guide

## 🚀 One Command to Start Everything

```bash
./dev-kinefly.sh          # Start on default port 9871
./dev-kinefly.sh 9872     # Start on custom port
```

## 📁 Clean Project Structure

```
Kinefly_docker/
├── dev-kinefly.sh           # 🎯 Main startup script
├── start-kinefly-all.sh     # Container startup script
├── copy-config.sh           # Copy configs from container
├── config/                  # 📄 Configuration files
│   ├── kinefly.yaml
│   └── README.md
├── launch/                  # 🚀 ROS launch files
├── tests/                   # 🧪 Test files
│   ├── test_zmq_client.py
│   ├── test_camera.sh
│   ├── test_flystate_publisher.py
│   └── README.md
├── ros_zmq_bridge.py        # 🌉 Core ZMQ bridge
└── Dockerfile               # 🐳 Container definition
```

## 🔧 Configuration Workflow

### 1. Edit in Container (Recommended)
```bash
./dev-kinefly.sh
# Once inside container:
nano /root/kinefly.yaml                    # Edit Kinefly config
nano /root/catkin/src/Kinefly/launch/main.launch  # Edit launch config
```

### 2. Copy Changes Back
```bash
./copy-config.sh  # Copies all configs from container
```

### 3. Rebuild Container
```bash
docker build -t kinefly .
```

## 🧪 Testing

### Test ZMQ Connection
```bash
python2 tests/test_zmq_client.py --zmq-url tcp://localhost:9871
```

### Test Without Camera
```bash
# In container:
python2 /opt/Kinefly_docker/test_flystate_publisher.py
# From host:
python2 tests/test_zmq_client.py --zmq-url tcp://localhost:9871
```

## 🎯 What Was Cleaned Up

✅ **Removed redundant scripts** - No more `entrypoint.sh`  
✅ **Organized test files** - Moved to `tests/` directory  
✅ **Created config management** - Easy edit/copy workflow  
✅ **Simplified documentation** - Clear, focused guides  
✅ **Added .gitignore** - Clean repository  
✅ **Removed empty directories** - No clutter  

## 💡 Key Benefits

- **One command startup** - `./dev-kinefly.sh`
- **Easy configuration** - Edit in container, copy back
- **Clean structure** - Everything organized
- **Simple testing** - All test files in one place
- **Clear documentation** - No confusion about what to use
