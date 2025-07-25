# Kinefly ZMQ Bridge

Simple bridge connecting Kinefly ROS topics to ZMQ for real-time wing tracking data.

## ⚡ One Command to Start Everything

```bash
./dev-kinefly.sh          # Start Kinefly + ZMQ bridge on port 9871
./dev-kinefly.sh 9872     # Custom port
```

That's it! Press Ctrl+C to stop everything gracefully.

## Container Commands

Once inside the container, you can also use:
```bash
kinefly           # Start everything on port 9871
kinefly 9872      # Start everything on custom port
status            # Check ROS topics and cameras  
test-data         # Test if Kinefly is publishing data
```

## ZMQ Output

Simple JSON format:
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

## Features

✅ **One command startup** - Everything starts together  
✅ **Configurable port** - No more port conflicts  
✅ **Graceful shutdown** - Ctrl+C stops everything cleanly  
✅ **Automatic cleanup** - Handles process management  
✅ **Port validation** - Prevents conflicts before starting  
✅ **Error handling** - Clear error messages and recovery

## Files

**Essential files only:**
- `dev-kinefly.sh` - Host script to start everything
- `start-kinefly-all.sh` - Container script (all-in-one)
- `ros_zmq_bridge.py` - Core ZMQ bridge
- `test_zmq_client.py` - Test ZMQ connection