# Test Files

This directory contains test files for Kinefly development and debugging.

## Available Tests

### `test_zmq_client.py`
Test ZMQ connection and data reception from the bridge.
```bash
python3 test_zmq_client.py --zmq-url tcp://localhost:9871
```

### `test_camera.sh`
Test camera detection and configuration.
```bash
./test_camera.sh
```

### `test_flystate_publisher.py`
Publish test flystate messages without requiring a camera.
```bash
# Inside container
python2 /opt/Kinefly_docker/test_flystate_publisher.py
```

## Usage

### Testing ZMQ Bridge
1. Start Kinefly: `./dev-kinefly.sh`
2. Test connection: `python3 tests/test_zmq_client.py --zmq-url tcp://localhost:9871`

### Testing Without Camera
1. Start container: `./dev-kinefly.sh`
2. In container, start test publisher:
   ```bash
   python2 /opt/Kinefly_docker/test_flystate_publisher.py
   ```
3. Test ZMQ connection from host:
   ```bash
   python3 tests/test_zmq_client.py --zmq-url tcp://localhost:9871
   ```

### Testing Camera
1. Start container: `./dev-kinefly.sh`
2. In container, run camera test:
   ```bash
   /opt/Kinefly_docker/test_camera.sh
   ``` 