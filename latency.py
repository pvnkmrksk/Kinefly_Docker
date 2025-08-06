import zmq
import json
import time

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:9871")
socket.setsockopt_string(zmq.SUBSCRIBE, "")

while True:
    msg = socket.recv_string()
    data = json.loads(msg)
    now = time.time()
    latency = now - data["timestamp_unix"]
    print("Latency: {:.3f} ms".format(latency * 1000))
