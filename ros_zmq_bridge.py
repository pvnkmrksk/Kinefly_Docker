#!/usr/bin/env python2

import os
import sys
import json
import math
import click

# Add Kinefly message path to Python path
kinefly_msg_path = "/root/catkin/src/Kinefly/src"
if kinefly_msg_path not in sys.path:
    sys.path.insert(0, kinefly_msg_path)

# Debug information
print("\n=== Debug Information ===")
print("Python Path:")
for p in sys.path:
    print("  " + p)

print("\nChecking ROS Environment:")
print("ROS_ROOT: {}".format(os.environ.get("ROS_ROOT", "Not Set")))
print("ROS_PACKAGE_PATH: {}".format(os.environ.get("ROS_PACKAGE_PATH", "Not Set")))

try:
    import rospkg

    rospack = rospkg.RosPack()
    kinefly_path = rospack.get_path("Kinefly")
    print("\nKinefly package found at: {}".format(kinefly_path))
except Exception as e:
    print("Error finding Kinefly package: {}".format(e))

print("\nTrying to import ROS and Kinefly...")
try:
    import rospy

    print("Successfully imported rospy")
except ImportError as e:
    print("Failed to import rospy: {}".format(e))

try:
    from Kinefly.msg import MsgFlystate

    print("Successfully imported MsgFlystate")
except ImportError as e:
    print("Failed to import MsgFlystate: {}".format(e))

print("=== End Debug Information ===\n")

import zmq


def process_ros_message(msg, socket_zmq):
    """
    Process ROS messages from Kinefly and convert to a well-structured format for external applications.
    The message contains left and right wing data in the same message.
    """
    current_time = rospy.get_time()

    # Get the first angle from the arrays if available, otherwise use 0.0
    left_angle = msg.left.angles[0] if msg.left.angles else 0.0
    right_angle = msg.right.angles[0] if msg.right.angles else 0.0

    # Create well-structured message with meaningful headers
    kinefly_data = {
        "message_info": {
            "source": "kinefly",
            "message_type": "wing_tracking",
            "frame_number": msg.header.seq,
            "timestamp_ros": current_time,
            "timestamp_unix": current_time,
            "frame_id": msg.header.frame_id,
        },
        "wing_tracking": {
            "left_wing": {
                "angle_radians": left_angle,
                "angle_degrees": left_angle * 180.0 / 3.14159265359,
                "beat_frequency_hz": msg.left.freq,
                "tracking_confidence": msg.left.intensity,
                "gradient": msg.left.gradients[0] if msg.left.gradients else 0.0,
            },
            "right_wing": {
                "angle_radians": right_angle,
                "angle_degrees": right_angle * 180.0 / 3.14159265359,
                "beat_frequency_hz": msg.right.freq,
                "tracking_confidence": msg.right.intensity,
                "gradient": msg.right.gradients[0] if msg.right.gradients else 0.0,
            },
        },
        "body_parts": {
            "head": {
                "angle_radians": msg.head.angles[0] if msg.head.angles else 0.0,
                "tracking_confidence": msg.head.intensity,
                "radius": msg.head.radii[0] if msg.head.radii else 0.0,
            },
            "abdomen": {
                "angle_radians": msg.abdomen.angles[0] if msg.abdomen.angles else 0.0,
                "tracking_confidence": msg.abdomen.intensity,
                "radius": msg.abdomen.radii[0] if msg.abdomen.radii else 0.0,
            },
        },
        "legacy_unity_format": {
            "x": left_angle,  # For backward compatibility with Unity
            "y": right_angle,
            "z": 0.0,
            "yaw": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
        },
    }

    try:
        socket_zmq.send_string(json.dumps(kinefly_data))
    except Exception as e:
        rospy.logerr("Failed to send data over ZMQ. Error: {}".format(e))


def colored_print(message, color):
    color_codes = {"red": "\033[91m", "green": "\033[92m", "reset": "\033[0m"}
    print("{}{}{}".format(color_codes[color], message, color_codes["reset"]))


@click.command()
@click.option("--zmq-url", type=str, default="tcp://*:9871", help="ZMQ binding URL.")
@click.option(
    "--topic",
    type=str,
    default="/kinefly/flystate",
    help="Kinefly flystate topic",
)
def main(zmq_url, topic):
    """Bridge between Kinefly ROS topics and ZMQ, maintaining compatibility with existing Unity setup."""

    rospy.init_node("kinefly_zmq_bridge", anonymous=True)

    colored_print("\n\n\n\n\nInit ZMQ binding on {}...".format(zmq_url), "green")
    context = zmq.Context()
    socket_zmq = context.socket(zmq.PUB)
    socket_zmq.bind(zmq_url)

    colored_print("Subscribing to Kinefly topic: {}".format(topic), "green")

    # Subscribe to flystate topic
    sub = rospy.Subscriber(
        topic, MsgFlystate, lambda msg: process_ros_message(msg, socket_zmq)
    )

    colored_print("Bridge is running. Waiting for messages...", "green")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        colored_print("\nShutting down Kinefly-ZMQ bridge...", "red")
    finally:
        socket_zmq.close()
        context.term()


if __name__ == "__main__":
    main()
