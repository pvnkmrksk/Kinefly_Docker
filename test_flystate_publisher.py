#!/usr/bin/env python2

import rospy
import sys
import os
import math
import time

# Add Kinefly message path to Python path
kinefly_msg_path = "/root/catkin/src/Kinefly/src"
if kinefly_msg_path not in sys.path:
    sys.path.insert(0, kinefly_msg_path)

from Kinefly.msg import MsgFlystate, MsgState
from std_msgs.msg import Header


def create_test_flystate():
    """Create a test flystate message with realistic data"""
    msg = MsgFlystate()

    # Header
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "camera"
    msg.header.seq = 0

    # Left wing
    msg.left = MsgState()
    msg.left.angles = [0.5]  # 0.5 radians
    msg.left.freq = 200.0  # 200 Hz
    msg.left.intensity = 0.8
    msg.left.gradients = [0.1]

    # Right wing
    msg.right = MsgState()
    msg.right.angles = [-0.3]  # -0.3 radians
    msg.right.freq = 205.0  # 205 Hz
    msg.right.intensity = 0.9
    msg.right.gradients = [-0.05]

    # Head
    msg.head = MsgState()
    msg.head.angles = [0.0]
    msg.head.intensity = 0.7
    msg.head.radii = [5.0]

    # Abdomen
    msg.abdomen = MsgState()
    msg.abdomen.angles = [0.1]
    msg.abdomen.intensity = 0.6
    msg.abdomen.radii = [3.0]

    return msg


def main():
    rospy.init_node("test_flystate_publisher", anonymous=True)

    pub = rospy.Publisher("/kinefly/flystate", MsgFlystate, queue_size=10)

    rate = rospy.Rate(30)  # 30 Hz
    seq = 0

    print("Publishing test flystate messages on /kinefly/flystate at 30 Hz")
    print("Press Ctrl+C to stop")

    try:
        while not rospy.is_shutdown():
            msg = create_test_flystate()
            msg.header.seq = seq

            # Add some variation to make it more realistic
            t = time.time()
            msg.left.angles[0] = 0.5 + 0.2 * math.sin(t * 10)  # Wing beating
            msg.right.angles[0] = -0.3 + 0.2 * math.sin(
                t * 10 + math.pi
            )  # Opposite phase

            pub.publish(msg)
            seq += 1
            rate.sleep()

    except KeyboardInterrupt:
        print("\nShutting down test publisher")


if __name__ == "__main__":
    main()
