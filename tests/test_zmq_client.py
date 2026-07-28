#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import zmq
import json
import time


def test_zmq_bridge():
    print("🔍 Testing Kinefly ZMQ Bridge Connection...")
    print("📡 Connecting to ZMQ publisher on tcp://localhost:9871")

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:9871")
    socket.setsockopt(zmq.SUBSCRIBE, b"")  # Subscribe to all messages

    # Set a timeout so we don't wait forever
    socket.RCVTIMEO = 5000  # 5 seconds timeout

    print("⏳ Waiting for messages (5 second timeout)...")

    try:
        message_count = 0
        while True:
            try:
                message = socket.recv_string()
                data = json.loads(message)
                message_count += 1

                print("\n📊 === Message #{} ===".format(message_count))

                # Display message info
                if "message_info" in data:
                    info = data["message_info"]
                    print("🔖 Source: {}".format(info.get("source", "N/A")))
                    print("📋 Type: {}".format(info.get("message_type", "N/A")))
                    print("🎬 Frame: {}".format(info.get("frame_number", "N/A")))
                    print("🕒 Timestamp: {}".format(info.get("timestamp_ros", "N/A")))

                # Display wing tracking data
                if "wing_tracking" in data:
                    wings = data["wing_tracking"]

                    if "left_wing" in wings:
                        left = wings["left_wing"]
                        print("🦋 Left Wing:")
                        print(
                            "   📐 Angle: {:.4f} rad ({:.2f}°)".format(
                                left.get("angle_radians", "N/A"),
                                left.get("angle_degrees", "N/A"),
                            )
                        )
                        print(
                            "   🎯 Confidence: {:.3f}".format(
                                left.get("tracking_confidence", "N/A")
                            )
                        )
                        print(
                            "   🎵 Frequency: {:.2f} Hz".format(
                                left.get("beat_frequency_hz", "N/A")
                            )
                        )

                    if "right_wing" in wings:
                        right = wings["right_wing"]
                        print("🦋 Right Wing:")
                        print(
                            "   📐 Angle: {:.4f} rad ({:.2f}°)".format(
                                right.get("angle_radians", "N/A"),
                                right.get("angle_degrees", "N/A"),
                            )
                        )
                        print(
                            "   🎯 Confidence: {:.3f}".format(
                                right.get("tracking_confidence", "N/A")
                            )
                        )
                        print(
                            "   🎵 Frequency: {:.2f} Hz".format(
                                right.get("beat_frequency_hz", "N/A")
                            )
                        )

                # Display body parts tracking
                if "body_parts" in data:
                    body = data["body_parts"]
                    if "head" in body:
                        head = body["head"]
                        print(
                            "🗣️  Head: confidence={:.3f}".format(
                                head.get("tracking_confidence", "N/A")
                            )
                        )
                    if "abdomen" in body:
                        abdomen = body["abdomen"]
                        print(
                            "🔵 Abdomen: confidence={:.3f}".format(
                                abdomen.get("tracking_confidence", "N/A")
                            )
                        )

                # Show legacy format for compatibility
                if "legacy_unity_format" in data:
                    legacy = data["legacy_unity_format"]
                    print(
                        "🎮 Legacy Unity Format: x={:.4f}, y={:.4f}".format(
                            legacy.get("x", "N/A"), legacy.get("y", "N/A")
                        )
                    )

                print("─" * 60)

            except zmq.Again:
                print("⚠️  No messages received within timeout period")
                print("💡 This could mean:")
                print("   - Kinefly is not running")
                print("   - No flystate messages are being published")
                print("   - Bridge is not connected to ROS topic")
                break

    except KeyboardInterrupt:
        print("\n🛑 Stopped by user (received {} messages)".format(message_count))
    finally:
        socket.close()
        context.term()
        print("🔌 ZMQ connection closed")


if __name__ == "__main__":
    test_zmq_bridge()
