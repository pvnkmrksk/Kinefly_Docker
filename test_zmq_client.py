#!/usr/bin/env python3

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

                print(f"\n📊 === Message #{message_count} ===")

                # Display message info
                if "message_info" in data:
                    info = data["message_info"]
                    print(f"🔖 Source: {info.get('source', 'N/A')}")
                    print(f"📋 Type: {info.get('message_type', 'N/A')}")
                    print(f"🎬 Frame: {info.get('frame_number', 'N/A')}")
                    print(f"🕒 Timestamp: {info.get('timestamp_ros', 'N/A')}")

                # Display wing tracking data
                if "wing_tracking" in data:
                    wings = data["wing_tracking"]

                    if "left_wing" in wings:
                        left = wings["left_wing"]
                        print(f"🦋 Left Wing:")
                        print(
                            f"   📐 Angle: {left.get('angle_radians', 'N/A'):.4f} rad ({left.get('angle_degrees', 'N/A'):.2f}°)"
                        )
                        print(
                            f"   🎯 Confidence: {left.get('tracking_confidence', 'N/A'):.3f}"
                        )
                        print(
                            f"   🎵 Frequency: {left.get('beat_frequency_hz', 'N/A'):.2f} Hz"
                        )

                    if "right_wing" in wings:
                        right = wings["right_wing"]
                        print(f"🦋 Right Wing:")
                        print(
                            f"   📐 Angle: {right.get('angle_radians', 'N/A'):.4f} rad ({right.get('angle_degrees', 'N/A'):.2f}°)"
                        )
                        print(
                            f"   🎯 Confidence: {right.get('tracking_confidence', 'N/A'):.3f}"
                        )
                        print(
                            f"   🎵 Frequency: {right.get('beat_frequency_hz', 'N/A'):.2f} Hz"
                        )

                # Display body parts tracking
                if "body_parts" in data:
                    body = data["body_parts"]
                    if "head" in body:
                        head = body["head"]
                        print(
                            f"🗣️  Head: confidence={head.get('tracking_confidence', 'N/A'):.3f}"
                        )
                    if "abdomen" in body:
                        abdomen = body["abdomen"]
                        print(
                            f"🔵 Abdomen: confidence={abdomen.get('tracking_confidence', 'N/A'):.3f}"
                        )

                # Show legacy format for compatibility
                if "legacy_unity_format" in data:
                    legacy = data["legacy_unity_format"]
                    print(
                        f"🎮 Legacy Unity Format: x={legacy.get('x', 'N/A'):.4f}, y={legacy.get('y', 'N/A'):.4f}"
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
        print(f"\n🛑 Stopped by user (received {message_count} messages)")
    finally:
        socket.close()
        context.term()
        print("🔌 ZMQ connection closed")


if __name__ == "__main__":
    test_zmq_bridge()
