#!/usr/bin/env python3
"""
ROS2 Camera Publisher Node for Raspberry Pi
Publishes compressed images directly for efficient streaming over network.

Author: Genesis
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from picamera2 import Picamera2
import cv2
import numpy as np
from typing import Optional

import subprocess
from rclpy.node import Node
from std_msgs.msg import String

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('mediamtx_publisher')
        
        # Start MediaMTX process
        self.process = subprocess.Popen(
            ['/home/genesis/mediamtx', '/home/genesis/mediamtx.yml'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        # Publish stream URL
        self.pub = self.create_publisher(String, 'stream_info', 10)
        self.timer = self.create_timer(2.0, self.publish_url)

    def publish_url(self):
        msg = String()
        msg.data = '192.168.1.130:8889/cam/'
        self.pub.publish(msg)

    def destroy_node(self):
        self.process.terminate()
        super().destroy_node()

def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = None

    try:
        node = CameraPublisher()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f"Error: {e}")

    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()