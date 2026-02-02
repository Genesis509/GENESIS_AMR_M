#!/usr/bin/env python3
"""
ROS2 Camera Handler for God boy mode
Sends tracking error via UART to low-level controller
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from picamera2 import Picamera2
import cv2
import numpy as np
import serial
import subprocess
from vision_msgs.msg import Detection2D, BoundingBox2D
from geometry_msgs.msg import Pose2D
#distance goal : 100 000px²

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
        # bbox publisher
        self.bbox_pub = self.create_publisher(Detection2D, 'camera/bounding_box', 10)
        # UART setup for sending error to low-level controller
        self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
        self.get_logger().info("UART initialized on /dev/ttyAMA0 at 115200 baud")

        # Frame dimensions
        self.FRAME_W, self.FRAME_H = 640, 480
        self.CENTER_X, self.CENTER_Y = self.FRAME_W // 2, self.FRAME_H // 2

        # Initialize the Camera
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(main={'format': 'RGB888', 'size': (self.FRAME_W, self.FRAME_H)})
        self.picam2.configure(config)
        self.picam2.start()
        self.get_logger().info("Camera started. Press 'q' to close.")

        # Green color range in HSV
        self.GREEN_LOWER = np.array([35, 100, 100])
        self.GREEN_UPPER = np.array([85, 255, 255])

        self.last_log_time = self.get_clock().now()
        self.LOG_INTERVAL = 0.5  # 500ms
        #Scan variables
        # In __init__
        self.scan_points = [(100,self.CENTER_Y),(self.CENTER_X , self.CENTER_Y),(540,self.CENTER_Y), (100,100), (540,100), (540,380), (100,380)]
        self.current_scan_index = 0
        self.scan_delay = 2.0
        self.last_scan_time = self.get_clock().now()
        
        
    def publish_url(self):
        msg = String()
        msg.data = '192.168.1.130:8889/cam/'
        self.pub.publish(msg)
    
    def publish_bbox(self, x, y, w, h):
        detection_msg = Detection2D()
        detection_msg.header.stamp = self.get_clock().now().to_msg()
        detection_msg.header.frame_id = "camera_frame"

        bbox = BoundingBox2D()
        bbox.center.position.x = float(x + w / 2)
        bbox.center.position.y = float(y + h / 2)
        bbox.size_x = float(w)
        bbox.size_y = float(h)

        detection_msg.bbox = bbox
        self.bbox_pub.publish(detection_msg)

    def main_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            frame = self.picam2.capture_array()
            frame = cv2.flip(frame, 0)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
            
            # Create and clean mask
            mask = cv2.inRange(hsv, self.GREEN_LOWER, self.GREEN_UPPER)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            error_x, error_y = 0, 0
            detected = False

            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 500:
                    x, y, w, h = cv2.boundingRect(largest)
                    current_time = self.get_clock().now()
                    elapsed = (current_time - self.last_log_time).nanoseconds / 1e9
                    if elapsed >= self.LOG_INTERVAL:
                        area = w * h
                        #self.get_logger().info(f"Bounding box area: {area} px²")
                        self.last_log_time = current_time

                    self.publish_bbox(x, y, w, h)

                    # Ball center
                    ball_x = x + w // 2
                    ball_y = y + h // 2
                    
                    # Calculate error from frame center
                    error_x = ball_x - self.CENTER_X
                    error_y = ball_y - self.CENTER_Y
                    detected = True
                    
                    # Draw bounding box and center point
                    cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame_bgr, (ball_x, ball_y), 5, (0, 0, 255), -1)

            if not detected:
                current_time = self.get_clock().now()
                elapsed = (current_time - self.last_scan_time).nanoseconds / 1e9
                
                # 1. Timer only handles SWITCHING targets
                if elapsed >= self.scan_delay:
                    self.current_scan_index = (self.current_scan_index + 1) % len(self.scan_points)
                    self.last_scan_time = current_time
                
                # 2. ALWAYS calculate error towards the current target (even if timer didn't tick)
                scan_x, scan_y = self.scan_points[self.current_scan_index]
                error_x = scan_x - self.CENTER_X
                error_y = scan_y - self.CENTER_Y
                
                # Visual debug for scanning
                cv2.circle(frame_bgr, (scan_x, scan_y), 10, (0, 255, 255), 2)
                 

            # Send data via UART: "error_x,error_y,detected\n"
            msg = f"{error_x},{error_y},{1}\n"
            self.ser.write(msg.encode())
            
            # Draw crosshair at frame center
            cv2.line(frame_bgr, (self.CENTER_X - 20, self.CENTER_Y), (self.CENTER_X + 20, self.CENTER_Y), (255, 0, 0), 2)
            cv2.line(frame_bgr, (self.CENTER_X, self.CENTER_Y - 20), (self.CENTER_X, self.CENTER_Y + 20), (255, 0, 0), 2)
            
            # Display TX info
            cv2.putText(frame_bgr, f"TX: {error_x},{error_y},{1 if detected else 0}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            cv2.imshow("Pi 5 Camera Preview", frame_rgb)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):       
                break

    def destroy_node(self):
        self.process.terminate()
        self.ser.close()
        self.picam2.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = None

    try:
        node = CameraPublisher()
        node.main_loop()

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