#!/usr/bin/env python3
"""
ROS2 Camera Handler for God boy mode
Computes pan/tilt servo commands locally and sends via UART
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from picamera2 import Picamera2
import cv2
import numpy as np
import serial
import subprocess
import time
from vision_msgs.msg import Detection2D, BoundingBox2D
from sensor_msgs.msg import JointState


class PanTiltController:
    """Converts tracking errors to servo angles using proportional control."""
    
    def __init__(self):
        # State variables (persisted between calls)
        self.pan_angle = 90.0
        self.tilt_angle = 90.0
        self.last_signal_time = time.time()
        
        # Tuning Parameters
        self.DEADZONE = 30       # Ignore errors smaller than this
        self.GAIN = 0.01         # Speed of following
        self.ANGLE_MIN = 20
        self.ANGLE_MAX = 160
        self.TIMEOUT_SEC = 3.0   # Auto-center after 3 seconds of no detection

    def _constrain(self, val, min_val, max_val):
        return max(min_val, min(val, max_val))

    def update(self, error_x, error_y, detected):
        """
        Calculates new servo angles based on error inputs.
        
        Args:
            error_x (int): Horizontal error
            error_y (int): Vertical error
            detected (bool/int): 1 if object is seen, 0 if not
            
        Returns:
            tuple: (int pan_angle, int tilt_angle)
        """
        current_time = time.time()
        
        #if detected:
        self.last_signal_time = current_time
        
        # Proportional Control: Pan (X Axis)
        if abs(error_x) > self.DEADZONE:
            self.pan_angle += error_x * self.GAIN
            
        # Proportional Control: Tilt (Y Axis)
        if abs(error_y) > self.DEADZONE:
            self.tilt_angle -= error_y * self.GAIN
            
        # Constrain to physical limits
        self.pan_angle = self._constrain(self.pan_angle, self.ANGLE_MIN, self.ANGLE_MAX)
        self.tilt_angle = self._constrain(self.tilt_angle, self.ANGLE_MIN, self.ANGLE_MAX)
        
        # Timeout Logic: Auto-center if object lost for too long
        #if (current_time - self.last_signal_time) > self.TIMEOUT_SEC:
        #    self.pan_angle = 90.0
        #    self.tilt_angle = 90.0
            
        return int(self.pan_angle), int(self.tilt_angle)


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
        
        # Bbox publisher
        self.bbox_pub = self.create_publisher(Detection2D, 'camera/bounding_box', 10)
        
        # Joint state publisher for pan/tilt
        self.servo_pub = self.create_publisher(JointState, 'camera/servo_joints', 10)
        
        # UART setup for sending servo commands to low-level controller
        self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
        self.get_logger().info("UART initialized on /dev/ttyAMA0 at 115200 baud")

        # Frame dimensions
        self.FRAME_W, self.FRAME_H = 640, 480
        self.CENTER_X, self.CENTER_Y = self.FRAME_W // 2, self.FRAME_H // 2

        # Initialize the Camera
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={'format': 'RGB888', 'size': (self.FRAME_W, self.FRAME_H)}
        )
        self.picam2.configure(config)
        self.picam2.start()
        self.get_logger().info("Camera started. Press 'q' to close.")

        # Green color range in HSV
        self.GREEN_LOWER = np.array([35, 100, 100])
        self.GREEN_UPPER = np.array([85, 255, 255])

        self.last_log_time = self.get_clock().now()
        self.LOG_INTERVAL = 0.5  # 500ms
        
        # Scan variables
        self.scan_points = [
            (100, self.CENTER_Y),
            (self.CENTER_X, self.CENTER_Y),
            (540, self.CENTER_Y),
            (540, 380),
            (100, 380),
            (100, 100),
            (540, 100)
        ]
        self.current_scan_index = 0
        self.scan_delay = 1.0
        self.last_scan_time = self.get_clock().now()
        
        # Initialize pan/tilt controller
        self.gimbal = PanTiltController()
        self.pan_pos = 90
        self.tilt_pos = 90

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
    
    def publish_servo_angles(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_gimbal"
        msg.name = ['pan', 'tilt']
        msg.position = [float(self.pan_pos), float(self.tilt_pos)]  # degrees
        msg.velocity = []
        msg.effort = []
        self.servo_pub.publish(msg)

    def send_pantilt_command(self):
        """Send pan/tilt angles via UART. Format: 'pan,tilt\n'"""
        pantilt_str = f"{int(self.pan_pos)},{int(self.tilt_pos)}\n"
        self.ser.write(pantilt_str.encode('utf-8'))

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
                
                # Timer handles switching targets
                if elapsed >= self.scan_delay:
                    self.current_scan_index = (self.current_scan_index + 1) % len(self.scan_points)
                    self.last_scan_time = current_time
                
                # Calculate error towards current scan target
                scan_x, scan_y = self.scan_points[self.current_scan_index]
                error_x = scan_x - self.CENTER_X
                error_y = scan_y - self.CENTER_Y
                
                # Visual debug for scanning
                cv2.circle(frame_bgr, (scan_x, scan_y), 10, (0, 255, 255), 2)

            # Update gimbal controller and get servo angles
            self.pan_pos, self.tilt_pos = self.gimbal.update(error_x, error_y, detected)
            
            # Send servo commands via UART
            self.send_pantilt_command()
            # Publish joint states
            self.publish_servo_angles()
            
            # Draw crosshair at frame center
            cv2.line(frame_bgr, (self.CENTER_X - 20, self.CENTER_Y), 
                     (self.CENTER_X + 20, self.CENTER_Y), (255, 0, 0), 2)
            cv2.line(frame_bgr, (self.CENTER_X, self.CENTER_Y - 20), 
                     (self.CENTER_X, self.CENTER_Y + 20), (255, 0, 0), 2)
            
            # Display TX info (now shows servo angles)
            cv2.putText(frame_bgr, f"Pan: {self.pan_pos} Tilt: {self.tilt_pos}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame_bgr, f"Error: ({error_x}, {error_y}) Det: {detected}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
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