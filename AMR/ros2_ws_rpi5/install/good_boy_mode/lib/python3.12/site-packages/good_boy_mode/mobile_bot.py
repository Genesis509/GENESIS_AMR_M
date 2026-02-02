import rclpy
import serial
from std_msgs.msg import String
from rclpy.node import Node 
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState  
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2
import cv2
from vision_msgs.msg import Detection2D, BoundingBox2D
from sensor_msgs.msg import JointState

# Distance goal: 100,000 px²
TARGET_AREA = 100000.0
KP = 0.000005  # Proportional gain - tune this based on testing
# Rotation control parameters
TARGET_PAN = 90.0       # Desired pan angle (centered)
PAN_DEADZONE = 5.0      # Ignore small errors (degrees)ca
KP_ROTATION = 0.005     # Proportional gain for rotation

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.serial_wheels = serial.Serial('/dev/ttyAMA3', 115200, timeout=1)
        self.get_logger().info("ESP32 Bridge Node has been started.")
        
        # bbox subscriber
        self.bbox_sub = self.create_subscription(
            Detection2D, 
            'camera/bounding_box', 
            self.PID_bbox_command, 
            10
        )
        self.servo_sub = self.create_subscription(
            JointState,
            'camera/servo_joints',
            self.servo_callback,
            10
        )
        self.vx = 0.0
        self.vy = 0.0
        self.rz = 0.0
        self.current_pan = 90.0  # Default to centered

    def servo_callback(self, msg: JointState):
        """Update current pan angle from gimbal."""
        if 'pan' in msg.name:
            pan_index = msg.name.index('pan')
            self.current_pan = msg.position[pan_index]
    
    def calculate_rotation(self):
        """Calculate rotation speed based on pan error."""
        pan_error = self.current_pan - TARGET_PAN
        
        # Apply deadzone
        if abs(pan_error) < PAN_DEADZONE:
            return 0.0
        
        # Proportional control
        # Positive pan_error (pan > 90) = target is right = rotate right (negative rz)
        # Negative pan_error (pan < 90) = target is left = rotate left (positive rz)
        rz = -KP_ROTATION * pan_error
        
        # Clamp between -1 and 1
        return max(-1.0, min(1.0, rz))
    
    def PID_bbox_command(self, bbox_msg):
        # Extract bbox dimensions
        bbox = bbox_msg.bbox
        width = bbox.size_x
        height = bbox.size_y
        
        # Calculate area
        area = width * height
        
        # Skip if no detection (area is 0)
        if area <= 0:
            self.vx = 0.0
            self.send_serial_command()
            return
        
        # Calculate error: positive = too far (need to go forward)
        #                  negative = too close (need to go back)
        error = TARGET_AREA - area
        
        # Proportional control
        self.vx = KP * error
        
        # Clamp between -1 and 1
        self.vx = max(-1.0, min(1.0, self.vx))
        
        # No lateral or rotational movement for now
        self.vy = 0.0
        self.rz = self.calculate_rotation()
            
        
        self.get_logger().info(f"Area: {area:.0f} px² | Error: {error:.0f} | vx: {self.vx:.2f}")
        self.send_serial_command()

    def send_command(self, cmd):
        try:
            self.vx = cmd.linear.x
            self.vy = cmd.linear.y
            self.rz = cmd.angular.z
            self.send_serial_command()
        except ValueError:
            self.get_logger().error("Invalid cmd_vel format. Expected 'vx,vy,rz'.")

    def send_serial_command(self):
        speed_vect_str = f"{self.vx:.2f},{self.vy:.2f},{self.rz:.2f}\n"
        self.serial_wheels.write(speed_vect_str.encode('utf-8'))
        # self.get_logger().info(f"Sent to ESP32: {speed_vect_str.strip()}")


def main(args=None):
    rclpy.init(args=args)
    esp32_bridge = ESP32Bridge()
    try:
        rclpy.spin(esp32_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        esp32_bridge.destroy_node()
        esp32_bridge.serial_wheels.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()