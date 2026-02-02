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

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.serial_wheels = serial.Serial('/dev/ttyAMA3', 115200)
        self.serial_pantilt = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.get_logger().info("ESP32 Bridge Node has been started.")

        self.vx = 0.0
        self.vy = 0.0
        self.rz = 0.0

        self.pan_pos = 90.0
        self.tilt_pos = 90.0
    
    def joint_callback(self, msg):
        try:
            # Extract pan and tilt from JointState
            if 'pan_joint' in msg.name and 'tilt_joint' in msg.name:
                pan_idx = msg.name.index('pan_joint')
                tilt_idx = msg.name.index('tilt_joint')
                self.pan_pos = msg.position[pan_idx]
                self.tilt_pos = msg.position[tilt_idx]
                self.send_pantilt_command()
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Joint state error: {e}")
    
    
    def h(self, msg):
        try:
            self.vx = msg.linear.x
            self.vy = msg.linear.y
            self.rz = msg.angular.z
            
            self.send_serial_command()
        except ValueError:
            self.get_logger().error("Invalid cmd_vel format. Expected 'vx,vy,rz'.")

    def send_pantilt_command(self):
        # Format: "pan,tilt\n" (degrees as integers)
        pantilt_str = f"{int(self.pan_pos)},{int(self.tilt_pos)}\n"
        self.serial_pantilt.write(pantilt_str.encode('utf-8'))
        #self.get_logger().info(f"Sent to Pan/Tilt ESP32: {pantilt_str.strip()}")

    def send_serial_command(self):
       
        speed_vect_str = f"{self.vx:.2f},{self.vy:.2f},{self.rz:.2f}\n"
        self.serial_wheels.write(speed_vect_str.encode('utf-8'))
        #self.get_logger().info(f"Sent to ESP32: {speed_vect_str.strip()}")

   
        
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
        esp32_bridge.serial_pantilt.close()
        rclpy.shutdown()
        esp32_bridge.serial_wheels.close()
