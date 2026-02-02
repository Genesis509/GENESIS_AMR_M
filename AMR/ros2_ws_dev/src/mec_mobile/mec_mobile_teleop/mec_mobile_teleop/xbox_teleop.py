#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header
<<<<<<< HEAD
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
=======
>>>>>>> f50b9f6b3049fb19bd7fa335d9d4bce0bb0e238f

class XboxTeleop(Node):
    def __init__(self):
        super().__init__('xbox_teleop')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscriber for joy messages
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # Movement parameters
        self.raw_joy_data = Twist() 
        self.speed_factor_x = 2.2
        self.speed_factor_y = 1.5
        self.rotation_factor = 0.6
        self.right_trigger_raw_data = 1.0  # Rest position
        self.left_trigger_raw_data = 1.0   # Rest position
<<<<<<< HEAD

        # Pan/tilt parameters
        self.raw_pan_joy_data = 0.0
        self.raw_tilt_joy_data = 0.0
        self.pan_pos = 90.0   # Start at center
        self.tilt_pos = 90.0  # Start at center
        self.pan_speed = 1.5  # Degrees per callback 
        self.tilt_speed = 1.0
        self.ANGLE_MAX = 180.0  
        self.ANGLE_MIN = 0.0

        self.get_logger().info('Xbox Teleop Node Started')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)
=======

        # Pan/tilt parameters
        self.raw_pan_joy_data = 0.0
        self.raw_tilt_joy_data = 0.0
        self.pan_pos = 90.0   # Start at center
        self.tilt_pos = 90.0  # Start at center
        self.pan_speed = 1.5  # Degrees per callback 
        self.tilt_speed = 1.0
        self.ANGLE_MAX = 180.0  
        self.ANGLE_MIN = 0.0

        self.get_logger().info('Xbox Teleop Node Starteeeed')
>>>>>>> f50b9f6b3049fb19bd7fa335d9d4bce0bb0e238f
    
    def joy_callback(self, msg):
        # Left stick - movement
        self.raw_joy_data.linear.x = msg.axes[1]
        self.raw_joy_data.linear.y = msg.axes[0]
        
        # Triggers - rotation
        self.right_trigger_raw_data = msg.axes[5]
        self.left_trigger_raw_data = msg.axes[2]

        # Right stick - pan/tilt
        self.raw_pan_joy_data = msg.axes[3]
        self.raw_tilt_joy_data = msg.axes[4]

        self.publish_cmd_vel()
        self.handle_pan_tilt()

    def add_deadzone(self, value, deadzone=0.1):
        if abs(value) < deadzone:
            return 0.0
        return value

    def clamp(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))

    def publish_cmd_vel(self):
        processed_twist = Twist()
        
        # Apply deadzone and scaling
        processed_twist.linear.x = self.add_deadzone(self.raw_joy_data.linear.x) #* self.speed_factor_x
        processed_twist.linear.y = self.add_deadzone(self.raw_joy_data.linear.y) #* self.speed_factor_y
        processed_twist.angular.z = self.process_trigger_rotation()
        
        self.cmd_vel_pub.publish(processed_twist)

    def process_trigger_rotation(self):
        rt = (1.0 - self.right_trigger_raw_data) / 2.0
        lt = (1.0 - self.left_trigger_raw_data) / 2.0
        deadzone = 0.05
        
        if rt > deadzone:
            return rt #* self.rotation_factor
        elif lt > deadzone:
            return -lt #* self.rotation_factor
        return 0.0

    def handle_pan_tilt(self):
        # Apply deadzone to right stick
        pan_input = self.add_deadzone(self.raw_pan_joy_data)
        tilt_input = self.add_deadzone(self.raw_tilt_joy_data)
        
        # Update positions with clamping
        self.pan_pos += pan_input * self.pan_speed
        self.tilt_pos += tilt_input * self.tilt_speed
        self.pan_pos = self.clamp(self.pan_pos, self.ANGLE_MIN, self.ANGLE_MAX)
        self.tilt_pos = self.clamp(self.tilt_pos, self.ANGLE_MIN, self.ANGLE_MAX)

        # Publish JointState with header
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['pan_joint', 'tilt_joint']
        msg.position = [self.pan_pos, self.tilt_pos]
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    xbox_teleop = XboxTeleop()
    
    try:
        rclpy.spin(xbox_teleop)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        xbox_teleop.cmd_vel_pub.publish(stop_msg)
        xbox_teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()