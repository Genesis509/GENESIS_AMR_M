import cv2
from rclpy.node import Node
from std_msgs.msg import String
import rclpy
from rclpy.executors import MultiThreadedExecutor
import webbrowser

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.Opened = False
        
        self.sub = self.create_subscription(
            String, 'stream_info', self.on_stream_url, 10)

    def on_stream_url(self, msg):
        if not self.Opened:
            self.Opened = True
            url = msg.data
            webbrowser.open(url)
            self.get_logger().info(f'Opened stream: {url}')
      
def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = None

    try:
        node = CameraSubscriber()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f"Error: {e}")

    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()


