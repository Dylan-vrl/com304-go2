import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

from datetime import datetime
from PIL import Image as PILImage

class Go2DepthNode(Node):
    def __init__(self):
        super().__init__('go2_depth_node')

        self.save_subscriber = self.create_subscription(Empty, '/save_depth', self.save_callback, 10)
        self.depth_subscriber = self.create_subscription(Image, '/d435i/depth/image_rect_raw', self.depth_callback, 10)
        self.last_image = None

    def depth_callback(self, msg: Image):
        self.last_image = msg

    def save_callback(self, msg: Empty):
        file_name = f'depth-{datetime.now().timestamp()}.jpg'
        self.get_logger().info(f'Start saving depth image to {file_name}...')
        if self.last_image is not None:
            bytes_image = bytes(self.last_image.data)
            image = PILImage.frombytes('RGB', (self.last_image.width, self.last_image.height), bytes_image)
            image.save(file_name)
            self.get_logger().info(f'Saved depth image to {file_name}.')
        else:
            self.get_logger().info('No image received yet.')


def main(args=None):
    rclpy.init(args=args)

    cam_node = Go2DepthNode()

    rclpy.spin(cam_node)

    cam_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
