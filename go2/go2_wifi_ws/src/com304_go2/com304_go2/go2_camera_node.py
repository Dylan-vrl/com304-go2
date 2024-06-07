import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

from datetime import datetime
from PIL import Image as PILImage

class Go2CameraNode(Node):
    def __init__(self):
        super().__init__('go2_camera_node')
        
        self.save_subscriber = self.create_subscription(Empty, '/save_camera', self.save_callback, 10)
        self.cam_subscriber = self.create_subscription(Image, '/go2_camera/color/image', self.cam_callback, 10)
        self.last_image = None

    def cam_callback(self, msg: Image):
        self.last_image = msg

    def save_callback(self, msg: Empty):
        file_name = f'{datetime.now().timestamp()}.jpg'
        self.get_logger().info(f'Start saving camera image to {file_name}...')
        if self.last_image is not None:
            bytes_image = bytes(self.last_image.data)
            image = PILImage.frombytes('RGB', (self.last_image.width, self.last_image.height), bytes_image)
            image.save(file_name)
            self.get_logger().info(f'Saved camera image to {file_name}.')
        else:
            self.get_logger().info('No image received yet.')


def main(args=None):
    rclpy.init(args=args)

    cam_node = Go2CameraNode()

    rclpy.spin(cam_node)

    cam_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()