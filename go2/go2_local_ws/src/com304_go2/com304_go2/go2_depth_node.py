import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

import numpy as np
import cv2
from cv_bridge import CvBridge
from datetime import datetime


class Go2DepthNode(Node):
    def __init__(self):
        super().__init__('go2_depth_node')

        self.save_subscriber = self.create_subscription(Empty, '/save_depth', self.save_callback, 10)
        self.depth_subscriber = self.create_subscription(Image, '/d435i/depth/image_rect_raw', self.depth_callback, 10)
        self.last_image = None
        self.bridge = CvBridge()

    def depth_callback(self, msg: Image):
        self.last_image = msg

    def save_callback(self, msg: Empty):
        file_name = f'depth-{datetime.now().timestamp()}.jpg'
        self.get_logger().info(f'Start saving depth image to {file_name}...')
        if self.last_image is not None:
            cv_image = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding="passthrough")
            cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
            cv_image = np.uint8(cv_image)
            cv2.imwrite(file_name, cv_image)
            self.get_logger().info(f'Saved depth image to {file_name}.')
        else:
            self.get_logger().info('No image received yet.')


def main(args=None):
    rclpy.init(args=args)

    depth_node = Go2DepthNode()

    rclpy.spin(depth_node)

    depth_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
