import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

from datetime import datetime
import cv2
from cv_bridge import CvBridge

class Go2CameraNode(Node):
    def __init__(self):
        super().__init__('go2_camera_node')
        
        self.save_subscriber = self.create_subscription(Empty, '/save_camera', self.save_callback, 10)
        self.cam_subscriber = self.create_subscription(Image, '/d435i/color/image_raw', self.cam_callback, 10)
        self.last_image = None
        self.bridge = CvBridge()

    def cam_callback(self, msg: Image):
        self.last_image = msg

    def save_callback(self, msg: Empty):
        file_name = f'{datetime.now().timestamp()}.jpg'
        self.get_logger().info(f'Start saving camera image to {file_name}...')
        if self.last_image is not None:
            cv_image = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding="bgr8")
            cv2.imwrite(file_name, cv_image)
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