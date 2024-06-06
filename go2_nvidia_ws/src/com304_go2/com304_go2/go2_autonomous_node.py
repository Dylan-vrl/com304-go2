import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from com304_interfaces.msg import Move, Rotate

from pathlib import Path
from .habitat_nn import HabitatController
from .habitat_utils.ppo_agents import PPOAgentConfig

from PIL import Image as PILImage
import os
import numpy as np
import cv2
from cv_bridge import CvBridge

class Go2AutonomousNode(Node):
    def __init__(self):
        super().__init__('go2_autonomous_node')

        self.stop_publisher = self.create_publisher(Empty, '/stop', 10)
        self.move_publisher = self.create_publisher(Move, '/move', 10)
        self.rotate_publisher = self.create_publisher(Rotate, '/rotate', 10)

        self.rgb_subscriber = self.create_subscription(Image, '/d435i/color/image_raw', self.rgb_callback, 10)
        self.depth_subscriber = self.create_subscription(Image, '/d435i/depth/image_rect_raw', self.depth_callback, 10)
        self.goal_subscriber = self.create_subscription(Empty, '/goal_reached', self.goal_reached_callback, 10)
        self.start_subscriber = self.create_subscription(Empty, '/start', self.start_callback, 10)

        self.last_rgb = None
        self.last_depth = None
        self.bridge = CvBridge()

        cfg_path = Path(__file__).parent.parent.parent.parent.parent / 'share' / __package__ / 'config' / 'config.yaml'
        ckpt_path = Path(__file__).parent.parent.parent.parent.parent / 'share' / __package__ / 'models' / 'model.pth'

        # copied from config, ORDER MATTERS DO NOT EDIT
        self.actions = [self.stop, self.move_forward, self.turn_left, self.turn_right]
        self.model = HabitatController(cfg_path, ckpt_path)
        self.model.reset()        

    def stop(self):
        msg = Empty()
        self.stop_publisher.publish(msg)
        self.get_logger().info(f"Action taken: Stop")

    def move_forward(self):
        msg = Move()
        msg.x = 0.3
        msg.y = 0.0
        self.move_publisher.publish(msg)
        self.get_logger().info(f"Action taken: Move forward")

    def turn_left(self):
        msg = Rotate()
        msg.yaw = math.pi/6
        self.rotate_publisher.publish(msg)
        self.get_logger().info(f"Action taken: Turn left")

    def turn_right(self):
        msg = Rotate()
        msg.yaw = -math.pi/6
        self.rotate_publisher.publish(msg)
        self.get_logger().info(f"Action taken: Turn right")

    def rgb_callback(self, msg: Image):
        self.last_rgb = msg

    def depth_callback(self, msg: Image):
        self.last_depth = msg

    def goal_reached_callback(self, msg: Empty):
        self.get_logger().info("Goal reached callback")
        self.execute_next_action()

    def start_callback(self, msg: Empty):
        self.get_logger().info("Starting autonomous navigation")
        self.execute_next_action()

    def execute_next_action(self):
        # Collect rgb observations
        rgb_data = self.bridge.imgmsg_to_cv2(self.last_rgb, desired_encoding="rgb8")
        im = PILImage.fromarray(rgb_data)
        im.save('last_rgb.jpg')
        self.get_logger().info('Saved last frame rgb')

        # Collect depth observations
        depth_data = self.bridge.imgmsg_to_cv2(self.last_depth, desired_encoding="passthrough")

        depth_img = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        im = PILImage.fromarray(depth_img)
        im.save('last_depth.jpg')
        self.get_logger().info('Saved last frame depth')

        depth_data = depth_data.astype('float32')
        depth_data = cv2.normalize(depth_data, None, 0, 1, cv2.NORM_MINMAX)
        depth_data = depth_data[:, :, np.newaxis]

        observations = {
            "rgb": rgb_data,
            "depth": depth_data
        }

        next_action = self.model.act(observations)['action']
        self.get_logger().info(f'Next action {next_action}')
        self.actions[next_action]()

def main(args=None):
    rclpy.init(args=args)

    node = Go2AutonomousNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
