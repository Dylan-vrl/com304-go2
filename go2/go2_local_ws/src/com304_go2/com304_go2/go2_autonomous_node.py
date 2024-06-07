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
        self.action_count = 0

    def stop(self):
        msg = Empty()
        self.stop_publisher.publish(msg)
        self.get_logger().info(f"Action taken: Stop")

    def move_forward(self):
        msg = Move()
        msg.x = 0.5
        msg.y = 0.0
        self.move_publisher.publish(msg)
        self.get_logger().info(f"Action taken: Move forward")

    def turn_left(self):
        msg = Rotate()
        msg.yaw = math.pi/4
        self.rotate_publisher.publish(msg)
        self.get_logger().info(f"Action taken: Turn left")

    def turn_right(self):
        msg = Rotate()
        msg.yaw = -math.pi/4
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
        def remove_top(array):
            flat = array.flatten()
            threshold = np.percentile(flat, 95)
            mask = flat >= threshold
            flat[mask] = 0
            flat[mask] = np.max(flat)
            return flat.reshape(array.shape)
        
        obs_space = self.model.obs_space

        # Collect rgb observations
        rgb_data = self.bridge.imgmsg_to_cv2(self.last_rgb, desired_encoding="rgb8")
        cv2.imwrite(f'last_high_rgb_{self.action_count}.jpg', cv2.cvtColor(rgb_data, cv2.COLOR_RGB2BGR))

        rgb_resize_shape = obs_space['rgb'].shape[:2][::-1]
        rgb_obs = cv2.resize(rgb_data, rgb_resize_shape)
        cv2.imwrite(f'last_low_rgb_{self.action_count}.jpg', cv2.cvtColor(rgb_obs, cv2.COLOR_RGB2BGR))
        self.get_logger().info('Saved last frame rgb')

        # Collect depth observations
        depth_data = self.bridge.imgmsg_to_cv2(self.last_depth, desired_encoding="passthrough")
        self.get_logger().info(str(np.max(depth_data)))
        depth_data = remove_top(depth_data)
        depth_data = depth_data.astype('float32')
        depth_data = depth_data / 1000 / 10

        depth_img = (depth_data * 255).astype('uint8')
        cv2.imwrite(f'last_high_depth_{self.action_count}.jpg', cv2.cvtColor(depth_img, cv2.COLOR_RGB2BGR))

        depth_resize_shape = obs_space['depth'].shape[:2][::-1]
        depth_obs = cv2.resize(depth_data, depth_resize_shape)

        depth_img = (depth_obs * 255).astype('uint8')
        cv2.imwrite(f'last_low_depth_{self.action_count}.jpg', cv2.cvtColor(depth_img, cv2.COLOR_RGB2BGR))
        self.get_logger().info('Saved last frame depth')

        depth_obs = depth_obs[:, :, np.newaxis]

        observations = {
            "rgb": rgb_obs,
            "depth": depth_obs
        }

        next_action = self.model.act(observations)
        self.get_logger().info(f'Next action {next_action}')
        self.action_count += 1
        self.actions[next_action]()

def main(args=None):
    rclpy.init(args=args)

    node = Go2AutonomousNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
