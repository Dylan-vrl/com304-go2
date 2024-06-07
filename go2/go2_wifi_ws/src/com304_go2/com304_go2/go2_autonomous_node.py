import math
from pathlib import Path

import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from com304_interfaces.msg import Move, Rotate

from .autonomous_agent.controller import Controller
from .rgb2depth.converter import RGBToDepthConverter

class Go2AutonomousNode(Node):
    def __init__(self):
        super().__init__('go2_autonomous_node')

        self.stop_publisher = self.create_publisher(Empty, '/stop', 10)
        self.move_publisher = self.create_publisher(Move, '/move', 10)
        self.rotate_publisher = self.create_publisher(Rotate, '/rotate', 10)

        self.cam_subscriber = self.create_subscription(Image, '/go2_camera/color/image', self.cam_callback, 10)
        self.goal_subscriber = self.create_subscription(Empty, '/goal_reached', self.goal_reached_callback, 10)
        self.start_subscriber = self.create_subscription(Empty, '/start', self.start_callback, 10)

        self.last_rgb = None
        self.bridge = CvBridge()

        cfg_path = Path(__file__).parent.parent.parent.parent.parent / 'share' / __package__ / 'config' / 'config_empty.yaml'
        ckpt_path = Path(__file__).parent.parent.parent.parent.parent / 'share' / __package__ / 'models' / 'model_empty.pth'

        # copied from config, ORDER MATTERS DO NOT EDIT
        self.actions = [self.stop, self.move_forward, self.turn_left, self.turn_right]
        self.action_count = 0

        self.model = Controller(cfg_path, ckpt_path)
        self.rgb2depth = RGBToDepthConverter(Path(__file__).parent.parent.parent.parent.parent / 'share' / __package__ / 'models' / 'mono+stereo_640x192')

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

    def cam_callback(self, msg: Image):
        self.last_rgb = msg

    def goal_reached_callback(self, msg: Empty):
        self.get_logger().info("Goal reached callback")
        self.execute_next_action()

    def start_callback(self, msg: Empty):
        self.get_logger().info("Starting autonomous navigation")
        self.execute_next_action()

    def execute_next_action(self):
        obs_space = self.model.obs_space

        # Collect rgb observations
        rgb_data = self.bridge.imgmsg_to_cv2(self.last_rgb, desired_encoding="rgb8")
        cv2.imwrite(f'last_high_rgb_{self.action_count}.jpg', cv2.cvtColor(rgb_data, cv2.COLOR_RGB2BGR))
        self.get_logger().info(str(rgb_data.shape))

        rgb_resize_shape = obs_space['rgb'].shape[:2][::-1]
        rgb_obs = cv2.resize(rgb_data, rgb_resize_shape)
        self.get_logger().info(str(rgb_obs.shape))
        cv2.imwrite(f'last_low_rgb_{self.action_count}.jpg', cv2.cvtColor(rgb_obs, cv2.COLOR_RGB2BGR))
        self.get_logger().info('Saved last frame rgb')

        # Collect depth observations
        depth_obs = self.rgb2depth.convert(rgb_obs)
        cv2.imwrite(f'last_low_depth_{self.action_count}.jpg', cv2.cvtColor(depth_obs * 255, cv2.COLOR_RGB2BGR))
        self.get_logger().info(str(np.max(depth_obs)))
        depth_obs = cv2.normalize(depth_obs, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        self.get_logger().info(str(np.max(depth_obs)))
        self.get_logger().info('Saved last frame depth')

        depth_obs = depth_obs[:, :, np.newaxis]

        observations = {
            "rgb": rgb_obs,
            "depth": depth_obs
        }
        self.action_count += 1
        next_action = self.model.act(observations)
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
