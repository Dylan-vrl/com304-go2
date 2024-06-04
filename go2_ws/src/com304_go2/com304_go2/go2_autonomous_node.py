import math
import PIL.Image as pil

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from com304_interfaces.msg import Move, Rotate
import numpy as np

from pathlib import Path
from .custom_agent import CustomAgent
from .habitat_utils.ppo_agents import PPOAgentConfig

from math import cos, sin

class Go2AutonomousNode(Node):
    def __init__(self):
        super().__init__('go2_autonomous_node')

        self.stop_publisher = self.create_publisher(Empty, '/stop', 10)
        self.move_publisher = self.create_publisher(Move, '/move', 10)
        self.rotate_publisher = self.create_publisher(Rotate, '/rotate', 10)

        self.cam_subscriber = self.create_subscription(Image, '/go2/sensor/camera', self.cam_callback, 10)
        self.depth_subscriber = self.create_subscription(Image, '/d435i/depth/image_rect_raw', self.depth_callback, 10)
        self.goal_subscriber = self.create_subscription(Empty, '/goal_reached', self.goal_reached_callback, 10)
        self.start_subscriber = self.create_subscription(Empty, '/start', self.start_callback, 10)

        self.last_image_rgb = None
        self.last_image_depth = None
        model_path = Path(__file__).parent.parent.parent.parent.parent / 'share' / __package__ / 'models' / 'model_only.pth'
        agent_config = PPOAgentConfig()
        agent_config.INPUT_TYPE = "rgbd"
        agent_config.MODEL_PATH = model_path
        agent_config.GOAL_SENSOR_UUID = "pointgoal"
        agent_config.RESOLUTION = 128

        # copied from config, ORDER MATTERS DO NOT EDIT
        self.actions = [self.stop, self.move_forward, self.turn_left, self.turn_right]
        self.model = CustomAgent(agent_config, Path(__file__).parent.parent.parent.parent.parent / 'share' / __package__ / 'models' / 'mono+stereo_640x192')
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

    def cam_callback(self, msg: Image):
        self.last_image_rgb = msg

    def depth_callback(self, msg: Image):
        self.last_image_depth = msg

    def goal_reached_callback(self, msg: Empty):
        self.get_logger().info("Goal reached callback")
        self.execute_next_action()

    def start_callback(self, msg: Empty):
        self.get_logger().info("Starting autonomous navigation")
        self.execute_next_action()

    def execute_next_action(self):
        data_rgb = np.frombuffer(self.last_image_rgb.data, dtype='uint8')
        data_2d_rgb = np.reshape(data_rgb, (self.last_image_rgb.height, self.last_image_rgb.step))
        image_array_rgb = data_2d_rgb.reshape((self.last_image_rgb.height, self.last_image_rgb.width, 3))

        # data_depth = np.frombuffer(self.last_image_depth.data, dtype='float32') / 255.0
        data_depth = np.frombuffer(self.last_image_depth.data, dtype='uint8')
        data_2d_depth = np.reshape(data_depth, (self.last_image_depth.height, self.last_image_depth.step))
        image_array_depth = data_2d_depth.reshape((self.last_iamge_depth.height, self.last_image_depth.width, 3))
        image_depth = pil.fromarray(image_array_depth).convert("L")  # "L" = Grayscale

        observations = {
            "rgb": np.array(image_array_rgb, dtype='uint8'),
            "depth": np.array(image_depth, dtype='float32')
        }
        self.get_logger().info(f'Before act')
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
