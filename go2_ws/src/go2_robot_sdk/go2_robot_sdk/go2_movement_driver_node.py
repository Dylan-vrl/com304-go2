import json
import logging
import os
import threading
import asyncio

from scripts.go2_constants import ROBOT_CMD, RTC_TOPIC
from scripts.webrtc_driver import Go2Connection
from scripts.go2_func import gen_command, gen_mov_command

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist
from go2_interfaces.msg import Go2State, IMU
from std_msgs.msg import String


logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RobotBaseNode(Node):

    def __init__(self):
        super().__init__('go2_movement_driver_node')

        self.declare_parameter('robot_ip', os.getenv('ROBOT_IP', os.getenv('GO2_IP')))
        self.declare_parameter('token', os.getenv('ROBOT_TOKEN', os.getenv('GO2_TOKEN','')))
        
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.token = self.get_parameter('token').get_parameter_value().string_value
        self.conn = None
        qos_profile = QoSProfile(depth=10)

        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.robot_cmd_vel = None
        self.robot_sport_state = None
        self.robot_command_queue = []
        
        self.go2_state_pub = self.create_publisher(Go2State, 'go2_states', qos_profile)
        self.imu_pub = self.create_publisher(IMU, 'imu', qos_profile)

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile)
        
        self.command_sub = self.create_subscription(
            String,
            'command',
            self.command_callback,
            qos_profile)
        
        self.robot_state_timer = self.create_timer(0.05, self.robot_state_callback)

    def robot_state_callback(self):
        self.publish_robot_state()

    def cmd_vel_callback(self, msg: Twist):
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z
        #if x > 0.0 or y > 0.0 or z > 0.0: disable backwards movement
        self.robot_cmd_vel = gen_mov_command(x, y, z)

    def command_callback(self, msg):
        self.get_logger().info(f"Received command: {msg.data}:{ROBOT_CMD[msg.data]}")
        self.robot_command_queue.append(msg)

    def handle_cmd(self):            
        if self.robot_cmd_vel:
            self.conn.data_channel.send(self.robot_cmd_vel)
            self.robot_cmd_vel = None

        if len(self.robot_command_queue) > 0:
            msg = self.robot_command_queue[0]
            robot_cmd = gen_command(ROBOT_CMD[msg.data])
            self._logger.info(f"Sending command: {msg.data}:{ROBOT_CMD[msg.data]}")
            self.conn.data_channel.send(robot_cmd)
            self.robot_command_queue.pop(0)

    def on_validated(self):
        for topic in RTC_TOPIC.values():
            self.conn.data_channel.send(json.dumps({"type": "subscribe", "topic": topic}))
        
    def on_data_channel_message(self, _, msg):
        if msg.get('topic') == RTC_TOPIC['LF_SPORT_MOD_STATE']:
            self.robot_sport_state = msg

    def publish_robot_state(self):
        if self.robot_sport_state:
            go2_state = Go2State()
            go2_state.mode = self.robot_sport_state["data"]["mode"]
            go2_state.progress = self.robot_sport_state["data"]["progress"]
            go2_state.gait_type = self.robot_sport_state["data"]["gait_type"]
            go2_state.position = list(map(float,self.robot_sport_state["data"]["position"]))
            go2_state.body_height = float(self.robot_sport_state["data"]["body_height"])
            go2_state.velocity = self.robot_sport_state["data"]["velocity"]
            go2_state.range_obstacle = list(map(float, self.robot_sport_state["data"]["range_obstacle"]))
            go2_state.foot_force = self.robot_sport_state["data"]["foot_force"]
            go2_state.foot_position_body = list(map(float,self.robot_sport_state["data"]["foot_position_body"]))
            go2_state.foot_speed_body = list(map(float, self.robot_sport_state["data"]["foot_speed_body"]))
            self.go2_state_pub.publish(go2_state) 

            imu = IMU()
            imu.quaternion = list(map(float,self.robot_sport_state["data"]["imu_state"]["quaternion"]))
            imu.accelerometer = list(map(float,self.robot_sport_state["data"]["imu_state"]["accelerometer"]))
            imu.gyroscope = list(map(float,self.robot_sport_state["data"]["imu_state"]["gyroscope"]))
            imu.rpy = list(map(float,self.robot_sport_state["data"]["imu_state"]["rpy"]))
            imu.temperature = self.robot_sport_state["data"]["imu_state"]["temperature"]
            self.imu_pub.publish(imu) 

    async def run(self, conn):
        self.conn = conn
        await self.conn.connect()
        self.get_logger().info(f"Connected to {os.environ.get('ROBOT_IP')}")

        while True:
            self.handle_cmd()
            await asyncio.sleep(0.1)


async def spin(node: Node):
    cancel = node.create_guard_condition(lambda: None)
    def _spin(node: Node,
              future: asyncio.Future,
              event_loop: asyncio.AbstractEventLoop):
        while not future.cancelled():
            rclpy.spin_once(node)
        if not future.cancelled():
            event_loop.call_soon_threadsafe(future.set_result, None)
    event_loop = asyncio.get_event_loop()
    spin_task = event_loop.create_future()
    spin_thread = threading.Thread(target=_spin, args=(node, spin_task, event_loop))
    spin_thread.start()
    try:
        await spin_task
    except asyncio.CancelledError:
        cancel.trigger()
    spin_thread.join()
    node.destroy_guard_condition(cancel)


async def start_node():
    base_node = RobotBaseNode()
    conn = Go2Connection(
        robot_ip=base_node.robot_ip,
        token=base_node.token,
        on_validated=base_node.on_validated,
        on_message=base_node.on_data_channel_message,

    )
    spin_task = asyncio.get_event_loop().create_task(spin(base_node))
    sleep_task = asyncio.get_event_loop().create_task(base_node.run(conn))
    await asyncio.wait([spin_task, sleep_task], return_when=asyncio.FIRST_COMPLETED)

def main():
    rclpy.init()
    asyncio.get_event_loop().run_until_complete(start_node())
    asyncio.get_event_loop().close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
