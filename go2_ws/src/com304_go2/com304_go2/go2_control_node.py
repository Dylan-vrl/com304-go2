import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from go2_interfaces.msg import Go2State, IMU
from com304_interfaces.msg import COM304Move, COM304Rotate

MOVE_DELTA_TIME = 0.2
GOAL_EPSILON = 0.002 # Distance to be considered as reaching the goal
LOW_VEL = {
    'x': 0.2,
    'y': 0.2,
    'yaw': 0.4
}
HIGH_VEL = {
    'x': 0.4,
    'y': 0.4,
    'yaw': 0.8
}
LOW_VEL_THRESHOLD = {
    'x': 0.3,
    'y': 0.3,
    'yaw': math.pi / 4
}

class RobotControlNode(Node):

    def __init__(self):
        super().__init__('go2_control_node')

        self.move_data = {
            'pose': {'x': 0, 'y': 0, 'yaw': 0},
            'goal': {'x': 0, 'y': 0, 'yaw': 0},
            'shouldMove': {'x': False, 'y': False, 'yaw': False}
        }

        self.command_publisher = self.create_publisher(String, '/command', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_timer = self.create_timer(MOVE_DELTA_TIME, self.move_callback)

        self.move_subscriber = self.create_subscription(COM304Move, '/move', self.move, 10)
        self.rotate_subscriber = self.create_subscription(COM304Rotate, '/rotate', self.rotate, 10)
        self.stop_subscriber = self.create_subscription(Empty, '/stop', self.stop, 10)

        self.go2_state_subscriber = self.create_subscription(Go2State, '/go2_states', self.go2_state_callback, 10)
        self.imu_subscriber = self.create_subscription(IMU, '/imu', self.imu_callback, 10)

# ========== Actions ==========
    def move(self, msg: COM304Move):
        self.stop()

        delta_x = msg.x
        delta_y = msg.y

        self.move_data['goal']['x'] = self.move_data['pose']['x'] + delta_x
        self.move_data['goal']['y'] = self.move_data['pose']['y'] + delta_y
    
        self.move_data['shouldMove']['x'] = abs(delta_x) > 0
        self.move_data['shouldMove']['y'] = abs(delta_y) > 0

        self.get_logger().info(f'Start moving from ({self.move_data["pose"]["x"]}, {self.move_data["pose"]["y"]}) to ({self.move_data["goal"]["x"]}, {self.move_data["goal"]["y"]})')
    
    def rotate(self, msg: COM304Rotate):
        self.stop()

        delta_yaw = msg.yaw

        # Normalize goal to be between -PI and PI
        goal_yaw = self.move_data['pose']['yaw'] + delta_yaw
        while goal_yaw > math.pi:
            goal_yaw -= 2 * math.pi
        while goal_yaw < -math.pi:
            goal_yaw += 2 * math.pi

        self.move_data['goal']['yaw'] = goal_yaw
        self.move_data['shouldMove']['yaw'] = abs(delta_yaw) > 0

        self.getlogger().info(f'Start rotating from {self.move_data["pose"]["yaw"]} to {self.move_data["goal"]["yaw"]}')

    def stop(self, msg: Empty=None):
        for axis in self.move_data['shouldMove'].keys():
            self.stop_axis(axis)

    def stop_axis(self, axis: str):
        self.move_data['shouldMove'][axis] = False
        if any(self.move_data['shouldMove'].values()):
            return
        self.command_publisher.publish(String(data='StopMove'))

# ========== Callbacks ==========
    def move_callback(self):
        move = False
        v = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        for axis in v.keys():
            if (self.move_data['shouldMove'][axis]):
                move = True
            else:
                continue
                
            v[axis] = (LOW_VEL if self.dist_to_goal(axis) < LOW_VEL_THRESHOLD[axis] else HIGH_VEL)[axis]
            v[axis] *= self.dir_to_goal(axis)

            if self.goal_reached(axis):
                self.get_logger().info('Goal reached for axis %s' % axis)
                self.stop_axis(axis)
                v[axis] = 0.0

        if not move:
            return
        msg = Twist()
        msg.linear.x = v['x']
        msg.linear.y = v['y']
        msg.angular.z = v['yaw']
        self.get_logger().info(f'Moving with v: ({msg.linear.x}, {msg.linear.y}, {msg.angular.z})') 
        self.cmd_vel_publisher.publish(msg)

    def go2_state_callback(self, msg: Go2State):
        self.move_data['pose']['x'] = msg.position[0]
        self.move_data['pose']['y'] = msg.position[1]
        # self.get_logger().info(f'Updated pose: ({self.move_data["pose"]["x"]}, {self.move_data["pose"]["y"]})')

    def imu_callback(self, msg: IMU):
        self.move_data['pose']['yaw'] = msg.rpy[2]

# ========== Utils ==========
    def dist_to_goal(self, axis: str) -> float:
        current = self.move_data['pose'][axis]
        return abs(self.move_data['goal'][axis] - current)

    def dir_to_goal(self, axis: str) -> int:
        current = self.move_data['pose'][axis]
        return 1 if self.move_data['goal'][axis] - current > 0 else -1

    def goal_reached(self, axis: str) -> bool:
        self.get_logger().info(f'Distance to goal for axis {axis}: {self.dist_to_goal(axis)}')
        return self.dist_to_goal(axis) < GOAL_EPSILON

def main(args=None):
    rclpy.init(args=args)

    robot_control = RobotControlNode()

    rclpy.spin(robot_control)

    robot_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()