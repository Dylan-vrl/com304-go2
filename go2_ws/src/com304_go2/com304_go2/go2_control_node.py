import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from go2_interfaces.msg import Go2State, IMU
from com304_interfaces.msg import Move, Rotate

from math import cos, sin

GOAL_EPSILON = { 
    'x': 0.08,
    'y': 0.08,
    'yaw': 0.08
} # Distance to be considered as reaching the goal
MAX_VEL = {
    'x': 0.4,
    'y': 0.4,
    'yaw': 0.6
}
MIN_VEL = {
    'x': 0.2,
    'y': 0.2,
    'yaw': 0.2
}

class Go2ControlNode(Node):
    def __init__(self):
        super().__init__('go2_control_node')

        self.pose = None
        self.goal = None
        self.last_goal = None

        self.delta_queue = [] # Expressed as deltas, e.g. next_pos = current_pos + delta
        self.move_msg = Twist()

        self.command_publisher = self.create_publisher(String, '/command', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.move_msg_timer = self.create_timer(0.05, self.move_msg_callback)
        self.cmd_vel_timer = self.create_timer(0.5, self.cmd_vel_callback)

        self.move_subscriber = self.create_subscription(Move, '/move', self.move, 10)
        self.rotate_subscriber = self.create_subscription(Rotate, '/rotate', self.rotate, 10)
        self.stop_subscriber = self.create_subscription(Empty, '/stop', self.stop_clear, 10)

        self.go2_state_subscriber = self.create_subscription(Go2State, '/go2_states', self.go2_state_callback, 10)
        self.imu_subscriber = self.create_subscription(IMU, '/imu', self.imu_callback, 10)

# ========== Actions ==========

    # Set the next goal to move to, assumes the robot is not moving
    def set_next_goal(self):
        if self.is_moving() or not self.delta_queue or self.pose is None:
            return
        
        delta_local = self.delta_queue.pop(0)
        pose = self.pose if self.last_goal is None else self.last_goal 

        delta_world = self.local_to_world(pose['yaw'], delta_local) # In robot local space (x forward, y left)


        goal = {}
        goal['x'] = delta_world['x'] + pose['x']
        goal['y'] = delta_world['y'] + pose['y']
        goal['yaw'] = self.normalize_yaw(pose['yaw'] + delta_world['yaw'])

        self.goal = goal
        self.last_goal = goal

        self.get_logger().info(f'Start moving from {pose} to {goal}')

    def move(self, msg: Move):
        delta_x = msg.x
        delta_y = msg.y

        self.delta_queue.append({'x': delta_x, 'y': delta_y, 'yaw': 0.0})
        if len(self.delta_queue) == 1:
            self.set_next_goal()
            return
    
    def rotate(self, msg: Rotate):
        delta_yaw = msg.yaw

        self.delta_queue.append({'x': 0.0, 'y': 0.0, 'yaw': delta_yaw})
        if len(self.delta_queue) == 1:
            self.set_next_goal()
            return

    # Stop moving and clear the delta queue
    def stop_clear(self, msh: Empty=None):
        self.delta_queue = []
        self.last_goal = None
        self.stop(force_stop=True)

    def stop(self, msg: Empty=None, force_stop: bool=False):
        if not force_stop and self.is_stopped():
            return
        self.goal = None
        self.command_publisher.publish(String(data='StopMove'))
       
# ========== Callbacks ==========

    def move_msg_callback(self):
        v = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        move_this_frame = False
        
        if self.is_moving():
            for axis in v.keys():           
                dist = self.dist_to_goal(axis)
                if dist < GOAL_EPSILON[axis]:
                    v[axis] = 0.0
                else:
                    v[axis] = MAX_VEL[axis] / (1.0 if dist > 1 else max(1 / dist, MIN_VEL[axis]))

                v[axis] *= self.dir_to_goal(axis)

                if self.goal_reached(axis):
                    v[axis] = 0.0
                else:
                    move_this_frame = True

        if not move_this_frame:
            if self.is_moving():
                self.get_logger().info(f'Goal reached: goal: {self.goal}, position: {self.pose}')
            self.stop()
            self.set_next_goal()
            return
        
        msg = Twist()
        msg.linear.x = v['x']
        msg.linear.y = v['y']
        msg.angular.z = v['yaw']
        self.move_msg = msg

    def cmd_vel_callback(self):
        if self.is_stopped():
            return
        self.cmd_vel_publisher.publish(self.move_msg)

    def go2_state_callback(self, msg: Go2State):
        pose = {}
        pose['x'] = msg.position[0]
        pose['y'] = msg.position[1]
        pose['yaw'] = 0.0 if self.pose is None else self.pose['yaw']
        self.pose = pose
        # self.get_logger().info(f'Updated pose: ({self.move_data["pose"]["x"]}, {self.move_data["pose"]["y"]})')

    def imu_callback(self, msg: IMU):
        pose = {}
        pose['x'] = 0.0 if self.pose is None else self.pose['x']
        pose['y'] = 0.0 if self.pose is None else self.pose['y']
        pose['yaw'] = msg.rpy[2]
        self.pose = pose

# ========== State ==========
    
    def is_stopped(self):
        return self.goal is None
    
    def is_moving(self):
        return not self.is_stopped()
    
    def dist_to_goal(self, axis: str) -> float:
        if not self.pose or not self.goal:
            return 0.0

        local_pose = self.world_to_local(self.pose['yaw'], self.pose)
        local_goal = self.world_to_local(self.pose['yaw'], self.goal)
        return abs(local_goal[axis] - local_pose[axis])

    def dir_to_goal(self, axis: str) -> int:
        if not self.pose or not self.goal:
            return 0

        pose_to_goal = {axis: self.goal[axis] - self.pose[axis] for axis in ['x', 'y', 'yaw']}
        local_pose_to_goal = self.world_to_local(self.pose['yaw'], pose_to_goal)
        goal = local_pose_to_goal[axis]
        if axis == 'yaw':
            goal = self.normalize_yaw(goal)
        return 1 if goal > 0 else -1

    def goal_reached(self, axis: str) -> bool:
        return self.dist_to_goal(axis) < GOAL_EPSILON[axis]
    
# ========== Utils ==========

    # Normalize yaw goal to be between -PI and PI
    def normalize_yaw(self, yaw: float) -> float:
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw < -math.pi:
            yaw += 2 * math.pi
        return yaw
    
    def local_to_world(self, orientation: float, local: dict) -> dict:
        x = local['x'] * cos(orientation) - local['y'] * sin(orientation)
        y = local['x'] * sin(orientation) + local['y'] * cos(orientation)
        yaw = local.get('yaw', orientation)
        return {'x': x, 'y': y, 'yaw': yaw}
    
    def world_to_local(self, orientation: float, world: dict) -> dict:
        x = world['x'] * cos(orientation) + world['y'] * sin(orientation)
        y = world['x'] * (-sin(orientation)) + world['y'] * cos(orientation)
        yaw = world.get('yaw', orientation)
        return {'x': x, 'y': y, 'yaw': yaw}

def main(args=None):
    rclpy.init(args=args)

    robot_control = Go2ControlNode()

    rclpy.spin(robot_control)

    robot_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()