import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip', default=os.getenv('ROBOT_IP'))
    robot_token = LaunchConfiguration('robot_token', default=os.getenv('ROBOT_TOKEN',''))

    return LaunchDescription([
        Node(
            package='com304_go2',
            executable='go2_control_driver_node',
            parameters=[{'robot_ip': robot_ip, 'token': robot_token}],
            ),
        Node(
            package='ros2_go2_video',
            executable='ros2_go2_video',
            parameters=[{'robot_ip': robot_ip, 'robot_token': robot_token}],
            ),
        Node(
            package='com304_go2',
            executable='go2_control_node',
            ),
        Node(
            package='com304_go2',
            executable='go2_camera_node',
            ),
    ])