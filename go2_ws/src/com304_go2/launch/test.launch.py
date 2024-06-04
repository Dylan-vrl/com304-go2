from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='com304_go2',
            executable='go2_control_node',
        ),
        Node(
            package='com304_go2',
            executable='go2_camera_node',
        ),
        Node(
            package='com304_go2',
            executable='go2_depth_node',
        ),
    ])
