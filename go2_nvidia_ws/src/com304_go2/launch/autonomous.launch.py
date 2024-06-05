import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_ip = LaunchConfiguration('model_file', default=os.getenv('MODEL_FILE'))

    return LaunchDescription([
        Node(
            package='com304_go2',
            executable='go2_control_node',
            ),
        Node(
            package='com304_go2',
            executable='go2_autonomous_node',
            ),
    ])