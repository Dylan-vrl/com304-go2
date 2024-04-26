import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_ip = LaunchConfiguration('robot_ip', default=os.getenv('ROBOT_IP'))
    robot_token = LaunchConfiguration('robot_token', default=os.getenv('ROBOT_TOKEN',''))
    
    default_config_topics = os.path.join(get_package_share_directory('go2_robot_sdk'),
                                         'config', 'twist_mux.yaml')
    
    return LaunchDescription([

        DeclareLaunchArgument(
            'config_topics',
            default_value=default_config_topics,
            description='Default topics config file'),

        DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='cmd_vel',
            description='cmd vel output topic'),
        
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            parameters=[{'robot_ip': robot_ip, 'token': robot_token}],
            ),
        Node(
            package='go2_robot_sdk',
            executable='go2_proc_text',
            ),
    ])