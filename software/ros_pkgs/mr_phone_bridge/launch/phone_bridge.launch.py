"""mr_phone_bridge 단독 launch. 베이스 스택(mr_bringup)과 별도로 띄운다.

    ros2 launch mr_phone_bridge phone_bridge.launch.py [params_file:=…]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(get_package_share_directory('mr_phone_bridge'), 'config', 'phone_bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_cfg),
        Node(package='mr_phone_bridge', executable='phone_bridge', name='mr_phone_bridge',
             parameters=[LaunchConfiguration('params_file')], output='screen', emulate_tty=True),
    ])
