"""mr_mission 단독 launch. follow 노드(mr_aruco)와 베이스 스택이 따로 떠 있어야 실제로 움직인다.

    ros2 launch mr_mission mission.launch.py [params_file:=…]

이 노드는 follow를 켜서 로봇을 움직인다. 실기에서는 사용자가 지켜볼 때만 띄운다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(get_package_share_directory('mr_mission'), 'config', 'mission.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_cfg),
        Node(package='mr_mission', executable='mission', name='mr_mission',
             parameters=[LaunchConfiguration('params_file')], output='screen', emulate_tty=True),
    ])
