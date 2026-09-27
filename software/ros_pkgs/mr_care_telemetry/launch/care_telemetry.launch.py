"""mr_care_telemetry 단독 launch. 브리지·베이스와 별도 프로세스로 띄운다.

    ros2 launch mr_care_telemetry care_telemetry.launch.py [params_file:=…]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(get_package_share_directory('mr_care_telemetry'), 'config', 'care_telemetry.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_cfg),
        Node(package='mr_care_telemetry', executable='care_telemetry', name='mr_care_telemetry',
             parameters=[LaunchConfiguration('params_file')], output='screen', emulate_tty=True),
    ])
