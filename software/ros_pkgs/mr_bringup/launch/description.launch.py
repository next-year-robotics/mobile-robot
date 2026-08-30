"""robot_state_publisher만 띄운다. TF 체인 확인용.

base_footprint 아래로 카메라 프레임까지 붙는지 본다.

    ros2 launch mr_bringup description.launch.py
    ros2 run tf2_tools view_frames
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from mr_bringup.launch_common import build_description


def _setup(context, *args, **kwargs):
    del args, kwargs
    sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    return build_description(use_sim_time=sim)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
