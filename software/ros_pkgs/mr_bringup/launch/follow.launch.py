"""작업자 추종 전체 스택.

`follow`는 비활성 상태로 뜬다. 명시적으로 켜야 움직인다.

    ros2 launch mr_bringup follow.launch.py
    ros2 service call /follow/enable std_srvs/srv/SetBool "{data: true}"
    ros2 topic pub -1 /safety/lock std_msgs/msg/Bool "{data: true}"    # 비상 정지
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from mr_bringup.launch_common import (
    build_aruco_tracker, build_base, build_camera, build_description,
    build_follow, build_marker_track, build_twist_mux,
)


def _setup(context, *args, **kwargs):
    del args, kwargs
    nodes = build_description()
    if LaunchConfiguration('base').perform(context) == 'true':
        nodes += build_base(
            serial_device=LaunchConfiguration('serial_device').perform(context))
    nodes += build_camera(
        video_device=LaunchConfiguration('video_device').perform(context))
    nodes += build_aruco_tracker()
    nodes += build_marker_track(
        marker_id=int(LaunchConfiguration('marker_id').perform(context)))
    nodes += build_twist_mux()
    nodes += build_follow(enabled_on_start=False)
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('video_device', default_value=''),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument(
            'base', default_value='true',
            description='micro-ROS 에이전트와 오도메트리를 같이 띄울지. '
                        '노트북에서 인지만 볼 때는 false.'),
        OpaqueFunction(function=_setup),
    ])
