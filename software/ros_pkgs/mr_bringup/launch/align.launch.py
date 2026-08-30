"""고정 마커 정렬 전체 스택.

정렬은 액션이라 goal을 보내야 움직인다.

    ros2 launch mr_bringup align.launch.py marker_id:=10
    ros2 action send_goal /align_to_marker mr_aruco_msgs/action/AlignToMarker \
        "{marker_id: 10}" --feedback
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from mr_bringup.launch_common import (
    build_align, build_aruco_tracker, build_base, build_camera,
    build_description, build_marker_track, build_twist_mux,
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
    nodes += build_align()
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('marker_id', default_value='10',
                              description='고정 정렬 타깃 id.'),
        DeclareLaunchArgument('video_device', default_value=''),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('base', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
