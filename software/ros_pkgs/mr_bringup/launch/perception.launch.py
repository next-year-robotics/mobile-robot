"""인지 계통 전체 — TF + 카메라 + 검출 + 추적 필터. 제어는 띄우지 않는다.

바퀴를 움직이지 않으므로 마커 검출을 안전하게 확인할 수 있다.

    ros2 launch mr_bringup perception.launch.py marker_id:=0
    ros2 topic echo /marker_track
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from mr_bringup.launch_common import (
    build_aruco_tracker, build_camera, build_description, build_marker_track,
)


def _setup(context, *args, **kwargs):
    del args, kwargs
    marker_id = int(LaunchConfiguration('marker_id').perform(context))
    video_device = LaunchConfiguration('video_device').perform(context)
    nodes = []
    if LaunchConfiguration('description').perform(context) == 'true':
        nodes += build_description()
    nodes += build_camera(video_device=video_device)
    nodes += build_aruco_tracker()
    nodes += build_marker_track(marker_id=marker_id)
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('marker_id', default_value='0',
                              description='추적할 마커 id. 작업자 0 / 고정 타깃 10.'),
        DeclareLaunchArgument('video_device', default_value=''),
        DeclareLaunchArgument('description', default_value='true',
                              description='robot_state_publisher를 같이 띄울지.'),
        OpaqueFunction(function=_setup),
    ])
