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
    build_follow, build_lidar, build_marker_track, build_teleop, build_twist_mux,
)


def _setup(context, *args, **kwargs):
    del args, kwargs
    nodes = build_description()
    if LaunchConfiguration('base').perform(context) == 'true':
        nodes += build_base(
            serial_device=LaunchConfiguration('serial_device').perform(context),
            ekf=LaunchConfiguration('ekf').perform(context) == 'true')
    if LaunchConfiguration('camera').perform(context) == 'true':
        nodes += build_camera(
            video_device=LaunchConfiguration('video_device').perform(context))
        nodes += build_aruco_tracker()
    if LaunchConfiguration('lidar').perform(context) == 'true':
        nodes += build_lidar(
            lidar_device=LaunchConfiguration('lidar_device').perform(context))
    nodes += build_marker_track(
        marker_id=int(LaunchConfiguration('marker_id').perform(context)))
    nodes += build_twist_mux()
    nodes += build_follow(enabled_on_start=False)
    if LaunchConfiguration('teleop').perform(context) == 'true':
        nodes += build_teleop()
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('video_device', default_value=''),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='카메라와 ArUco 검출을 같이 띄울지. 카메라가 노트북에 꽂혀 '
                        '/aruco_detections를 밖에서 받을 때는 false.'),
        DeclareLaunchArgument(
            'lidar', default_value='false',
            description='RPLIDAR C1(/scan)을 같이 띄울지. 추종 자체는 쓰지 않고, '
                        '부스 대시보드 표시와 심부름 이동의 장애물 정지가 쓴다.'),
        DeclareLaunchArgument(
            'lidar_device', default_value='',
            description='비우면 lidar.yaml의 /dev/rplidar(udev 규칙)를 쓴다.'),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument(
            'base', default_value='true',
            description='micro-ROS 에이전트와 오도메트리를 같이 띄울지. '
                        '노트북에서 인지만 볼 때는 false.'),
        DeclareLaunchArgument(
            'ekf', default_value='true',
            description='EKF(바퀴 + 자이로)가 odom → base_footprint를 낸다. '
                        'false면 예전처럼 바퀴만으로 낸다.'),
        DeclareLaunchArgument(
            'teleop', default_value='true',
            description='Xbox 패드 수동 개입. LB를 쥐면 자율 입력을 누른다.'),
        OpaqueFunction(function=_setup),
    ])
