"""라이다 지도 작성 — 베이스(EKF) + RPLIDAR C1 + slam_toolbox + Xbox 패드.

카메라와 추종은 띄우지 않는다. 부스 스택이 떠 있으면 시리얼·패드·twist_mux가 겹치니
먼저 내린다(부스 리포 `scripts/robot.sh down`과 같다).

    sudo systemctl stop mr-booth.target
    ros2 launch mr_bringup slam.launch.py
    # LB를 쥐고 천천히 돈다. 출발한 자리로 돌아와 루프를 닫는다.
    mkdir -p ~/maps
    # .pgm + .yaml, Nav2 map_server용. slam_toolbox는 /map을 transient local로 5 s마다 내므로
    # 그 QoS로 받아야 한다. 기본값으로는 2 s 안에 못 받아 "Failed to spin map subscription"이 난다.
    ros2 run nav2_map_server map_saver_cli -f ~/maps/<이름> \
        --ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
        "{filename: '/home/emjdpi/maps/<이름>'}"                  # 나중에 이어 그리기용

노트북 RViz: Fixed Frame `map`, LaserScan `/scan`(QoS Best Effort도 된다), Map `/map`.

처음 장착한 뒤에는 `slam:=false`로 띄워 방향부터 본다. Fixed Frame을 `base_link`로 두고
로봇 앞에 상자를 놓았을 때 +x에 찍혀야 한다. 그다음 Fixed Frame `odom`, Decay Time 10 s로
제자리 한 바퀴를 돌려 벽이 한 겹으로 겹치는지 본다. 번지면 odom 회전(EKF)이 틀린 것이다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from mr_bringup.launch_common import (
    build_base, build_description, build_lidar, build_slam, build_teleop,
    build_twist_mux,
)


def _setup(context, *args, **kwargs):
    del args, kwargs
    nodes = build_description()
    if LaunchConfiguration('base').perform(context) == 'true':
        nodes += build_base(
            serial_device=LaunchConfiguration('serial_device').perform(context),
            ekf=LaunchConfiguration('ekf').perform(context) == 'true')
    nodes += build_lidar(
        lidar_device=LaunchConfiguration('lidar_device').perform(context))
    if LaunchConfiguration('slam').perform(context) == 'true':
        nodes += build_slam()
    if LaunchConfiguration('teleop').perform(context) == 'true':
        nodes += build_twist_mux()
        nodes += build_teleop()
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument(
            'lidar_device', default_value='',
            description='비우면 lidar.yaml의 /dev/rplidar(udev 규칙)를 쓴다.'),
        DeclareLaunchArgument(
            'base', default_value='true',
            description='micro-ROS 에이전트·오도메트리·EKF를 같이 띄울지.'),
        DeclareLaunchArgument(
            'ekf', default_value='true',
            description='EKF(바퀴 + 자이로)가 odom → base_footprint를 낸다. '
                        'false면 바퀴만으로 낸다.'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='slam_toolbox 지도 작성. false면 라이다 방향 확인만 한다.'),
        DeclareLaunchArgument(
            'teleop', default_value='true',
            description='twist_mux와 Xbox 패드. LB를 쥐고 몰면서 지도를 그린다.'),
        OpaqueFunction(function=_setup),
    ])
