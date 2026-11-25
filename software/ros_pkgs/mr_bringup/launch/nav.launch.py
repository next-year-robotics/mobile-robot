"""Nav2 자율주행 실행 — SLAM(또는 저장 지도 AMCL) + Nav2(A* · RPP · Collision Monitor) + twist_mux.

사용 예:
    # 1) 실시간 2D SLAM + Nav2 동시 실행(실물)
    ros2 launch mr_bringup nav.launch.py slam:=true

    # 2) 저장한 지도로 AMCL 위치 추정 + Nav2 실행
    ros2 launch mr_bringup nav.launch.py slam:=false map:=$HOME/maps/home.yaml

    # 3) 음성·대시보드 `/nav/command` JSON 브리지와 함께 실행(기본 켜짐)
    ros2 topic pub --once /nav/command std_msgs/msg/String \
        "{data: '{\"cmd\":\"goto_pose\",\"id\":\"t1\",\"x\":1.98,\"y\":1.25,\"yaw\":0.0}'}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from mr_bringup.launch_common import (
    build_base,
    build_description,
    build_lidar,
    build_nav2,
    build_nav2_goal_bridge,
    build_slam,
    build_teleop,
    build_twist_mux,
)


def _setup(context, *args, **kwargs):
    del args, kwargs
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    slam = LaunchConfiguration('slam').perform(context) == 'true'
    map_file = LaunchConfiguration('map').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    nodes = build_description(use_sim_time=use_sim_time)
    if LaunchConfiguration('base').perform(context) == 'true':
        nodes += build_base(
            serial_device=LaunchConfiguration('serial_device').perform(context),
            ekf=LaunchConfiguration('ekf').perform(context) == 'true',
        )
    if LaunchConfiguration('lidar').perform(context) == 'true':
        nodes += build_lidar(
            lidar_device=LaunchConfiguration('lidar_device').perform(context),
        )
    if slam:
        nodes += build_slam(use_sim_time=use_sim_time)
    if LaunchConfiguration('twist_mux').perform(context) == 'true':
        nodes += build_twist_mux()
    if LaunchConfiguration('teleop').perform(context) == 'true':
        nodes += build_teleop()
    nodes += build_nav2(
        params_file=params_file,
        map_file=map_file,
        slam=slam,
        use_sim_time=use_sim_time,
    )
    if LaunchConfiguration('nav_bridge').perform(context) == 'true':
        nodes += build_nav2_goal_bridge(use_sim_time=use_sim_time)
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('lidar_device', default_value=''),
        DeclareLaunchArgument('base', default_value='true'),
        DeclareLaunchArgument('ekf', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('twist_mux', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='true'),
        DeclareLaunchArgument('nav_bridge', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
