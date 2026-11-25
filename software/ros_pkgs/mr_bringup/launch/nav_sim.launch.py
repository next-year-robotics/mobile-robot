"""PC 단독 Nav2 + SLAM 통합 시뮬레이션 실행.

실물 하드웨어 없이 2D 실내 차동구동·RPLIDAR C1 시뮬레이터(`nav2_sim_robot`) 위에서
`mr_description` + `twist_mux` + `slam_toolbox` + `Nav2(A* · RPP · Collision Monitor)` +
`nav2_goal_bridge`를 한 번에 띄운다.

    ros2 launch mr_bringup nav_sim.launch.py
    # RViz2로 지도·코스트맵·A* 경로 확인:
    ros2 launch mr_bringup nav_sim.launch.py rviz:=true
    # 음성·대시보드와 동일한 `/nav/command` JSON으로 목표 전송:
    ros2 topic pub --once /nav/command std_msgs/msg/String \
        "{data: '{\"cmd\":\"goto_pose\",\"id\":\"demo1\",\"x\":1.98,\"y\":1.25,\"yaw\":0.0}'}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mr_bringup.launch_common import (
    build_description,
    build_nav2,
    build_nav2_goal_bridge,
    build_slam,
    build_twist_mux,
    share,
)


def _setup(context, *args, **kwargs):
    del args, kwargs
    nodes = build_description(use_sim_time=False)
    nodes.append(Node(
        package='mr_bringup',
        executable='nav2_sim_robot',
        name='nav2_sim_robot',
        output='screen',
    ))
    nodes += build_twist_mux()
    nodes += build_slam(use_sim_time=False)
    nodes += build_nav2(slam=True, use_sim_time=False)
    nodes += build_nav2_goal_bridge(use_sim_time=False, global_frame='map')
    if LaunchConfiguration('rviz').perform(context) == 'true':
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', share('mr_bringup', 'rviz', 'nav2.rviz')],
        ))
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='true이면 지도·코스트맵·A* 경로가 설정된 RViz2를 같이 연다.',
        ),
        OpaqueFunction(function=_setup),
    ])
