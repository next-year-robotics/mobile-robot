"""수동 조종 — 베이스 + twist_mux + Xbox 패드. 카메라와 자율 제어는 띄우지 않는다.

    ros2 launch mr_bringup teleop.launch.py
    # LB 누른 채 왼쪽 스틱 = 보통, RB 누른 채 = 빠르게. 손을 떼면 선다.

follow·align launch에도 패드가 기본으로 들어 있다. 자율 주행 중에 LB를 쥐면
사람이 넘겨받고, 놓으면 0.3 s 뒤 자율 입력으로 돌아간다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from mr_bringup.launch_common import build_base, build_teleop, build_twist_mux


def _setup(context, *args, **kwargs):
    del args, kwargs
    nodes = []
    if LaunchConfiguration('base').perform(context) == 'true':
        nodes += build_base(
            serial_device=LaunchConfiguration('serial_device').perform(context))
    nodes += build_twist_mux()
    nodes += build_teleop()
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument(
            'base', default_value='true',
            description='micro-ROS 에이전트와 오도메트리를 같이 띄울지. '
                        'false면 /cmd_vel까지만 나오고 바퀴는 안 돈다.'),
        OpaqueFunction(function=_setup),
    ])
