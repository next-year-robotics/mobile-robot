"""twist_mux만 띄운다. M8 회귀 검증용.

베이스만 있는 상태에서 중재가 제대로 도는지 먼저 확인한다.

    ros2 launch mr_bringup twist_mux.launch.py
    python3 tools/calib/kbd_teleop.py --via-mux
    ros2 topic pub -1 /safety/lock std_msgs/msg/Bool "{data: true}"   # 정지
"""
from launch import LaunchDescription

from mr_bringup.launch_common import build_twist_mux


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(build_twist_mux())
