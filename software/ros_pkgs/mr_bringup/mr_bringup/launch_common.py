"""launch 파일들이 공유하는 노드 생성기.

launch 파일 자체는 인자 선언과 조립만 하고, 노드 하나를 어떻게 띄우는지는 전부
여기 모은다. 같은 노드가 여러 launch에 등장하는데(카메라는 perception·follow·align
셋 다 쓴다) 그때마다 인자를 다시 쓰면 언젠가 한 곳만 고쳐진다.

`maknae/mm_bringup/launch_common.py`의 구성을 그대로 가져왔다.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def share(package: str, *parts: str) -> str:
    return os.path.join(get_package_share_directory(package), *parts)


def build_description(use_sim_time: bool = False) -> list:
    """robot_state_publisher — base_link와 카메라 프레임을 세운다.

    `mr_base`가 내는 `odom → base_footprint` 아래로 나머지 체인이 붙는다.
    이게 없으면 aruco_opencv의 output_frame 변환이 실패한다.
    """
    # CAD 기반 mr_description이 xacro 매핑과 robot_state_publisher 구성을 소유한다.
    # bringup에서는 RViz와 가짜 joint state를 끄고 TF 발행만 포함한다.
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            share('mr_description', 'launch', 'description.launch.py')),
        launch_arguments={
            'use_sim_time': str(use_sim_time).lower(),
            'use_rviz': 'false',
            'publish_joint_states': 'false',
            'joint_state_gui': 'false',
        }.items(),
    )]


def build_camera(video_device: str = '') -> list:
    """usb_cam — /camera/image_raw, /camera/camera_info.

    네임스페이스가 곧 토픽 접두사다. usb_cam이 상대 토픽명을 쓰기 때문이다.

    usb_cam 0.8.1은 예전 V4L2 제어명 ``focus_auto``를 찾지만 Pi의
    현재 커널은 ``focus_automatic_continuous``를 노출한다. 노드 설정만
    믿으면 autofocus가 켜진 채 남아 승계한 캘리브레이션과 갈린다.
    그래서 동일 패키지가 런타임으로 가져오는 v4l2-ctl로 AF를 먼저
    끄고, 그 프로세스가 끝난 뒤에만 카메라를 연다.
    """
    device = video_device or '/dev/video0'
    params = [share('mr_bringup', 'config', 'c920.yaml')]
    if video_device:
        params.append({'video_device': video_device})
    camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='camera',
        output='screen',
        parameters=params,
    )
    disable_autofocus = ExecuteProcess(
        cmd=[
            'v4l2-ctl', '--device', device, '--set-ctrl',
            'focus_automatic_continuous=0',
        ],
        name='c920_disable_autofocus',
        output='screen',
    )
    # C920은 두 focus 제어를 한 ioctl에 묶으면 focus_absolute를 EIO로
    # 거절한다. 스트리밍을 열기 전에 AF -> absolute 순서로 따로 적용한다.
    set_focus_absolute = ExecuteProcess(
        cmd=[
            'v4l2-ctl', '--device', device, '--set-ctrl',
            'focus_absolute=0',
        ],
        name='c920_set_focus_absolute',
        output='screen',
    )
    set_focus_after_autofocus = RegisterEventHandler(OnProcessExit(
        target_action=disable_autofocus,
        on_exit=[set_focus_absolute],
    ))
    start_camera_after_focus = RegisterEventHandler(OnProcessExit(
        target_action=set_focus_absolute,
        on_exit=[camera],
    ))
    # 핸들러를 먼저 등록해 짧은 v4l2-ctl 프로세스의 exit 이벤트를
    # 놓치지 않는다.
    return [set_focus_after_autofocus, start_camera_after_focus,
            disable_autofocus]


def build_aruco_tracker(marker_id: int | None = None) -> list:
    """aruco_opencv — /aruco_detections.

    lifecycle이 아닌 autostart 실행 파일을 쓴다. 생명주기를 관리할 이유가 없다.
    """
    del marker_id            # 검출기는 모든 마커를 낸다. 선택은 marker_track이 한다.
    return [Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        name='aruco_tracker',
        output='screen',
        parameters=[share('mr_aruco', 'config', 'aruco_tracker.yaml')],
    )]


def build_marker_track(marker_id: int | None = None) -> list:
    params = [share('mr_aruco', 'config', 'marker_track.yaml')]
    if marker_id is not None:
        params.append({'target_marker_id': marker_id})
    return [Node(
        package='mr_aruco',
        executable='marker_track_node',
        name='marker_track',
        output='screen',
        parameters=params,
    )]


def build_follow(enabled_on_start: bool = False) -> list:
    return [Node(
        package='mr_aruco',
        executable='follow_node',
        name='follow',
        output='screen',
        parameters=[share('mr_aruco', 'config', 'follow.yaml'),
                    {'enabled_on_start': enabled_on_start}],
    )]


def build_align() -> list:
    return [Node(
        package='mr_aruco',
        executable='align_node',
        name='align',
        output='screen',
        parameters=[share('mr_aruco', 'config', 'align.yaml')],
    )]


def build_twist_mux() -> list:
    """twist_mux — 입력들을 중재해 /cmd_vel로 낸다.

    출력 토픽 이름이 `cmd_vel_out`이라 리맵이 필요하다.
    """
    return [Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[share('mr_bringup', 'config', 'twist_mux.yaml')],
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )]


def build_base(serial_device: str = '/dev/ttyAMA0', baud: str = '921600') -> list:
    """micro-ROS 에이전트 + 오도메트리 노드.

    baud와 장치는 계약이 정한 확정값이다(base_contract.md "물리 트랜스포트").
    ROS_DOMAIN_ID는 펌웨어가 0으로 정해 놨다 — 셸에 23이 남아 있으면 토픽이
    보이는데 데이터가 없는 고장이 난다.
    """
    # micro_ros_agent는 apt/주 작업공간이 아니라 전용 작업공간에 설치돼 있다.
    # launch를 어느 셸에서 실행해도 한 명령으로 기동되도록 그 환경을 명시해
    # 실행한다. bash의 위치 인자를 써서 장치명/baud를 셸 문자열에 삽입하지 않는다.
    microros_setup = os.path.expanduser('~/microros_ws/install/setup.bash')
    microros_agent = os.path.expanduser(
        '~/microros_ws/install/micro_ros_agent/lib/micro_ros_agent/'
        'micro_ros_agent')
    return [
        ExecuteProcess(
            cmd=[
                '/bin/bash', '-c',
                'source "$1" && exec "$2" serial --dev "$3" -b "$4"',
                'micro_ros_agent_launch', microros_setup, microros_agent,
                serial_device, baud,
            ],
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '0'},
        ),
        Node(
            package='mr_base',
            executable='odom_node',
            name='mr_base',
            output='screen',
            parameters=[share('mr_base', 'config', 'base_params.yaml')],
        ),
    ]
