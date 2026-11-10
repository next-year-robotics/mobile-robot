"""lidar.yaml·slam_toolbox.yaml이 URDF, mr_base, EKF와 같은 이름을 쓰는지 본다.

frame 이름이 하나 어긋나면 slam_toolbox는 죽지 않고 tf 경고만 낸 채 스캔을 버린다.
"""
from __future__ import annotations

from pathlib import Path

import pytest
import yaml

ROS_PKGS = Path(__file__).resolve().parents[2]
CONFIG = ROS_PKGS / 'mr_bringup' / 'config'
XACRO_PATH = ROS_PKGS / 'mr_description' / 'urdf' / 'mobile_robot.urdf.xacro'
BASE_PATH = ROS_PKGS / 'mr_base' / 'config' / 'base_params.yaml'


def _params(path: Path, node: str) -> dict:
    return yaml.safe_load(path.read_text())[node]['ros__parameters']


@pytest.fixture(scope='module')
def lidar() -> dict:
    return _params(CONFIG / 'lidar.yaml', 'sllidar_node')


@pytest.fixture(scope='module')
def slam() -> dict:
    return _params(CONFIG / 'slam_toolbox.yaml', 'slam_toolbox')


def test_lidar_frame_exists_in_urdf(lidar: dict) -> None:
    assert f'<link name="{lidar["frame_id"]}"/>' in XACRO_PATH.read_text()


def test_lidar_is_c1_on_udev_name(lidar: dict) -> None:
    assert lidar['serial_baudrate'] == 460800
    assert lidar['serial_port'] == '/dev/rplidar'
    rule = (CONFIG / 'udev' / '99-rplidar.rules').read_text()
    assert 'SYMLINK+="rplidar"' in rule


def test_slam_frames_match_odometry(slam: dict) -> None:
    base = _params(BASE_PATH, 'mr_base')
    ekf = _params(CONFIG / 'ekf.yaml', 'ekf_filter_node')
    assert slam['base_frame'] == base['base_frame_id'] == ekf['base_link_frame']
    assert slam['odom_frame'] == base['odom_frame_id'] == ekf['odom_frame']
    assert slam['map_frame'] == ekf['map_frame']


def test_slam_reads_driver_scan_within_c1_range(slam: dict) -> None:
    # sllidar_node는 상대 이름 scan으로 낸다.
    assert slam['scan_topic'] == '/scan'
    assert slam['mode'] == 'mapping'
    assert 0.05 <= slam['min_laser_range'] < slam['max_laser_range'] <= 12.0
