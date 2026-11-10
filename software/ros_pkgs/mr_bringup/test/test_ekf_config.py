"""ekf.yaml이 mr_base의 프레임·토픽과 MCU IMU가 실제로 내는 값에 맞는지 본다.

프레임 이름이 하나만 어긋나도 EKF는 죽지 않고 경고만 낸 채 입력을 버린다.
"""
from __future__ import annotations

from pathlib import Path

import pytest
import yaml

ROS_PKGS = Path(__file__).resolve().parents[2]
EKF_PATH = ROS_PKGS / 'mr_bringup' / 'config' / 'ekf.yaml'
BASE_PATH = ROS_PKGS / 'mr_base' / 'config' / 'base_params.yaml'

# robot_localization 15칸 순서
X, Y, Z, ROLL, PITCH, YAW, VX, VY, VZ, VROLL, VPITCH, VYAW, AX, AY, AZ = range(15)


@pytest.fixture(scope='module')
def ekf() -> dict:
    return yaml.safe_load(EKF_PATH.read_text())['ekf_filter_node']['ros__parameters']


@pytest.fixture(scope='module')
def base() -> dict:
    return yaml.safe_load(BASE_PATH.read_text())['mr_base']['ros__parameters']


def _fused(config: list) -> set[int]:
    assert len(config) == 15
    return {i for i, on in enumerate(config) if on}


def test_frames_match_wheel_odometry(ekf: dict, base: dict) -> None:
    assert ekf['base_link_frame'] == base['base_frame_id']
    assert ekf['odom_frame'] == base['odom_frame_id']
    assert ekf['world_frame'] == ekf['odom_frame']
    assert ekf['two_d_mode'] is True
    assert ekf['publish_tf'] is True


def test_inputs_are_mr_base_outputs(ekf: dict) -> None:
    # odom_node는 'odom', imu_node는 'imu/data'(공분산을 채운 쪽)로 낸다.
    assert ekf['odom0'] == 'odom'
    assert ekf['imu0'] == 'imu/data'


def test_wheel_odometry_contributes_velocities_only(ekf: dict) -> None:
    assert _fused(ekf['odom0_config']) == {VX, VYAW}


def test_imu_contributes_gyro_z_only(ekf: dict) -> None:
    # MCU는 orientation을 내지 않는다(orientation_covariance[0] = -1). 가속도는 적분하면 발산한다.
    assert _fused(ekf['imu0_config']) == {VYAW}
