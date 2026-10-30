"""`/imu/data_raw`에 붙일 공분산. ROS 비의존.

MCU는 공분산을 전부 0(= sensor_msgs/Imu 규약상 "모름")으로 보낸다. 값은 캘리브레이션
상수라 펌웨어 밖에 두고(upgrade_plan.md 5.4절 2항), 이 노드가 파라미터로 채운다.
0을 그대로 EKF에 넘기면 robot_localization은 "무한 신뢰"로 읽는다.
"""
from __future__ import annotations

import math


def covariance_from_variances(variances) -> list:
    """축별 분산 3개를 3x3 row-major 대각 공분산으로 편다.

    0이나 음수, NaN은 받지 않는다. 0은 소비자에게 "완전한 센서"로, 음수는 규약상
    "이 값 없음"(-1)으로 읽히므로 둘 다 설정 실수다.
    """
    values = [float(v) for v in variances]
    if len(values) != 3:
        raise ValueError(f'variance needs 3 entries (x, y, z): {values}')
    for v in values:
        if not math.isfinite(v) or v <= 0.0:
            raise ValueError(f'variance must be finite and > 0: {values}')
    cov = [0.0] * 9
    for i, v in enumerate(values):
        cov[i * 4] = v
    return cov


def variance_from_density(noise_density: float, noise_bandwidth_hz: float) -> float:
    """데이터시트 노이즈 밀도 × √NBW로 RMS를 내고 제곱한다.

    noise_density 단위가 곧 결과 단위의 제곱근이다(예: rad/s/√Hz → (rad/s)²).
    진동 없는 정지 조건의 하한이다. 실차 값은 이보다 크다.
    """
    rms = float(noise_density) * math.sqrt(float(noise_bandwidth_hz))
    return rms * rms
