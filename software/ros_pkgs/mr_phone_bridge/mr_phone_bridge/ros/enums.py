"""WebSocket 문자열 ↔ mr_care_msgs 정수 상수 변환표. 변환표는 이 파일 한 곳에만 둔다(PROTOCOL.md 7.2).

rclpy·메시지 패키지를 import하지 않는다. 값이 .msg 상수와 같은지는 test_ros_enums.py가 확인한다.
- 모르는 정수 → 'unknown' (없으면 표의 기본값)
- 모르는 문자열 → ValueError. 앱 입력은 스키마 검증을 이미 통과했으므로 여기서 나면 코드 버그다.
"""
from __future__ import annotations

# RobotAppStatus.MISSION_*
MISSION = {'idle': 0, 'starting': 1, 'active': 2, 'arrived': 3, 'cancelling': 4, 'cancelled': 5, 'error': 6,
           'unknown': 255}
# RobotAppStatus.TRACKING_*
TRACKING = {'unavailable': 0, 'searching': 1, 'following': 2, 'lost': 3, 'error': 4, 'unknown': 255}
# TripCommand.COMMAND_*
TRIP_COMMAND = {'start': 1, 'cancel': 2}
# PhoneLocation.REASON_*
LOCATION_REASON = {'periodic': 1, 'sos': 2}
# PhoneLocation.SOURCE_*
LOCATION_SOURCE = {'none': 0, 'cache': 1, 'fresh': 2}
# PhoneLocation.FAILURE_*  (None = 실패 없음)
LOCATION_FAILURE = {None: 0, 'permission_denied': 1, 'provider_disabled': 2, 'timeout': 3, 'unavailable': 4}
# SosRequest.REASON_*
SOS_REASON = {'unknown': 0, 'tracking_lost': 1, 'stuck': 2, 'low_battery': 3, 'manual': 4}
# SosRequest.SEVERITY_*  (0은 정의되지 않음 → critical로 본다)
SOS_SEVERITY = {'critical': 1, 'warning': 2}
# SosClear.RESOLUTION_*
SOS_RESOLUTION = {'unknown': 0, 'tracking_recovered': 1, 'handled': 2, 'cancelled': 3}
# mr_aruco_msgs/MarkerTrack.STATUS_* → tracking_state (PROTOCOL.md 7.3)
MARKER_STATUS_TO_TRACKING = {0: 'lost', 1: 'following', 2: 'following'}

_DEFAULTS = {id(SOS_SEVERITY): 'critical', id(LOCATION_FAILURE): None}


def to_int(table: dict, value) -> int:
    try:
        return table[value]
    except KeyError:
        raise ValueError(f'unknown enum value {value!r}') from None


def to_str(table: dict, value: int):
    for k, v in table.items():
        if v == value:
            return k
    return _DEFAULTS.get(id(table), 'unknown')
