"""care-telemetry/1 문서 조립·문구·push id. rclpy·네트워크를 쓰지 않는다.

정본: software/contracts/care-telemetry/v1/README.md (3장 문서 형식, 3.2 이벤트 표).
"""
from __future__ import annotations

import hashlib
import math
import random
import re
import threading
import uuid

PUSH_CHARS = '-0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ_abcdefghijklmnopqrstuvwxyz'
UUID_RE = re.compile(r'^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$')
KEY_BAD = re.compile(r'[.$#\[\]/\x00-\x1f\x7f]')

MISSION_STATES = ('idle', 'starting', 'active', 'arrived', 'cancelling', 'cancelled', 'error', 'unknown')
TRACKING_STATES = ('unavailable', 'searching', 'following', 'lost', 'idle', 'error', 'unknown')
BASE_AGENT_STATES = ('waiting', 'available', 'connected', 'disconnected')   # mr_msgs/BaseStatus.AGENT_*

SOS_REQUEST_MESSAGES = {
    'tracking_lost': '로봇이 어르신을 놓쳤어요',
    'stuck': '로봇이 움직이지 못하고 있어요',
    'low_battery': '로봇 배터리가 부족해요',
    'manual': '어르신이 도움을 요청했어요',
    'unknown': 'SOS가 발생했어요',
}
SOS_CLEAR_MESSAGES = {
    'tracking_recovered': '어르신을 다시 찾았어요',
    'handled': 'SOS가 처리되었어요',
    'cancelled': 'SOS가 취소되었어요',
    'unknown': 'SOS가 종료되었어요',
}
MSG_AGENT_ONLINE = '로봇이 온라인이 되었어요'
MSG_AGENT_OFFLINE = '로봇 기록 장치가 꺼졌어요'
MSG_LINK_CONNECTED = '어르신 폰이 로봇에 연결되었어요'
MSG_LINK_DISCONNECTED = '어르신 폰과 로봇의 연결이 끊어졌어요'
MSG_TRIP_CANCELLED = '주행을 멈췄어요'
MSG_TRIP_ARRIVED = '목적지에 도착했어요'
MSG_TRIP_ERROR = '주행 중 문제가 생겼어요'
MSG_SOS_ACK_OK = '어르신 폰이 응답했어요'
MSG_SOS_ACK_FAIL = '어르신 폰이 응답하지 않아요'


def josa_euro(word: str) -> str:
    """'(으)로'. 받침이 없거나 ㄹ받침이면 '로'."""
    code = ord(word[-1]) - 0xAC00 if word else -1
    if 0 <= code < 11172:
        jong = code % 28
        return '로' if jong in (0, 8) else '으로'
    return '(으)로'


def trip_start_message(from_label: str, to_label: str) -> str:
    return f'{from_label}에서 {to_label}{josa_euro(to_label)} 출발했어요'


def accuracy_bucket(has_fix: bool, accuracy_m: float | None, low_confidence: bool) -> str:
    if not has_fix:
        return 'none'
    if low_confidence or accuracy_m is None:
        return 'poor'
    if accuracy_m <= 10:
        return 'good'
    if accuracy_m <= 50:
        return 'fair'
    return 'poor'


def sos_wire_id(ros_id: str) -> str:
    """mr_phone_bridge.ros.backend.sos_wire_id와 같은 규칙(시험이 둘을 대조한다).
    ROS의 sos_id가 UUID v4가 아니면 앱에는 해시로 고정된 v4 모양 id가 간다."""
    low = ros_id.strip().lower()
    if UUID_RE.match(low):
        return low
    digest = hashlib.sha1(('mr-sos:' + ros_id).encode('utf-8')).digest()[:16]
    return str(uuid.UUID(bytes=digest, version=4))


def safe_key(s: str) -> str:
    """RTDB 키로 쓸 수 없는 문자를 바꾼다."""
    return KEY_BAD.sub('_', s) or '_'


def topic_tree(topics: dict[str, dict]) -> dict:
    """{'/base/status': leaf} → {'base': {'status': leaf}} (care-telemetry README 3.5).
    RTDB 키에 '/'를 쓸 수 없어서 토픽 이름을 경로로 펼친다. 한 이름이 다른 이름의 앞부분 경로면 ValueError."""
    tree: dict = {}
    for name, leaf in topics.items():
        parts = [safe_key(p) for p in name.strip('/').split('/') if p]
        if not parts:
            raise ValueError(f'empty topic name {name!r}')
        node = tree
        for p in parts[:-1]:
            node = node.setdefault(p, {})
            if 'last_seen_ms' in node:
                raise ValueError(f'topic {name!r} is below another topic')
        if parts[-1] in node:
            raise ValueError(f'topic {name!r} collides with another topic')
        node[parts[-1]] = dict(leaf)
    return tree


def finite_or_none(x):
    if x is None:
        return None
    try:
        x = float(x)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def event_doc(ts_ms: int, kind: str, severity: str, message: str, *, trip_id=None, sos_id=None,
              data: dict | None = None) -> dict:
    return {'ts_ms': int(ts_ms), 'kind': kind, 'severity': severity, 'trip_id': trip_id or None,
            'sos_id': sos_id or None, 'message': message[:200], 'data': dict(data or {})}


class PushIds:
    """RTDB push id(20자, 시간순). 같은 ms면 난수부를 1 올려 순서를 지킨다(firebase 클라이언트와 같은 방식)."""

    def __init__(self, rng: random.Random | None = None):
        self._rng = rng or random.SystemRandom()
        self._lock = threading.Lock()
        self._last_ts = -1
        self._last_rand = [0] * 12

    def __call__(self, ts_ms: int) -> str:
        with self._lock:
            if ts_ms == self._last_ts:
                i = 11
                while i >= 0 and self._last_rand[i] == 63:
                    self._last_rand[i] = 0
                    i -= 1
                if i >= 0:
                    self._last_rand[i] += 1
            else:
                self._last_rand = [self._rng.randrange(64) for _ in range(12)]
                self._last_ts = ts_ms
            ts = int(ts_ms)
            head = []
            for _ in range(8):
                head.append(PUSH_CHARS[ts % 64])
                ts //= 64
            return ''.join(reversed(head)) + ''.join(PUSH_CHARS[r] for r in self._last_rand)
