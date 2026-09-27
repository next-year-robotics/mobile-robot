"""ROS 쪽 Backend. ROS executor 스레드와 asyncio 스레드 사이의 경계다.

- ROS 콜백(ROS 스레드)은 `update_*`로 최신 입력을 넣는다. 잠금으로 보호한다.
- 코어(asyncio 스레드)는 `snapshot()`으로 읽고, `start_trip`·`publish_*`는 `outbox` 큐에 넣기만 한다.
  노드의 타이머(ROS 스레드)가 큐를 비우며 실제로 발행한다. 서로를 blocking하지 않는다.
rclpy를 import하지 않으므로 ROS 없이도 단위 시험할 수 있다.

robot.status 채우기(PROTOCOL.md 7.3):
1. `/robot/app_status`가 신선하면 그대로 옮긴다.
2. 없거나 낡았으면 mission_state=unknown, tracking_state는 marker_track(TRACKING/HOLDING→following, LOST→lost,
   없음→unavailable).
3. battery_percent는 app_status 값(-1이면 null).
4. bridge_state: 내부 오류면 error, app_status·marker_track 모두 낡았으면 degraded, 아니면 ready.
"""
from __future__ import annotations

import hashlib
import logging
import queue
import threading
import time
import uuid
from collections import OrderedDict
from typing import Callable

from ..protocol.envelope import is_uuid
from ..server.backend import Backend, MissionSnapshot
from . import enums

log = logging.getLogger(__name__)

MAX_SOS_IDS = 64


def sos_wire_id(ros_id: str) -> str:
    """ROS의 sos_id를 앱에 보낼 UUID v4로 바꾼다. UUID가 아니면(CLI 시험 등) 해시로 고정된 v4 모양 id를 만든다."""
    low = ros_id.strip().lower()
    if is_uuid(low):
        return low
    digest = hashlib.sha1(('mr-sos:' + ros_id).encode('utf-8')).digest()[:16]
    return str(uuid.UUID(bytes=digest, version=4))


class RosBackend(Backend):

    def __init__(self, *, app_status_stale_s: float = 6.0, marker_stale_s: float = 1.0,
                 clock: Callable[[], float] = time.monotonic):
        self.app_status_stale_s = app_status_stale_s
        self.marker_stale_s = marker_stale_s
        self._clock = clock
        self._lock = threading.Lock()
        self._app: dict | None = None
        self._app_at = 0.0
        self._marker: int | None = None
        self._marker_at = 0.0
        self.internal_error: str | None = None
        self.outbox: queue.SimpleQueue = queue.SimpleQueue()
        self._sos_ids: OrderedDict[str, str] = OrderedDict()     # wire id → ROS id

    # ------------------------------------------------------------ ROS 스레드

    def update_app_status(self, fields: dict, age_s: float = 0.0) -> None:
        """fields: mission_state·tracking_state(문자열), trip_id·from_place_id·to_place_id·last_error(없으면 None),
        battery_percent(없으면 None). age_s: 받는 시점에 이미 지난 시간(header.stamp 기준)."""
        with self._lock:
            self._app = dict(fields)
            self._app_at = self._clock() - max(0.0, age_s)

    def update_marker(self, status: int) -> None:
        with self._lock:
            self._marker = status
            self._marker_at = self._clock()

    def remember_sos(self, ros_id: str) -> str:
        wire = sos_wire_id(ros_id)
        with self._lock:
            self._sos_ids[wire] = ros_id
            self._sos_ids.move_to_end(wire)
            while len(self._sos_ids) > MAX_SOS_IDS:
                self._sos_ids.popitem(last=False)
        if wire != ros_id:
            log.warning('sos_id %r is not a lowercase UUID v4; using %s on the wire', ros_id, wire)
        return wire

    # ------------------------------------------------------------ asyncio 스레드 (Backend)

    def snapshot(self) -> MissionSnapshot:
        now = self._clock()
        with self._lock:
            app, app_at, marker, marker_at = self._app, self._app_at, self._marker, self._marker_at
        app_fresh = app is not None and now - app_at <= self.app_status_stale_s
        marker_fresh = marker is not None and now - marker_at <= self.marker_stale_s
        if self.internal_error:
            bridge_state = 'error'
        elif app_fresh or marker_fresh:
            bridge_state = 'ready'
        else:
            bridge_state = 'degraded'
        if app_fresh:
            return MissionSnapshot(
                bridge_state=bridge_state,
                mission_state=app['mission_state'],
                tracking_state=app['tracking_state'],
                trip_id=app.get('trip_id'),
                from_place_id=app.get('from_place_id'),
                to_place_id=app.get('to_place_id'),
                battery_percent=app.get('battery_percent'),
                last_error=app.get('last_error'),
            )
        tracking = enums.MARKER_STATUS_TO_TRACKING.get(marker, 'unknown') if marker_fresh else 'unavailable'
        return MissionSnapshot(bridge_state=bridge_state, mission_state='unknown', tracking_state=tracking)

    def start_trip(self, *, request_id, trip_id, from_place_id, to_place_id) -> None:
        self.outbox.put(('trip_command', {'request_id': request_id, 'trip_id': trip_id, 'command': 'start',
                                          'from_place_id': from_place_id, 'to_place_id': to_place_id,
                                          'reason': ''}))

    def cancel_trip(self, *, request_id, trip_id, reason) -> None:
        self.outbox.put(('trip_command', {'request_id': request_id, 'trip_id': trip_id, 'command': 'cancel',
                                          'from_place_id': '', 'to_place_id': '', 'reason': reason}))

    def publish_location(self, *, message_id, payload, low_confidence) -> None:
        self.outbox.put(('location', {'message_id': message_id, 'payload': dict(payload),
                                      'low_confidence': low_confidence}))

    def publish_link_status(self, *, connected, app_instance_id, session_id, last_seen_ms) -> None:
        self.outbox.put(('link_status', {'connected': connected, 'app_instance_id': app_instance_id,
                                         'session_id': session_id, 'last_seen_ms': last_seen_ms}))

    def publish_catalog(self, catalog) -> None:
        self.outbox.put(('catalog', catalog.to_payload()))

    def publish_sos_ack(self, *, sos_id, app_received, retries, has_location, best_source) -> None:
        with self._lock:
            ros_id = self._sos_ids.get(sos_id, sos_id)
        self.outbox.put(('sos_ack', {'sos_id': ros_id, 'app_received': app_received, 'retries': retries,
                                     'has_location': has_location, 'best_source': best_source}))
