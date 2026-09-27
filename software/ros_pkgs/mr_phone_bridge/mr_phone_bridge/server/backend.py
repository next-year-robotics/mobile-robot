"""브리지 코어가 로봇 쪽에 기대하는 인터페이스. mock과 ROS 노드가 각각 구현한다.

모든 메서드는 asyncio 루프 스레드에서 불린다. 블로킹하지 않는다(ROS 쪽은 큐에 넣기만 한다).
로봇 쪽에서 상태가 바뀌면 `core.notify_status()`를, SOS는 `core.request_sos()`/`core.clear_sos()`를
같은 루프에서 부른다(다른 스레드면 `loop.call_soon_threadsafe`).
"""
from __future__ import annotations

from dataclasses import dataclass

BRIDGE_STATES = ('ready', 'degraded', 'error')
MISSION_STATES = ('idle', 'starting', 'active', 'arrived', 'cancelling', 'cancelled', 'error', 'unknown')
TRACKING_STATES = ('unavailable', 'searching', 'following', 'lost', 'error', 'unknown')
# 주행이 진행 중인 상태. 이때 trip.start는 INVALID_STATE, 쓰는 장소는 삭제·이동 금지.
ACTIVE_MISSION_STATES = frozenset({'starting', 'active', 'cancelling'})
# 새 trip.start를 받을 수 있는 상태. unknown(mission 노드 없음)은 NOT_READY.
STARTABLE_MISSION_STATES = frozenset({'idle', 'arrived', 'cancelled', 'error'})


@dataclass(frozen=True)
class MissionSnapshot:
    """robot.status를 채울 로봇 측 상태(PROTOCOL.md 7.3). robot_id는 코어가 채운다."""
    bridge_state: str = 'ready'
    mission_state: str = 'unknown'
    tracking_state: str = 'unavailable'
    trip_id: str | None = None
    from_place_id: str | None = None
    to_place_id: str | None = None
    battery_percent: float | None = None
    last_error: str | None = None


class Backend:
    """기본 구현은 아무것도 하지 않는다. 필요한 것만 덮어쓴다."""

    core = None

    def attach(self, core) -> None:
        self.core = core

    def snapshot(self) -> MissionSnapshot:
        return MissionSnapshot()

    def start_trip(self, *, request_id: str, trip_id: str, from_place_id: str, to_place_id: str) -> None:
        """ProtocolError를 올리면 앱에 그대로 error로 간다."""

    def cancel_trip(self, *, request_id: str, trip_id: str, reason: str) -> None:
        """정지 요청. 거절하지 않는다."""

    def publish_location(self, *, message_id: str, payload: dict, low_confidence: bool) -> None:
        pass

    def publish_link_status(self, *, connected: bool, app_instance_id: str, session_id: str,
                            last_seen_ms: int) -> None:
        pass

    def publish_catalog(self, catalog) -> None:
        pass

    def publish_sos_ack(self, *, sos_id: str, app_received: bool, retries: int, has_location: bool,
                        best_source: str) -> None:
        pass
