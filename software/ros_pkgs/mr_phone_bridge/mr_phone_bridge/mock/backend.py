"""ROS 대신 쓰는 가짜 로봇. mission 상태 전이를 흉내 내고 ROS로 나갈 메시지를 로그로 보여 준다.

trip.start  → starting(tracking searching) ─1 s→ active(tracking following) [─arrive_after→ arrived]
trip.cancel → cancelling ─1 s→ cancelled ─3 s→ idle
실제 mr_mission과 같은 전이다. `arrived`는 OPEN-N1 결정 전까지 실제로는 나오지 않으며
mock에서만 `--arrive-after`로 볼 수 있다.
"""
from __future__ import annotations

import asyncio
import logging
from dataclasses import replace

from ..server.backend import ACTIVE_MISSION_STATES, Backend, MissionSnapshot

log = logging.getLogger('mock.ros')


class MockBackend(Backend):

    def __init__(self, *, start_delay_s: float = 1.0, cancel_delay_s: float = 1.0,
                 terminal_hold_s: float = 3.0, arrive_after_s: float | None = None):
        self.start_delay_s = start_delay_s
        self.cancel_delay_s = cancel_delay_s
        self.terminal_hold_s = terminal_hold_s
        self.arrive_after_s = arrive_after_s
        self.state = MissionSnapshot(mission_state='idle', tracking_state='unavailable')
        self.published: list[tuple[str, dict]] = []   # 테스트가 읽는다
        self._timers: list[asyncio.TimerHandle] = []
        self._last_link: dict | None = None

    # ------------------------------------------------------------ 상태

    def snapshot(self) -> MissionSnapshot:
        return self.state

    def set(self, **changes) -> None:
        self.state = replace(self.state, **changes)
        if self.core is not None and self.core.loop is not None:
            asyncio.ensure_future(self.core.notify_status())

    def _later(self, delay: float, fn) -> None:
        loop = asyncio.get_running_loop()
        self._timers.append(loop.call_later(delay, fn))

    def _cancel_timers(self) -> None:
        for t in self._timers:
            t.cancel()
        self._timers.clear()

    def begin_trip(self, trip_id: str, from_place_id: str, to_place_id: str, *, immediate: bool = False) -> None:
        self._cancel_timers()
        self.set(mission_state='active' if immediate else 'starting',
                 tracking_state='following' if immediate else 'searching',
                 trip_id=trip_id, from_place_id=from_place_id, to_place_id=to_place_id, last_error=None)
        if not immediate:
            self._later(self.start_delay_s, self._activate)

    def _activate(self) -> None:
        if self.state.mission_state != 'starting':
            return
        self.set(mission_state='active', tracking_state='following')
        if self.arrive_after_s is not None:
            self._later(self.arrive_after_s, self._arrive)

    def _arrive(self) -> None:
        if self.state.mission_state == 'active':
            self.set(mission_state='arrived', tracking_state='unavailable')
            self._later(self.terminal_hold_s, self._to_idle)

    def _to_idle(self) -> None:
        if self.state.mission_state in ('cancelled', 'arrived'):
            self.set(mission_state='idle', tracking_state='unavailable', trip_id=None,
                     from_place_id=None, to_place_id=None)

    # ------------------------------------------------------------ Backend

    def start_trip(self, *, request_id, trip_id, from_place_id, to_place_id) -> None:
        self._record('/phone/trip_command', {'command': 'START', 'request_id': request_id, 'trip_id': trip_id,
                                             'from_place_id': from_place_id, 'to_place_id': to_place_id})
        self.begin_trip(trip_id, from_place_id, to_place_id)

    def cancel_trip(self, *, request_id, trip_id, reason) -> None:
        self._record('/phone/trip_command', {'command': 'CANCEL', 'request_id': request_id,
                                             'trip_id': trip_id, 'reason': reason})
        if self.state.mission_state not in ACTIVE_MISSION_STATES or self.state.mission_state == 'cancelling':
            return
        self._cancel_timers()
        self.set(mission_state='cancelling')

        def done():
            self.set(mission_state='cancelled', tracking_state='unavailable')
            self._later(self.terminal_hold_s, self._to_idle)
        self._later(self.cancel_delay_s, done)

    def raise_sos(self, *, request_id, sos_id, trip_id) -> None:
        """mission 대역: 곧바로 manual SOS를 연다(앱이 sos.request를 받고 위치를 회신하는 흐름까지 시험할 수 있게)."""
        self._record('/phone/sos_raise', {'sos_id': sos_id, 'trip_id': trip_id})
        asyncio.ensure_future(self.core.request_sos(sos_id, 'manual', 'critical', trip_id))

    def cancel_sos(self, *, request_id, sos_id) -> None:
        self._record('/phone/sos_cancel', {'sos_id': sos_id})
        asyncio.ensure_future(self.core.clear_sos(sos_id, 'cancelled'))

    def publish_location(self, *, message_id, payload, low_confidence) -> None:
        self._record('/phone/location', {'message_id': message_id, 'reason': payload['reason'],
                                         'sos_id': payload['sos_id'], 'source': payload['source'],
                                         'has_fix': payload['has_fix'], 'low_confidence': low_confidence})

    def publish_link_status(self, *, connected, app_instance_id, session_id, last_seen_ms) -> None:
        data = {'connected': connected, 'app_instance_id': app_instance_id, 'session_id': session_id}
        # 연결 중에는 5 s마다 재발행된다. 같은 내용이면 로그를 남기지 않는다.
        self._record('/phone/link_status', data, quiet=data == self._last_link)
        self._last_link = data

    def publish_catalog(self, catalog) -> None:
        self._record('/robot/place_catalog', {'revision': catalog.revision, 'places': len(catalog.places)})

    def publish_sos_ack(self, *, sos_id, app_received, retries, has_location, best_source) -> None:
        self._record('/safety/sos_ack', {'sos_id': sos_id, 'app_received': app_received, 'retries': retries,
                                         'has_location': has_location, 'best_source': best_source})

    def _record(self, topic: str, data: dict, quiet: bool = False) -> None:
        self.published.append((topic, data))
        if len(self.published) > 1000:
            del self.published[:500]
        if not quiet:
            log.info('%s %s', topic, ' '.join(f'{k}={v}' for k, v in data.items()))
