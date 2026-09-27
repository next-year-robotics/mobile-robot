"""ROS 없는 시뮬 입력. 실제 노드와 같은 `TelemetryAgent.on_*`를 불러 대시보드 개발용 데이터를 만든다.

한 사이클(기본 약 60 s, --fast면 1/5):
  앱 연결 → idle → 집→시장 출발(starting→active, following) → 추적 상실 SOS → 앱 ACK → 캐시 위치 → 새 위치
  → 추적 회복 sos.clear → 멈춤(cancelling→cancelled→idle) → 앱 연결 끊김 → 다시 연결
"""
from __future__ import annotations

import logging
import threading
import time
import uuid

from .agent import TelemetryAgent

log = logging.getLogger(__name__)

PLACES = {'place_home': '집', 'place_market': '시장', 'place_mart': '마트', 'place_senior_center': '노인정',
          'place_church': '교회', 'place_temple': '절'}
HOME = (36.1234, 128.5678)


class Simulator:

    def __init__(self, agent: TelemetryAgent, *, speed: float = 1.0, stop: threading.Event | None = None):
        self.agent = agent
        self.speed = speed
        self.stop = stop or threading.Event()
        self.app_instance_id = str(uuid.uuid4())

    def sleep(self, s: float) -> bool:
        return not self.stop.wait(s / self.speed)

    def status(self, mission, tracking='unavailable', trip=None, frm=None, to=None, err=None):
        self.agent.on_app_status(mission_state=mission, tracking_state=tracking, trip_id=trip, from_place_id=frm,
                                 to_place_id=to, battery_percent=None, last_error=err)

    def link(self, connected: bool):
        self.agent.on_link_status(connected=connected, app_instance_id=self.app_instance_id,
                                  session_id=str(uuid.uuid4()) if connected else '',
                                  last_seen_ms=time.time_ns() // 1_000_000)

    def run(self, cycles: int | None = None) -> None:
        a = self.agent
        a.on_bridge_alive(True)
        a.on_base_status(agent_state='connected', cmd_watchdog_ok=True)
        a.on_place_catalog(PLACES)
        self.status('idle')
        self.link(False)
        n = 0
        while not self.stop.is_set() and (cycles is None or n < cycles):
            n += 1
            if not self.cycle():
                return

    def cycle(self) -> bool:
        a, s = self.agent, self.sleep
        self.link(True)
        if not s(5):
            return False
        trip = str(uuid.uuid4())
        self.status('starting', 'searching', trip, 'place_home', 'place_market')
        if not s(2):
            return False
        self.status('active', 'following', trip, 'place_home', 'place_market')
        if not s(15):
            return False
        sos = str(uuid.uuid4())
        self.status('active', 'lost', trip, 'place_home', 'place_market')
        a.on_sos_request(sos_id=sos, reason='tracking_lost', severity='critical', trip_id=trip)
        if not s(0.3):
            return False
        a.on_sos_ack(sos_id=sos, app_received=True, retries=0, has_location=False, best_source='none')
        now = time.time_ns() // 1_000_000
        a.on_location(reason='sos', sos_id=sos, has_fix=True, lat=HOME[0] + 0.0012, lon=HOME[1] + 0.0015,
                      accuracy_m=4.2, captured_at_ms=now - 800, source='cache', low_confidence=False)
        if not s(8):
            return False
        now = time.time_ns() // 1_000_000
        a.on_location(reason='sos', sos_id=sos, has_fix=True, lat=HOME[0] + 0.0013, lon=HOME[1] + 0.0016,
                      accuracy_m=3.1, captured_at_ms=now - 200, source='fresh', low_confidence=False)
        if not s(10):
            return False
        a.on_sos_clear(sos_id=sos, resolution='tracking_recovered')
        self.status('active', 'following', trip, 'place_home', 'place_market')
        if not s(5):
            return False
        a.on_trip_command(command='cancel', trip_id=trip, reason='user_request')
        self.status('cancelling', 'following', trip, 'place_home', 'place_market')
        if not s(1):
            return False
        self.status('cancelled', 'unavailable', trip, 'place_home', 'place_market')
        if not s(3):
            return False
        self.status('idle')
        if not s(5):
            return False
        self.link(False)
        return s(5)
