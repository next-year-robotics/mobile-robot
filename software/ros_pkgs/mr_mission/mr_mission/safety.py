"""안전 판단 상태 기계(M3). rclpy를 import하지 않는다. 규칙과 선택지는 docs/senior-care/m3-safety-design.md.

D1~D5(SOS, `SafetyMonitor`)와 D6(앱 연결 끊김, `LinkWatch`) 모두 2026-09-27 사용자 결정. 노드(node.py)가 배선하고
파라미터 기본값은 config/mission.yaml(`sos_stop_on_no_ack: true`, `link_lost_stop_s: 30.0`). 여기 dataclass 기본값은
시험용이다(`stop_on_no_ack=False`는 "D4 끔" 경로를 시험하려고 남겼다).

    D1 주행 중(active)이고 이번 trip에서 마커를 한 번이라도 잡은 뒤, LOST(또는 marker_track 끊김)가
       lost_to_sos_s 이어지면 sos_request 한 번. searching(아직 못 잡음)·주행 밖에서는 내지 않는다.
    D2 해제되면 곧바로 재무장. 다음 SOS도 다시 lost_to_sos_s가 필요하다.
    D3 마커를 recover_to_clear_s 연속으로 다시 잡으면 sos_clear(tracking_recovered). trip 상태와 무관.
    D3b SOS 중 앱 명령: 멈춤 → sos_clear(cancelled), 새 출발 → sos_clear(handled).
    D4 SosAck(app_received=false)이고 stop_on_no_ack이면 ('stop', ...) 한 번. 주행 중일 때만.
    D5 trip 상태는 바꾸지 않는다. 정지는 호출자(mission)가 follow disable 경로로 한다.

입력 메서드는 모두 now(단조 시계, s)를 받고 actions 목록을 돌려준다:
    ('sos_request', sos_id, reason, severity, trip_id)
    ('sos_clear', sos_id, resolution)
    ('stop', message)
"""
from __future__ import annotations

import uuid
from dataclasses import dataclass
from typing import Callable

MARKER_LOST, MARKER_TRACKING, MARKER_HOLDING = 0, 1, 2
MSG_STOPPED_NO_ACK = '어르신 폰이 응답하지 않아 멈췄어요'
MSG_STOPPED_LINK_LOST = '폰 연결이 끊겨 멈췄어요'
TRIP_MOVING = ('starting', 'active')


@dataclass
class SafetyConfig:
    lost_to_sos_s: float = 10.0          # D1
    recover_to_clear_s: float = 5.0      # D3
    marker_stale_s: float = 1.0          # 이만큼 marker_track이 안 오면 끊김(= 못 봄)
    stop_on_no_ack: bool = False         # D4(OPEN-R3). 결정 전: False
    severity: str = 'critical'


class SafetyMonitor:

    def __init__(self, cfg: SafetyConfig | None = None, *, new_id: Callable[[], str] | None = None):
        self.cfg = cfg or SafetyConfig()
        self._new_id = new_id or (lambda: str(uuid.uuid4()))
        self.mission_state = 'idle'
        self.trip_id: str | None = None
        self._acquired = False               # 이번 trip에서 한 번이라도 잡았나
        self._marker_at: float | None = None   # 마지막 marker_track 수신 시각
        self._seen_at: float | None = None     # 마지막으로 TRACKING/HOLDING을 받은 시각
        self._recover_since: float | None = None
        self.sos_id: str | None = None       # 활성 SOS
        self._stop_sent = False
        self.log: list[str] = []

    # ------------------------------------------------------------ 입력

    def mission(self, now: float, state: str, trip_id: str | None) -> list:
        """mission 상태가 바뀔 때마다(또는 주기적으로) 부른다."""
        if trip_id != self.trip_id:
            self._acquired = False
        self.mission_state, self.trip_id = state, trip_id
        return self.tick(now)

    def marker(self, now: float, status: int) -> list:
        seen = status in (MARKER_TRACKING, MARKER_HOLDING)
        gap = self._marker_at is not None and now - self._marker_at > self.cfg.marker_stale_s
        self._marker_at = now
        if seen:
            if self.mission_state in ('starting', 'active'):      # mission의 searching 판정과 같다
                self._acquired = True
            if self._recover_since is None or gap:
                self._recover_since = now
            self._seen_at = now
        else:
            self._recover_since = None
        return self.tick(now)

    def trip_command(self, now: float, command: str) -> list:
        """앱의 trip.start/cancel(= /phone/trip_command). SOS 중이면 어르신이 앱을 직접 조작한 것으로 보고 해제."""
        if self.sos_id is None or command not in ('cancel', 'start'):
            return []
        # 이번 놓침은 끝난 것으로 본다: 새 SOS는 마커를 다시 잡은 뒤에만(mission이 아직 active여도 곧바로 다시 내지 않게)
        self._acquired = False
        return self._clear('cancelled' if command == 'cancel' else 'handled')

    def sos_ack(self, now: float, sos_id: str, app_received: bool) -> list:
        if sos_id != self.sos_id or app_received or self._stop_sent:
            return []
        self.log.append(f'SOS {sos_id}: phone did not acknowledge')
        if self.cfg.stop_on_no_ack and self.mission_state in ('starting', 'active'):
            self._stop_sent = True
            return [('stop', MSG_STOPPED_NO_ACK)]
        return []

    def tick(self, now: float) -> list:
        c = self.cfg
        if self._recover_since is not None and self._marker_at is not None and now - self._marker_at > c.marker_stale_s:
            self._recover_since = None                      # 회복 중 marker_track이 끊겼다
        if self.sos_id is not None:
            if self._recover_since is not None and now - self._recover_since >= c.recover_to_clear_s:
                return self._clear('tracking_recovered')
            return []
        if self.mission_state != 'active' or not self._acquired:
            return []
        if self._seen_at is not None and now - self._seen_at >= c.lost_to_sos_s:
            return self._request(now)
        return []

    # ------------------------------------------------------------ 출력

    def lost_for(self, now: float) -> float:
        """이번 trip에서 잡은 뒤 못 본 시간(s). 잡은 적 없으면 0."""
        if not self._acquired or self._seen_at is None:
            return 0.0
        return max(0.0, now - self._seen_at)

    # ------------------------------------------------------------ 내부

    def _request(self, now: float) -> list:
        self.sos_id = self._new_id()
        self._stop_sent = False
        self._recover_since = None
        self.log.append(f'SOS {self.sos_id}: marker lost for {now - self._seen_at:.1f} s (trip {self.trip_id})')
        return [('sos_request', self.sos_id, 'tracking_lost', self.cfg.severity, self.trip_id)]

    def _clear(self, resolution: str) -> list:
        sid, self.sos_id = self.sos_id, None
        self._stop_sent = False
        self.log.append(f'SOS {sid} cleared: {resolution}')
        return [('sos_clear', sid, resolution)]


# ============================================================ D6: 주행 중 앱 연결 끊김

@dataclass
class LinkConfig:
    stop_after_s: float = 30.0           # D6(2026-09-27 결정). 0이면 끔
    stale_s: float = 12.0                # connected=true가 이만큼 갱신되지 않으면 끊김(브리지는 연결 중 5 s마다 재발행)


class LinkWatch:
    """D6: 주행 중(starting/active) 앱 링크(/phone/link_status)가 stop_after_s 연속 끊겨 있으면 ('stop', 문구)를
    trip마다 한 번 낸다. 정지는 호출자(mission)가 멈춤 경로(follow disable + 정지 확인 → error)로 한다. 자동 재개 없음.

    끊김 = 마지막 link_status가 connected=false, 또는 connected=true가 stale_s 넘게 갱신 안 됨(브리지 프로세스가 죽으면
    latched true가 남는다), 또는 받은 적이 없음. 끊긴 시간은 trip 안에서만 센다: max(끊긴 때, trip 시작).
    브리지가 끊김을 알아채는 시점 기준이다(응답 없이 사라진 앱은 브리지 heartbeat 타임아웃 10 s가 앞에 붙는다).
    """

    def __init__(self, cfg: LinkConfig | None = None, now: float = 0.0):
        self.cfg = cfg or LinkConfig()
        self._started = now
        self._connected = False
        self._msg_at: float | None = None
        self._false_since: float | None = None
        self.state = 'idle'
        self.trip_id: str | None = None
        self._trip_since = now
        self._warned: str | None = None      # 이 trip에서 끊김을 알렸나
        self._stopped: str | None = None     # 이 trip을 이미 세웠나
        self.log: list[str] = []

    def down_since(self, now: float) -> float | None:
        """링크가 끊긴 시각. 연결돼 있으면 None."""
        if self._msg_at is None:
            return self._started
        if not self._connected:
            return self._false_since
        if now - self._msg_at > self.cfg.stale_s:
            return self._msg_at + self.cfg.stale_s
        return None

    def link(self, now: float, connected: bool) -> list:
        prev_down = self.down_since(now)
        self._msg_at = now
        self._connected = bool(connected)
        if not connected:
            self._false_since = prev_down if prev_down is not None else now
        return self.tick(now)

    def mission(self, now: float, state: str, trip_id: str | None) -> list:
        if trip_id != self.trip_id or (state == 'starting' and self.state != 'starting'):   # 새 trip(같은 id 재사용 포함)
            self._trip_since = now
            self._warned = self._stopped = None
        self.state, self.trip_id = state, trip_id
        return self.tick(now)

    def tick(self, now: float) -> list:
        trip = self.trip_id
        if self.cfg.stop_after_s <= 0 or self.state not in TRIP_MOVING or not trip or self._stopped == trip:
            return []
        down = self.down_since(now)
        if down is None:
            if self._warned == trip:
                self._warned = None
                self.log.append(f'phone link restored during trip {trip}: keep going')
            return []
        since = max(down, self._trip_since)
        if self._warned != trip:
            self._warned = trip
            left = max(0.0, self.cfg.stop_after_s - (now - since))
            self.log.append(f'phone link down during trip {trip}: stopping in {left:.0f} s unless it returns')
        if now - since >= self.cfg.stop_after_s:
            self._stopped = trip
            self.log.append(f'phone link down for {now - since:.1f} s during trip {trip}: safety stop')
            return [('stop', MSG_STOPPED_LINK_LOST)]
        return []

    def down_for(self, now: float) -> float:
        """이번 trip 안에서 끊겨 있던 시간(s). 연결돼 있거나 주행 밖이면 0."""
        down = self.down_since(now)
        if down is None or self.state not in TRIP_MOVING:
            return 0.0
        return max(0.0, now - max(down, self._trip_since))
