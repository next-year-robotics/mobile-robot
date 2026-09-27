"""최소 mission 상태 기계. rclpy를 import하지 않는다(노드는 node.py).

    idle ─start→ starting ─follow enable 성공→ active
                    │  enable 실패/시간 초과 → error
    starting/active ─cancel→ cancelling ─disable 성공 + 정지 확인→ cancelled ─3 s→ idle
                                   │ disable 실패 반복/정지 미확인 → error
    arrived는 OPEN-N1(도착 판정 수단) 결정 전까지 절대 내지 않는다.

- 정지는 거절하지 않는다. 어떤 상태에서 cancel이 와도 follow disable을 보낸다.
- 정지 확인: /odom 속도가 |v|<0.02 m/s, |w|<0.05 rad/s로 0.3 s 유지. /odom이 없으면 disable 성공 1 s 뒤
  시간 기준으로 판정한다(follow 노드는 비활성 전이 때 0을 한 번 보내고, MCU 워치독 200 ms가 backstop).
- 늦게 온 enable 성공(이미 취소·오류)에는 곧바로 disable을 다시 보낸다.
출력은 `actions`: ('enable', True|False) 목록. 노드가 서비스를 부르고 결과를 `enable_result`로 돌려준다.
"""
from __future__ import annotations

from dataclasses import dataclass

TRIP_STATES = ('starting', 'active', 'cancelling')
MARKER_LOST, MARKER_TRACKING, MARKER_HOLDING = 0, 1, 2


@dataclass
class MissionConfig:
    enable_timeout_s: float = 3.0
    stop_timeout_s: float = 5.0
    stop_fallback_s: float = 1.0
    stop_linear_eps: float = 0.02
    stop_angular_eps: float = 0.05
    stop_hold_s: float = 0.3
    odom_stale_s: float = 0.5
    disable_retry_s: float = 0.5
    terminal_hold_s: float = 3.0
    marker_stale_s: float = 1.0


MSG_ENABLE_FAILED = '추종을 시작하지 못했어요'
MSG_STOP_UNCONFIRMED = '정지를 확인하지 못했어요'


class Mission:

    def __init__(self, cfg: MissionConfig | None = None):
        self.cfg = cfg or MissionConfig()
        self.state = 'idle'
        self.trip_id: str | None = None
        self.from_place_id: str | None = None
        self.to_place_id: str | None = None
        self.last_error: str | None = None
        self._entered = 0.0            # 현재 상태에 들어온 시각
        self._acquired = False         # 이번 trip에서 마커를 한 번이라도 잡았나
        self._disable_ok_at: float | None = None
        self._disable_pending = False
        self._next_disable_at: float | None = None
        self._odom_at: float | None = None
        self._still_since: float | None = None
        self._marker: int | None = None
        self._marker_at = 0.0
        self.log: list[str] = []       # 노드가 비워 가며 로그로 옮긴다

    # ------------------------------------------------------------ 입력

    def start(self, now: float, trip_id: str, from_place_id: str, to_place_id: str) -> list:
        if self.state in TRIP_STATES:
            self.log.append(f'start {trip_id} ignored: trip {self.trip_id} is {self.state}')
            return []
        self.trip_id, self.from_place_id, self.to_place_id = trip_id, from_place_id, to_place_id
        self.last_error = None
        self._acquired = False
        self._set('starting', now)
        return [('enable', True)]

    def cancel(self, now: float, trip_id: str) -> list:
        if trip_id and self.trip_id and trip_id != self.trip_id:
            self.log.append(f'cancel for {trip_id} while current trip is {self.trip_id}: stopping anyway')
        if self.state == 'cancelling':
            return []
        if self.state in ('starting', 'active', 'error'):
            self._set('cancelling', now)
            self._disable_ok_at = None
            self._still_since = None
        return self._request_disable(now)

    def enable_result(self, now: float, want: bool, ok: bool, detail: str = '') -> list:
        if want:
            if not ok:
                self.log.append(f'follow enable failed: {detail}')
                if self.state == 'starting':
                    self._fail(now, MSG_ENABLE_FAILED)
                    return self._request_disable(now)
                return []
            if self.state == 'starting':
                self._set('active', now)
                return []
            # 이미 취소·오류·idle인데 늦게 켜졌다 → 다시 끈다
            self.log.append(f'late follow enable in state {self.state}: disabling again')
            return self._request_disable(now)
        self._disable_pending = False
        if ok:
            if self.state == 'cancelling' and self._disable_ok_at is None:
                self._disable_ok_at = now
            return []
        self.log.append(f'follow disable failed: {detail}')
        if self.state == 'cancelling':
            self._next_disable_at = now + self.cfg.disable_retry_s
        return []

    def odom(self, now: float, v: float, w: float) -> None:
        self._odom_at = now
        still = abs(v) < self.cfg.stop_linear_eps and abs(w) < self.cfg.stop_angular_eps
        if not still:
            self._still_since = None
        elif self._still_since is None:
            self._still_since = now

    def marker(self, now: float, status: int) -> None:
        self._marker, self._marker_at = status, now
        if status in (MARKER_TRACKING, MARKER_HOLDING) and self.state in ('starting', 'active'):
            self._acquired = True

    def tick(self, now: float) -> list:
        c = self.cfg
        if self.state == 'starting' and now - self._entered > c.enable_timeout_s:
            self.log.append('follow enable timed out')
            self._fail(now, MSG_ENABLE_FAILED)
            return self._request_disable(now)
        if self.state == 'cancelling':
            if self._disable_ok_at is not None and self._stopped(now):
                self._set('cancelled', now)
                return []
            if now - self._entered > c.stop_timeout_s:
                self._fail(now, MSG_STOP_UNCONFIRMED)
                return []
            if self._disable_ok_at is None and not self._disable_pending and self._next_disable_at is not None \
                    and now >= self._next_disable_at:
                return self._request_disable(now)
        if self.state == 'cancelled' and now - self._entered > c.terminal_hold_s:
            self.trip_id = self.from_place_id = self.to_place_id = None
            self._set('idle', now)
        return []

    # ------------------------------------------------------------ 출력

    def tracking_state(self, now: float) -> str:
        if self._marker is None or now - self._marker_at > self.cfg.marker_stale_s:
            return 'unavailable'
        if self._marker in (MARKER_TRACKING, MARKER_HOLDING):
            return 'following'
        if self.state == 'starting' or (self.state == 'active' and not self._acquired):
            return 'searching'
        return 'lost'

    def status(self, now: float) -> dict:
        return {
            'mission_state': self.state,
            'tracking_state': self.tracking_state(now),
            'trip_id': self.trip_id,
            'from_place_id': self.from_place_id,
            'to_place_id': self.to_place_id,
            'last_error': self.last_error,
        }

    # ------------------------------------------------------------ 내부

    def _set(self, state: str, now: float) -> None:
        if state != self.state:
            self.log.append(f'{self.state} -> {state} (trip {self.trip_id})')
        self.state = state
        self._entered = now

    def _fail(self, now: float, message: str) -> None:
        self.last_error = message
        self._set('error', now)

    def _request_disable(self, now: float) -> list:
        self._disable_pending = True
        self._next_disable_at = None
        return [('enable', False)]

    def _stopped(self, now: float) -> bool:
        c = self.cfg
        odom_fresh = self._odom_at is not None and now - self._odom_at <= c.odom_stale_s
        if odom_fresh:
            return self._still_since is not None and now - self._still_since >= c.stop_hold_s
        if now - self._disable_ok_at >= c.stop_fallback_s:
            self.log.append('no /odom: stop judged by time after follow disable')
            return True
        return False
