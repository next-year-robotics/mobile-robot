"""SOS 전달·재시도·ACK/위치 수집(PROTOCOL.md 10.3–10.6, 7.2).

- `request()`: 활성 SOS를 만들고 앱에 sos.request를 보낸다. 같은 sos_id가 이미 활성이면 무시한다.
- ACK가 `retry_interval_s` 안에 없으면 같은 sos_id로 최대 `max_retries`회 재전송한다(새 envelope id).
  앱이 연결돼 있지 않아도 시도 횟수는 센다. 그래야 앱이 없을 때도 약 8초 뒤 실패가 확정된다.
- 마지막 재전송 뒤에도 ACK가 없으면 `app_received=false`로 SosAck를 한 번 발행한다(로컬 폴백 유발).
- ACK를 받으면 SosAck(app_received=true)를 발행하고, 이후 위치 품질이 좋아질 때마다(none→cache→fresh) 다시 발행한다.
  ACK 전에 온 위치는 기록만 한다. app_received=false가 실패 신호이기 때문이다.
- 재연결(새 세션)하면 활성 SOS마다 sos.request를 다시 보내 앱 상태를 복원한다.
- `clear()`: 활성 SOS를 지우고 sos.clear를 보낸다. 앱이 연결돼 있지 않으면 다음 세션에 전달한다.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Awaitable, Callable

from ..protocol.envelope import make, now_ms

log = logging.getLogger(__name__)

SOURCE_RANK = {'none': 0, 'cache': 1, 'fresh': 2}
MAX_PENDING_CLEARS = 16


@dataclass
class ActiveSos:
    sos_id: str
    reason: str
    severity: str
    trip_id: str | None
    started_ms: int
    sends: int = 0                  # 시도 횟수(첫 전송 포함). retries = sends - 1
    next_retry_at: float | None = None
    acked: bool = False
    app_received: bool = False
    failure_published: bool = False
    has_location: bool = False
    best_source: str = 'none'

    @property
    def retries(self) -> int:
        return max(0, self.sends - 1)

    def request_payload(self) -> dict:
        return {'sos_id': self.sos_id, 'reason': self.reason, 'severity': self.severity,
                'trip_id': self.trip_id}


class SosManager:

    def __init__(self, *, send: Callable[[dict], Awaitable[bool]], connected: Callable[[], bool],
                 publish_ack: Callable[..., None], retry_interval_s: float = 2.0, max_retries: int = 3,
                 clock: Callable[[], float] = time.monotonic):
        self._send = send
        self._connected = connected
        self._publish_ack = publish_ack
        self.retry_interval_s = retry_interval_s
        self.max_retries = max_retries
        self._clock = clock
        self.active: dict[str, ActiveSos] = {}
        self.pending_clears: dict[str, dict] = {}

    # ------------------------------------------------------------ 로봇 → 앱

    async def request(self, sos_id: str, reason: str, severity: str, trip_id: str | None) -> bool:
        if sos_id in self.active:
            log.info('SOS %s already active; ignoring duplicate request', sos_id)
            return False
        self.pending_clears.pop(sos_id, None)
        sos = ActiveSos(sos_id=sos_id, reason=reason, severity=severity, trip_id=trip_id, started_ms=now_ms())
        self.active[sos_id] = sos
        log.warning('SOS %s requested: reason=%s severity=%s', sos_id, reason, severity)
        await self._attempt(sos)
        return True

    async def resend(self, sos_id: str) -> bool:
        """재시도 규칙과 무관하게 한 번 더 보낸다(mock sos-x3). 시도 횟수는 세지 않는다."""
        sos = self.active.get(sos_id)
        if sos is None or not self._connected():
            return False
        return await self._send(make('sos.request', sos.request_payload()))

    async def clear(self, sos_id: str, resolution: str) -> None:
        was_active = self.active.pop(sos_id, None) is not None
        msg = make('sos.clear', {'sos_id': sos_id, 'resolution': resolution})
        log.info('SOS %s cleared: %s (active=%s)', sos_id, resolution, was_active)
        if self._connected() and await self._send(msg):
            return
        if was_active:
            self.pending_clears[sos_id] = {'sos_id': sos_id, 'resolution': resolution}
            while len(self.pending_clears) > MAX_PENDING_CLEARS:
                self.pending_clears.pop(next(iter(self.pending_clears)))

    async def on_session_open(self) -> None:
        """새 세션: 전달 못 한 sos.clear, 그다음 활성 SOS 복원."""
        for sos_id, payload in list(self.pending_clears.items()):
            if await self._send(make('sos.clear', payload)):
                self.pending_clears.pop(sos_id, None)
        for sos in list(self.active.values()):
            await self._send(make('sos.request', sos.request_payload()))
            if not sos.acked and not sos.failure_published:
                sos.next_retry_at = self._clock() + self.retry_interval_s

    async def tick(self) -> None:
        now = self._clock()
        for sos in list(self.active.values()):
            if sos.acked or sos.next_retry_at is None or now < sos.next_retry_at:
                continue
            if sos.retries < self.max_retries:
                await self._attempt(sos)
            else:
                sos.next_retry_at = None
                sos.failure_published = True
                log.error('SOS %s: no ACK after %d retries', sos.sos_id, sos.retries)
                self._emit(sos, app_received=False)

    async def _attempt(self, sos: ActiveSos) -> None:
        sos.sends += 1
        sos.next_retry_at = self._clock() + self.retry_interval_s
        if self._connected():
            await self._send(make('sos.request', sos.request_payload()))

    # ------------------------------------------------------------ 앱 → 로봇

    def on_ack(self, sos_id: str, received: bool) -> bool:
        sos = self.active.get(sos_id)
        if sos is None:
            return False
        first = not sos.acked
        sos.acked = True
        sos.app_received = bool(received)
        sos.next_retry_at = None
        if first:
            self._emit(sos, app_received=sos.app_received)
        return True

    def on_location(self, sos_id: str | None, source: str) -> None:
        sos = self.active.get(sos_id) if sos_id else None
        if sos is None:
            return
        improved = SOURCE_RANK.get(source, 0) > SOURCE_RANK[sos.best_source]
        if source in ('cache', 'fresh'):
            sos.has_location = True
        if improved:
            sos.best_source = source
            if sos.acked:
                self._emit(sos, app_received=sos.app_received)

    def _emit(self, sos: ActiveSos, *, app_received: bool) -> None:
        try:
            self._publish_ack(sos_id=sos.sos_id, app_received=app_received, retries=sos.retries,
                              has_location=sos.has_location, best_source=sos.best_source)
        except Exception:
            log.exception('publish_sos_ack failed')
