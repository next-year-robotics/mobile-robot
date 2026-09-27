"""중복 id 캐시(PROTOCOL.md 8.1).

같은 `id`가 다시 오면 side effect를 반복하지 않고 처음 보낸 응답 frame을 그대로 다시 보낸다.
성공(`ack`)만 저장한다. `error`는 side effect가 없으므로 다시 계산하게 둔다
(예: NOT_READY 뒤 같은 id로 재시도하면 이번에는 처리될 수 있어야 한다).
캐시는 세션이 아니라 서버 전체 범위다. 재연결 후 같은 명령이 다시 와도 한 번만 실행된다.
"""
from __future__ import annotations

import time
from collections import OrderedDict
from typing import Callable

DEFAULT_TTL_S = 600.0


class DedupeCache:

    def __init__(self, ttl_s: float = DEFAULT_TTL_S, max_entries: int = 1024,
                 clock: Callable[[], float] = time.monotonic):
        self.ttl_s = ttl_s
        self.max_entries = max_entries
        self._clock = clock
        self._items: OrderedDict[str, tuple[float, tuple[str, ...]]] = OrderedDict()

    def get(self, msg_id: str) -> tuple[str, ...] | None:
        self._expire()
        item = self._items.get(msg_id)
        return item[1] if item else None

    def put(self, msg_id: str, frames: list[str] | tuple[str, ...]) -> None:
        self._expire()
        self._items[msg_id] = (self._clock() + self.ttl_s, tuple(frames))
        self._items.move_to_end(msg_id)
        while len(self._items) > self.max_entries:
            self._items.popitem(last=False)

    def __len__(self) -> int:
        self._expire()
        return len(self._items)

    def _expire(self) -> None:
        now = self._clock()
        while self._items:
            key, (expires, _) = next(iter(self._items.items()))
            if expires > now:
                break
            del self._items[key]
