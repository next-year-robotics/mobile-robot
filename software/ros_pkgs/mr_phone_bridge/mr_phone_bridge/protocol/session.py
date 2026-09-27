"""연결 하나의 세션 상태(PROTOCOL.md 8.2, 8.4).

AWAITING_HELLO ──hello──▶ ACTIVE ──close/timeout/supersede──▶ CLOSED

- 연결 후 `hello_timeout_s` 안에 hello가 없으면 닫는다(앱은 3초 안에 보낸다).
- ACTIVE에서 유효한 앱 메시지가 `heartbeat_timeout_ms` 동안 없으면 닫는다.
- handshake 전에는 hello 외의 메시지를 처리하지 않는다.
시각은 모두 monotonic. `sent_at_ms`는 쓰지 않는다.
"""
from __future__ import annotations

import enum
from dataclasses import dataclass, field

from .envelope import Message, make, new_id, now_ms
from .errors import INVALID_STATE, ProtocolError


class State(enum.Enum):
    AWAITING_HELLO = 'awaiting_hello'
    ACTIVE = 'active'
    CLOSED = 'closed'


@dataclass(frozen=True)
class ServerInfo:
    """hello.ack에 실을 서버 정보."""
    robot_id: str
    server_version: str
    heartbeat_interval_ms: int = 2000
    heartbeat_timeout_ms: int = 10000
    place_edit_enabled: bool = True
    max_places: int = 12


@dataclass
class Session:
    opened_at: float
    state: State = State.AWAITING_HELLO
    session_id: str | None = None
    app_instance_id: str | None = None
    app_version: str | None = None
    capabilities: tuple[str, ...] = ()
    last_seen: float = 0.0          # monotonic
    last_seen_ms: int = 0           # wall, 기록·ROS 발행용
    closed_reason: str | None = None
    extra: dict = field(default_factory=dict)   # mock 시나리오 등 부가 상태

    def __post_init__(self):
        self.last_seen = self.opened_at

    @property
    def active(self) -> bool:
        return self.state is State.ACTIVE

    def touch(self, now: float) -> None:
        self.last_seen = now
        self.last_seen_ms = now_ms()

    def accept_hello(self, msg: Message, info: ServerInfo, catalog_revision: str) -> dict:
        """hello를 받아 ACTIVE로 전이하고 hello.ack를 돌려준다."""
        if self.state is not State.AWAITING_HELLO:
            raise ProtocolError(INVALID_STATE, 'session already established; reconnect to start a new one')
        p = msg.payload
        self.session_id = new_id()
        self.app_instance_id = p['app_instance_id']
        self.app_version = p['app_version']
        self.capabilities = tuple(p['capabilities'])
        self.state = State.ACTIVE
        return make('hello.ack', {
            'session_id': self.session_id,
            'robot_id': info.robot_id,
            'server_version': info.server_version,
            'heartbeat_interval_ms': info.heartbeat_interval_ms,
            'heartbeat_timeout_ms': info.heartbeat_timeout_ms,
            'catalog_revision': catalog_revision,
            'place_edit_enabled': info.place_edit_enabled,
            'max_places': info.max_places,
        }, reply_to=msg.id)

    def accept_heartbeat(self, msg: Message) -> dict:
        if msg.payload['session_id'] != self.session_id:
            raise ProtocolError(INVALID_STATE, 'session_id does not match this connection')
        return make('heartbeat.ack', {'session_id': self.session_id}, reply_to=msg.id)

    def require_active(self) -> None:
        if not self.active:
            raise ProtocolError(INVALID_STATE, 'send hello first')

    def hello_expired(self, now: float, hello_timeout_s: float) -> bool:
        return self.state is State.AWAITING_HELLO and now - self.opened_at > hello_timeout_s

    def heartbeat_expired(self, now: float, heartbeat_timeout_ms: int) -> bool:
        return self.active and (now - self.last_seen) * 1000.0 > heartbeat_timeout_ms

    def close(self, reason: str) -> None:
        if self.state is not State.CLOSED:
            self.state = State.CLOSED
            self.closed_reason = reason
