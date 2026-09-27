"""통합 테스트용 앱 흉내 클라이언트와 브리지 하네스. websockets가 있어야 import된다."""
from __future__ import annotations

import asyncio
import json
import time
import uuid
from contextlib import asynccontextmanager
from dataclasses import dataclass
from pathlib import Path

import websockets
from websockets.exceptions import ConnectionClosed

from mr_phone_bridge.mock.backend import MockBackend
from mr_phone_bridge.mock.scenarios import seed_places, write_seed
from mr_phone_bridge.places.store import PlaceStore
from mr_phone_bridge.protocol.envelope import check_outgoing
from mr_phone_bridge.server.bridge import BridgeConfig, BridgeCore
from mr_phone_bridge.server.ws import BridgeServer

FAST = dict(heartbeat_timeout_ms=1000, hello_timeout_s=0.6, status_period_s=0.4, link_status_period_s=0.5,
            sos_retry_interval_s=0.2, tick_s=0.02, close_timeout_s=1.0, pending_trip_timeout_s=1.0)


def uid() -> str:
    return str(uuid.uuid4())


class Client:
    """앱 역할. strict면 받은 메시지를 모두 계약 스키마로 검사한다."""

    def __init__(self, ws, strict: bool = True):
        self.ws = ws
        self.strict = strict
        self.backlog: list[dict] = []
        self.session_id: str | None = None
        self.catalog: dict | None = None

    @classmethod
    async def connect(cls, url: str, strict: bool = True) -> 'Client':
        return cls(await websockets.connect(url), strict)

    async def send(self, mtype: str, payload: dict, *, reply_to: str | None = None,
                   msg_id: str | None = None) -> str:
        msg_id = msg_id or uid()
        await self.ws.send(json.dumps({'version': 1, 'type': mtype, 'id': msg_id,
                                       'sent_at_ms': int(time.time() * 1000), 'reply_to': reply_to,
                                       'payload': payload}, ensure_ascii=False))
        return msg_id

    async def send_raw(self, data) -> None:
        await self.ws.send(data)

    async def _recv_one(self, timeout: float) -> dict:
        raw = await asyncio.wait_for(self.ws.recv(), timeout)
        assert isinstance(raw, str), 'server must send text frames'
        msg = json.loads(raw)
        if self.strict:
            err = check_outgoing(msg)
            assert err is None, f'server message violates contract: {err}: {msg}'
        if msg['type'] == 'place.catalog':
            self.catalog = msg['payload']
        return msg

    async def recv(self, timeout: float = 2.0) -> dict:
        if self.backlog:
            return self.backlog.pop(0)
        return await self._recv_one(timeout)

    async def expect(self, mtype: str, *, reply_to: str | None = None, timeout: float = 2.0,
                     where=None) -> dict:
        """조건에 맞는 메시지가 올 때까지 기다린다. 건너뛴 메시지는 backlog에 남긴다."""
        for i, m in enumerate(self.backlog):
            if _match(m, mtype, reply_to, where):
                return self.backlog.pop(i)
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                raise AssertionError(f'no {mtype} (reply_to={reply_to}); backlog={[m["type"] for m in self.backlog]}')
            try:
                m = await self._recv_one(left)
            except asyncio.TimeoutError:
                continue
            if _match(m, mtype, reply_to, where):
                return m
            self.backlog.append(m)

    async def reply(self, request_id: str, timeout: float = 2.0) -> dict:
        """request_id에 대한 ack 또는 error."""
        for i, m in enumerate(self.backlog):
            if m.get('reply_to') == request_id:
                return self.backlog.pop(i)
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                raise AssertionError(f'no reply to {request_id}')
            try:
                m = await self._recv_one(left)
            except asyncio.TimeoutError:
                continue
            if m.get('reply_to') == request_id:
                return m
            self.backlog.append(m)

    async def hello(self) -> dict:
        hid = await self.send('hello', {'app_instance_id': uid(), 'app_version': '0.1.0', 'platform': 'android',
                                        'capabilities': ['location.cache', 'location.fresh', 'place.edit']})
        ack = await self.expect('hello.ack', reply_to=hid)
        self.session_id = ack['payload']['session_id']
        await self.expect('place.catalog')
        await self.expect('robot.status')
        return ack

    async def heartbeat(self) -> dict:
        hid = await self.send('heartbeat', {'session_id': self.session_id})
        return await self.reply(hid)

    async def closed(self, timeout: float = 3.0) -> int | None:
        """서버가 닫을 때까지 읽고 닫기 코드를 돌려준다."""
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline:
                await asyncio.wait_for(self.ws.recv(), max(0.01, deadline - time.monotonic()))
        except ConnectionClosed:
            return self.ws.close_code
        except asyncio.TimeoutError:
            pass
        raise AssertionError('connection still open')

    async def close(self) -> None:
        await self.ws.close()


def _match(m: dict, mtype: str, reply_to, where) -> bool:
    if m.get('type') != mtype:
        return False
    if reply_to is not None and m.get('reply_to') != reply_to:
        return False
    return where is None or where(m)


@dataclass
class Harness:
    core: BridgeCore
    backend: MockBackend
    server: BridgeServer
    places_path: Path

    @property
    def url(self) -> str:
        return f'ws://127.0.0.1:{self.server.port}/bridge/v1'

    async def client(self, strict: bool = True) -> Client:
        return await Client.connect(self.url, strict)

    def published(self, topic: str) -> list[dict]:
        return [d for t, d in self.backend.published if t == topic]


@asynccontextmanager
async def bridge(tmp_path: Path, *, scenario: str = 'default6', hooks=None, backend=None, **overrides):
    places_path = tmp_path / 'places.yaml'
    if not places_path.exists():
        write_seed(places_path, *seed_places(scenario))
    store = PlaceStore(places_path, max_places=overrides.pop('max_places', 12))
    backend = backend or MockBackend(start_delay_s=0.1, cancel_delay_s=0.1, terminal_hold_s=0.3)
    cfg = BridgeConfig(**{**FAST, **overrides})
    core = BridgeCore(cfg, store, backend, hooks)
    server = BridgeServer(core, '127.0.0.1', 0)
    await server.start()
    try:
        yield Harness(core, backend, server, places_path)
    finally:
        await server.stop()


def place_payload(revision: str, **kw) -> dict:
    p = {'place_id': None, 'catalog_revision': revision, 'kind': 'hospital', 'label': '병원', 'lat': 36.13,
         'lon': 128.57, 'radius_m': 30, 'enabled': True, 'sort_order': 70, 'is_default_origin': False,
         'address': None, 'source': 'map_pin'}
    p.update(kw)
    return p


def location_payload(reason: str = 'sos', sos_id: str | None = None, source: str = 'cache', **kw) -> dict:
    if source == 'none':
        p = {'reason': reason, 'sos_id': sos_id, 'has_fix': False, 'lat': None, 'lon': None,
             'horizontal_accuracy_m': None, 'captured_at_ms': None, 'age_ms': None, 'source': 'none',
             'failure_reason': 'timeout'}
    else:
        p = {'reason': reason, 'sos_id': sos_id, 'has_fix': True, 'lat': 36.1234, 'lon': 128.5678,
             'horizontal_accuracy_m': 4.2, 'captured_at_ms': int(time.time() * 1000) - 800, 'age_ms': 800,
             'source': source, 'failure_reason': None}
    p.update(kw)
    return p
