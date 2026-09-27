"""phone-bridge/1 서버 코어. 전송 계층(websockets)과 로봇 쪽(Backend)을 모두 모른다.

연결 하나마다 `handle(conn)`이 돈다. 앱 세션은 동시에 하나(MVP)이며, 새 hello가 성공하면 이전 세션을
닫는다(휴대폰이 재연결할 때 반쯤 열린 이전 소켓이 자리를 막지 않도록).

주기 작업(0.1 s 틱): hello/heartbeat 타임아웃, robot.status 2 s 주기, 링크 상태 재발행, SOS 재시도.
모든 코루틴은 한 asyncio 루프에서 돈다. ROS 스레드는 `call_threadsafe()`로 들어온다.
"""
from __future__ import annotations

import asyncio
import logging
import math
import time
from dataclasses import dataclass
from typing import Any, AsyncIterator, Awaitable, Callable, Protocol

from .. import __version__
from ..places.ops import check_revision, plan_delete, plan_upsert
from ..places.store import PLACE_ID_RE, PlaceStore, PlaceStoreError, PlaceValidationError
from ..protocol.dedupe import DedupeCache
from ..protocol.envelope import Message, check_outgoing, encode, is_uuid, make, make_error, new_id, parse_frame
from ..protocol.errors import (CATALOG_CONFLICT, INTERNAL_ERROR, INVALID_MESSAGE, INVALID_STATE, NOT_READY,
                               UNKNOWN_PLACE, ProtocolError)
from ..protocol.session import ServerInfo, Session, State
from .backend import (ACTIVE_MISSION_STATES, BRIDGE_STATES, MISSION_STATES, STARTABLE_MISSION_STATES,
                      TRACKING_STATES, Backend, MissionSnapshot)
from .sos import SosManager

log = logging.getLogger(__name__)

# 닫기 코드. 앱은 코드와 상관없이 재연결하면 된다.
CLOSE_GOING_AWAY = 1001
CLOSE_POLICY = 1008
CLOSE_TOO_BIG = 1009
CLOSE_INTERNAL = 1011
CLOSE_SUPERSEDED = 4000
CLOSE_HEARTBEAT_TIMEOUT = 4001

SOS_REASONS = ('tracking_lost', 'stuck', 'low_battery', 'manual', 'unknown')
SOS_SEVERITIES = ('critical', 'warning')
SOS_RESOLUTIONS = ('tracking_recovered', 'handled', 'cancelled', 'unknown')
LOW_CONFIDENCE_AGE_MS = 300_000
LOW_CONFIDENCE_ACCURACY_M = 50.0


@dataclass
class BridgeConfig:
    robot_id: str = 'mr-001'
    path: str = '/bridge/v1'
    place_edit_enabled: bool = True
    heartbeat_interval_ms: int = 2000
    heartbeat_timeout_ms: int = 10000
    hello_timeout_s: float = 5.0
    status_period_s: float = 2.0
    link_status_period_s: float = 5.0
    sos_retry_interval_s: float = 2.0
    sos_max_retries: int = 3
    dedupe_ttl_s: float = 600.0
    max_frame_bytes: int = 16 * 1024
    rate_limit_per_s: float = 20.0
    rate_limit_burst: int = 40
    tick_s: float = 0.1
    pending_trip_timeout_s: float = 5.0
    close_timeout_s: float = 3.0
    validate_outgoing: bool = True


class Connection(Protocol):
    """전송 계층 어댑터(server/ws.py). 받은 frame을 async iterator로 내놓는다."""
    path: str
    remote: str

    async def send(self, text: str) -> None: ...
    async def close(self, code: int, reason: str) -> None: ...
    def abort(self) -> None: ...
    def __aiter__(self) -> AsyncIterator[str | bytes]: ...


class Hooks:
    """mock 시나리오 확장점. 실제 브리지는 기본값을 쓴다."""

    async def on_session_open(self, core: 'BridgeCore', peer: 'Peer') -> None:
        pass

    def on_session_close(self, core: 'BridgeCore', peer: 'Peer') -> None:
        pass

    async def before_command(self, core: 'BridgeCore', peer: 'Peer', msg: Message) -> None:
        """명령 처리 직전. sleep(지연) 하거나 ProtocolError를 올려(강제 오류) 결과를 바꿀 수 있다."""


@dataclass
class PendingTrip:
    trip_id: str
    from_place_id: str
    to_place_id: str
    issued_at: float


class Peer:
    """연결 하나."""

    def __init__(self, conn: Connection, now: float, cfg: BridgeConfig):
        self.conn = conn
        self.session = Session(opened_at=now)
        self.closing = False
        self.superseded = False
        self.silent = False            # mock flaky: 아무것도 보내지도 받지도 않는다
        self._tokens = float(cfg.rate_limit_burst)
        self._token_ts = now
        self._rate = cfg.rate_limit_per_s
        self._burst = float(cfg.rate_limit_burst)
        self.send_lock = asyncio.Lock()
        self.tasks: set[asyncio.Task] = set()
        self.close_task: asyncio.Task | None = None

    def take_token(self, now: float) -> bool:
        self._tokens = min(self._burst, self._tokens + (now - self._token_ts) * self._rate)
        self._token_ts = now
        if self._tokens < 1.0:
            return False
        self._tokens -= 1.0
        return True

    def spawn(self, coro: Awaitable) -> asyncio.Task:
        task = asyncio.ensure_future(coro)
        self.tasks.add(task)
        task.add_done_callback(self.tasks.discard)
        return task


class BridgeCore:

    def __init__(self, config: BridgeConfig, store: PlaceStore, backend: Backend | None = None,
                 hooks: Hooks | None = None, clock: Callable[[], float] = time.monotonic):
        self.cfg = config
        self.store = store
        self.backend = backend or Backend()
        self.hooks = hooks or Hooks()
        self.clock = clock
        self.backend.attach(self)
        self.dedupe = DedupeCache(config.dedupe_ttl_s, clock=clock)
        self.sos = SosManager(send=self._send_active, connected=lambda: self.active is not None,
                              publish_ack=self.backend.publish_sos_ack,
                              retry_interval_s=config.sos_retry_interval_s,
                              max_retries=config.sos_max_retries, clock=clock)
        self.active: Peer | None = None
        self.peers: set[Peer] = set()
        self.loop: asyncio.AbstractEventLoop | None = None
        self._pending_trip: PendingTrip | None = None
        self._last_status_payload: dict | None = None
        self._last_status_sent_at = 0.0
        self._last_link_publish = 0.0
        self._ticker: asyncio.Task | None = None
        self._bg: set[asyncio.Task] = set()
        self._handlers = {
            'trip.start': self._trip_start,
            'trip.cancel': self._trip_cancel,
            'place.upsert': self._place_upsert,
            'place.delete': self._place_delete,
            'location.update': self._location_update,
            'sos.ack': self._sos_ack,
        }

    # ============================================================ 수명

    async def start(self) -> None:
        self.loop = asyncio.get_running_loop()
        cat = self.store.catalog
        if self.store.load_warning:
            log.error('places: %s', self.store.load_warning)
        log.info('places: %d place(s), revision %s (%s)', len(cat.places), cat.revision, self.store.path)
        self._safe(self.backend.publish_catalog, cat)
        self._publish_link(False)
        self._ticker = asyncio.ensure_future(self._tick_loop())

    async def stop(self) -> None:
        for peer in list(self.peers):
            self.close_peer(peer, CLOSE_GOING_AWAY, 'server shutdown')
        if self._bg:
            await asyncio.wait(list(self._bg), timeout=self.cfg.close_timeout_s + 1.0)
        if self._ticker:
            self._ticker.cancel()
            try:
                await self._ticker
            except asyncio.CancelledError:
                pass

    def call_threadsafe(self, fn: Callable[..., Any], *args, **kwargs) -> None:
        """다른 스레드(ROS executor)에서 코어 메서드를 부른다. 코루틴 함수면 태스크로 돌린다."""
        if self.loop is None or self.loop.is_closed():
            return

        def run():
            res = fn(*args, **kwargs)
            if asyncio.iscoroutine(res):
                self._spawn_bg(res)
        self.loop.call_soon_threadsafe(run)

    # ============================================================ 로봇 → 앱

    async def notify_status(self) -> None:
        """로봇 상태가 바뀌었을 수 있다. 달라졌으면 바로 보낸다."""
        await self._send_status(force=False)

    async def request_sos(self, sos_id: str, reason: str, severity: str, trip_id: str | None) -> bool:
        if not is_uuid(sos_id):
            raise ValueError(f'sos_id must be a lowercase UUID v4: {sos_id!r}')
        reason = reason if reason in SOS_REASONS else 'unknown'
        severity = severity if severity in SOS_SEVERITIES else 'critical'
        trip_id = trip_id if is_uuid(trip_id) else None
        return await self.sos.request(sos_id, reason, severity, trip_id)

    async def clear_sos(self, sos_id: str, resolution: str) -> None:
        if not is_uuid(sos_id):
            raise ValueError(f'sos_id must be a lowercase UUID v4: {sos_id!r}')
        await self.sos.clear(sos_id, resolution if resolution in SOS_RESOLUTIONS else 'unknown')

    async def broadcast_catalog(self) -> None:
        cat = self.store.catalog
        if self.active is not None:
            await self._send(self.active, make('place.catalog', cat.to_payload()))
        self._safe(self.backend.publish_catalog, cat)

    def status_payload(self) -> dict:
        snap = self._snapshot()
        battery = snap.battery_percent
        if not isinstance(battery, (int, float)) or isinstance(battery, bool) \
                or not math.isfinite(battery) or not 0 <= battery <= 100:
            battery = None
        return {
            'robot_id': self.cfg.robot_id,
            'bridge_state': snap.bridge_state if snap.bridge_state in BRIDGE_STATES else 'degraded',
            'mission_state': snap.mission_state if snap.mission_state in MISSION_STATES else 'unknown',
            'tracking_state': snap.tracking_state if snap.tracking_state in TRACKING_STATES else 'unknown',
            'trip_id': snap.trip_id if is_uuid(snap.trip_id) else None,
            'from_place_id': _place_id_or_none(snap.from_place_id),
            'to_place_id': _place_id_or_none(snap.to_place_id),
            'battery_percent': battery,
            'last_error': snap.last_error or None,
        }

    # ============================================================ 연결

    async def handle(self, conn: Connection) -> None:
        peer = Peer(conn, self.clock(), self.cfg)
        path = (conn.path or '').split('?', 1)[0]
        if path != self.cfg.path:
            log.info('reject %s: unknown path %r', conn.remote, conn.path)
            await self._close_conn(peer, CLOSE_POLICY, f'unknown path; use {self.cfg.path}')
            return
        self.peers.add(peer)
        log.info('connection from %s', conn.remote)
        try:
            async for data in conn:
                if peer.closing:
                    break
                if peer.silent:
                    continue
                if not peer.take_token(self.clock()):
                    log.warning('rate limit exceeded by %s', conn.remote)
                    self.close_peer(peer, CLOSE_POLICY, 'rate limit exceeded')
                    break
                await self._on_frame(peer, data)
        except Exception:
            log.exception('connection handler failed')
        finally:
            self._deactivate(peer, 'connection closed')
            self.peers.discard(peer)
            for task in list(peer.tasks):
                task.cancel()
            if peer.close_task is not None:
                # 핸들러가 먼저 끝나면 websockets가 1000으로 닫아 버린다. 우리 닫기 코드가 나가게 기다린다.
                await asyncio.wait([peer.close_task], timeout=self.cfg.close_timeout_s + 0.5)
            log.info('connection from %s closed', conn.remote)

    def close_peer(self, peer: Peer, code: int, reason: str) -> None:
        """세션을 즉시 비활성으로 돌리고(링크 끊김 발행) 닫기 핸드셰이크는 뒤에서 한다."""
        if peer.closing:
            return
        peer.closing = True
        self._deactivate(peer, reason)
        peer.close_task = self._spawn_bg(self._close_conn(peer, code, reason))

    def abort_peer(self, peer: Peer) -> None:
        """close frame 없이 TCP를 끊는다(mock flaky)."""
        peer.closing = True
        self._deactivate(peer, 'aborted')
        peer.conn.abort()

    async def _close_conn(self, peer: Peer, code: int, reason: str) -> None:
        try:
            await asyncio.wait_for(peer.conn.close(code, reason[:120]), self.cfg.close_timeout_s)
        except Exception:
            peer.conn.abort()

    def _deactivate(self, peer: Peer, reason: str, *, publish: bool = True) -> None:
        if peer.session.state is State.CLOSED:
            return
        was_active = peer.session.active
        peer.session.close(reason)
        if was_active and self.active is peer:
            self.active = None
            log.info('session %s closed: %s', peer.session.session_id, reason)
            if publish:
                self._publish_link(False, peer)
            try:
                self.hooks.on_session_close(self, peer)
            except Exception:
                log.exception('on_session_close hook failed')

    # ============================================================ 수신

    async def _on_frame(self, peer: Peer, data: str | bytes) -> None:
        try:
            msg = parse_frame(data, self.cfg.max_frame_bytes)
        except ProtocolError as e:
            log.info('rejected frame from %s: %s %s', peer.conn.remote, e.code, e.message)
            await self._send(peer, make_error(e))
            if e.close:
                self.close_peer(peer, CLOSE_TOO_BIG if e.code == INVALID_MESSAGE else CLOSE_POLICY, e.message)
            return
        peer.session.touch(self.clock())
        log.debug('<- %s %s', msg.type, msg.id)
        if msg.type == 'error':
            log.warning('app reported error: %s', msg.payload)
            return
        if msg.type == 'hello':
            await self._on_hello(peer, msg)
            return
        if not peer.session.active:
            await self._send(peer, make_error(ProtocolError(INVALID_STATE, 'send hello first'), msg.id))
            return
        if msg.type == 'heartbeat':
            try:
                reply = peer.session.accept_heartbeat(msg)
            except ProtocolError as e:
                reply = make_error(e, msg.id)
            await self._send(peer, reply)
            return
        await self._on_command(peer, msg)

    async def _on_hello(self, peer: Peer, msg: Message) -> None:
        info = ServerInfo(robot_id=self.cfg.robot_id, server_version=__version__,
                          heartbeat_interval_ms=self.cfg.heartbeat_interval_ms,
                          heartbeat_timeout_ms=self.cfg.heartbeat_timeout_ms,
                          place_edit_enabled=self.cfg.place_edit_enabled,
                          max_places=self.store.max_places)
        try:
            ack = peer.session.accept_hello(msg, info, self.store.catalog.revision)
        except ProtocolError as e:
            await self._send(peer, make_error(e, msg.id))
            return
        old = self.active
        if old is not None and old is not peer:
            old.superseded = True
            self._deactivate(old, 'superseded', publish=False)
            old.closing = True
            old.close_task = self._spawn_bg(self._close_conn(old, CLOSE_SUPERSEDED, 'superseded by a new session'))
        self.active = peer
        s = peer.session
        log.info('session %s opened: app %s v%s from %s', s.session_id, s.app_instance_id, s.app_version,
                 peer.conn.remote)
        await self._send(peer, ack)
        await self._send(peer, make('place.catalog', self.store.catalog.to_payload()))
        await self._send_status(force=True)
        await self.sos.on_session_open()
        self._publish_link(True, peer)
        try:
            await self.hooks.on_session_open(self, peer)
        except Exception:
            log.exception('on_session_open hook failed')

    async def _on_command(self, peer: Peer, msg: Message) -> None:
        cached = self.dedupe.get(msg.id)
        if cached is not None:
            log.info('duplicate %s %s: resending previous result', msg.type, msg.id)
            for frame in cached:
                await self._send_text(peer, frame)
            return
        try:
            await self.hooks.before_command(self, peer, msg)
            replies, after = await self._handlers[msg.type](msg)
        except ProtocolError as e:
            log.info('%s %s rejected: %s %s', msg.type, msg.id, e.code, e.message)
            await self._send(peer, make_error(e, msg.id))
            if e.code == CATALOG_CONFLICT:
                await self._send(peer, make('place.catalog', self.store.catalog.to_payload()))
            return
        except Exception:
            log.exception('%s %s failed', msg.type, msg.id)
            await self._send(peer, make_error(ProtocolError(INTERNAL_ERROR, 'internal error'), msg.id))
            return
        frames = [self._encode(r) for r in replies]
        self.dedupe.put(msg.id, frames)
        for frame in frames:
            await self._send_text(peer, frame)
        if after is not None:
            await after()

    # ============================================================ 명령

    async def _trip_start(self, msg: Message):
        p = msg.payload
        cat = self.store.catalog
        check_revision(cat, p['catalog_revision'])
        if p['from_place_id'] == p['to_place_id']:
            raise ProtocolError(INVALID_MESSAGE, 'from_place_id and to_place_id must differ')
        for key, what in (('from_place_id', 'origin'), ('to_place_id', 'destination')):
            place = cat.places.get(p[key])
            if place is None:
                raise ProtocolError(UNKNOWN_PLACE, f'Unknown {what} place_id')
            if not place['enabled']:
                raise ProtocolError(INVALID_STATE, f'{what} place is disabled')
        mission, _ = self._effective_mission()
        if mission == 'unknown':
            raise ProtocolError(NOT_READY, 'mission controller is not available')
        if mission not in STARTABLE_MISSION_STATES:
            raise ProtocolError(INVALID_STATE, f'a trip is already {mission}')
        trip_id = new_id()
        self.backend.start_trip(request_id=msg.id, trip_id=trip_id, from_place_id=p['from_place_id'],
                                to_place_id=p['to_place_id'])
        self._pending_trip = PendingTrip(trip_id, p['from_place_id'], p['to_place_id'], self.clock())
        log.info('trip %s accepted: %s -> %s', trip_id, p['from_place_id'], p['to_place_id'])
        return [make('ack', {'status': 'accepted', 'trip_id': trip_id}, msg.id)], self.notify_status

    async def _trip_cancel(self, msg: Message):
        # 정지 요청은 가능한 한 거절하지 않는다. mission 상태를 모르면(unknown) 그대로 전달한다.
        p = msg.payload
        snap = self._snapshot()
        known = {snap.trip_id}
        if self._pending_trip is not None:
            known.add(self._pending_trip.trip_id)
        if snap.mission_state != 'unknown' and p['trip_id'] not in known:
            raise ProtocolError(INVALID_STATE, 'trip_id does not match the current trip')
        self.backend.cancel_trip(request_id=msg.id, trip_id=p['trip_id'], reason=p['reason'])
        if self._pending_trip is not None and self._pending_trip.trip_id == p['trip_id']:
            self._pending_trip = None
        log.info('trip %s cancel accepted (%s)', p['trip_id'], p['reason'])
        return [make('ack', {'status': 'accepted'}, msg.id)], self.notify_status

    async def _place_upsert(self, msg: Message):
        self._require_place_edit()
        places, pid = plan_upsert(self.store.catalog, msg.payload, self._places_in_use())
        cat = self._commit(places)
        log.info('place %s saved (revision %s)', pid, cat.revision)
        return ([make('ack', {'status': 'accepted', 'place_id': pid, 'catalog_revision': cat.revision}, msg.id)],
                self.broadcast_catalog)

    async def _place_delete(self, msg: Message):
        self._require_place_edit()
        places, pid = plan_delete(self.store.catalog, msg.payload, self._places_in_use())
        cat = self._commit(places)
        log.info('place %s deleted (revision %s)', pid, cat.revision)
        return ([make('ack', {'status': 'accepted', 'place_id': pid, 'catalog_revision': cat.revision}, msg.id)],
                self.broadcast_catalog)

    async def _location_update(self, msg: Message):
        p = msg.payload
        age, acc = p['age_ms'], p['horizontal_accuracy_m']
        low = bool(p['has_fix'] and ((age is not None and age > LOW_CONFIDENCE_AGE_MS)
                                     or (acc is not None and acc > LOW_CONFIDENCE_ACCURACY_M)))
        self.backend.publish_location(message_id=msg.id, payload=p, low_confidence=low)
        if p['reason'] == 'sos':
            self.sos.on_location(p['sos_id'], p['source'])
        return [make('ack', {'status': 'accepted'}, msg.id)], None

    async def _sos_ack(self, msg: Message):
        p = msg.payload
        if not self.sos.on_ack(p['sos_id'], p['received']):
            log.info('sos.ack for inactive SOS %s', p['sos_id'])
        return [make('ack', {'status': 'accepted'}, msg.id)], None

    def _require_place_edit(self) -> None:
        if not self.cfg.place_edit_enabled:
            raise ProtocolError(INVALID_STATE, 'place editing is disabled on this robot')

    def _commit(self, places: dict):
        try:
            return self.store.replace(places)
        except PlaceValidationError as e:
            raise ProtocolError(INVALID_MESSAGE, str(e)) from None
        except PlaceStoreError as e:
            log.error('places save failed: %s', e)
            raise ProtocolError(INTERNAL_ERROR, 'failed to save places') from None

    # ============================================================ 상태

    def _snapshot(self) -> MissionSnapshot:
        try:
            return self.backend.snapshot()
        except Exception:
            log.exception('backend snapshot failed')
            return MissionSnapshot(bridge_state='error', mission_state='unknown', tracking_state='unknown')

    def _effective_mission(self) -> tuple[str, MissionSnapshot]:
        """mission이 아직 반영하지 않은 방금 발급한 trip이 있으면 starting으로 본다."""
        snap = self._snapshot()
        pt = self._pending_trip
        if pt is not None and snap.trip_id != pt.trip_id and snap.mission_state not in ACTIVE_MISSION_STATES:
            return 'starting', snap
        return snap.mission_state, snap

    def _places_in_use(self) -> set[str]:
        snap = self._snapshot()
        used: set[str] = set()
        if snap.mission_state in ACTIVE_MISSION_STATES:
            used |= {snap.from_place_id, snap.to_place_id}
        if self._pending_trip is not None:
            used |= {self._pending_trip.from_place_id, self._pending_trip.to_place_id}
        used.discard(None)
        return used

    async def _send_status(self, force: bool) -> None:
        if self.active is None:
            return
        payload = self.status_payload()
        if not force and payload == self._last_status_payload:
            return
        self._last_status_payload = payload
        self._last_status_sent_at = self.clock()
        await self._send(self.active, make('robot.status', payload))

    def _publish_link(self, connected: bool, peer: Peer | None = None) -> None:
        s = peer.session if peer is not None else None
        self._last_link_publish = self.clock()
        self._safe(self.backend.publish_link_status, connected=connected,
                   app_instance_id=(s.app_instance_id or '') if s else '',
                   session_id=(s.session_id or '') if s else '',
                   last_seen_ms=s.last_seen_ms if s else 0)

    # ============================================================ 틱

    async def _tick_loop(self) -> None:
        while True:
            await asyncio.sleep(self.cfg.tick_s)
            try:
                await self._tick()
            except Exception:
                log.exception('tick failed')

    async def _tick(self) -> None:
        now = self.clock()
        for peer in list(self.peers):
            s = peer.session
            if peer.closing:
                continue
            if s.hello_expired(now, self.cfg.hello_timeout_s):
                log.info('no hello from %s; closing', peer.conn.remote)
                self.close_peer(peer, CLOSE_POLICY, 'hello timeout')
            elif s.heartbeat_expired(now, self.cfg.heartbeat_timeout_ms):
                log.warning('session %s heartbeat timeout', s.session_id)
                self.close_peer(peer, CLOSE_HEARTBEAT_TIMEOUT, 'heartbeat timeout')
        pt = self._pending_trip
        if pt is not None:
            snap = self._snapshot()
            if snap.trip_id == pt.trip_id or now - pt.issued_at > self.cfg.pending_trip_timeout_s:
                self._pending_trip = None
        if self.active is not None:
            if now - self._last_status_sent_at >= self.cfg.status_period_s:
                await self._send_status(force=True)
            else:
                await self._send_status(force=False)
            if now - self._last_link_publish >= self.cfg.link_status_period_s:
                self._publish_link(True, self.active)
        await self.sos.tick()

    # ============================================================ 송신

    def _encode(self, msg: dict) -> str:
        if self.cfg.validate_outgoing:
            err = check_outgoing(msg)
            if err:
                log.error('outgoing %s violates contract: %s', msg.get('type'), err)
        return encode(msg)

    async def _send(self, peer: Peer, msg: dict) -> bool:
        return await self._send_text(peer, self._encode(msg))

    async def send_raw(self, peer: Peer, data: str | bytes) -> bool:
        """계약 검사 없이 보낸다(mock garbage 전용)."""
        return await self._send_text(peer, data)

    async def _send_text(self, peer: Peer, text: str | bytes) -> bool:
        if peer.silent:
            return False
        async with peer.send_lock:
            try:
                await peer.conn.send(text)
                return True
            except Exception as e:
                log.debug('send to %s failed: %s', peer.conn.remote, e)
                return False

    async def _send_active(self, msg: dict) -> bool:
        peer = self.active
        if peer is None:
            return False
        return await self._send(peer, msg)

    # ============================================================ 기타

    def _spawn_bg(self, coro) -> asyncio.Task:
        task = asyncio.ensure_future(coro)
        self._bg.add(task)
        task.add_done_callback(self._bg_done)
        return task

    def _bg_done(self, task: asyncio.Task) -> None:
        self._bg.discard(task)
        if not task.cancelled() and task.exception() is not None:
            log.error('background task failed: %r', task.exception())

    @staticmethod
    def _safe(fn: Callable, *args, **kwargs) -> None:
        try:
            fn(*args, **kwargs)
        except Exception:
            log.exception('backend call %s failed', getattr(fn, '__name__', fn))


def _place_id_or_none(value: Any) -> str | None:
    return value if isinstance(value, str) and PLACE_ID_RE.match(value) else None
