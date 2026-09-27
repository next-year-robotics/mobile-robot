"""mock 시나리오(PROTOCOL.md 7.1). 서버 코어는 실제 브리지와 같고, 시나리오는 Hooks와 초기 카탈로그만 바꾼다."""
from __future__ import annotations

import asyncio
import json
import logging
import os
from dataclasses import dataclass
from pathlib import Path

import yaml

from ..places.store import FIELDS, FILE_VERSION
from ..protocol import contracts_path
from ..protocol.envelope import Message, encode, make, new_id
from ..protocol.errors import CATALOG_CONFLICT, INTERNAL_ERROR, NOT_READY, ProtocolError
from ..protocol.session import State
from ..server.bridge import CLOSE_INTERNAL, BridgeCore, Hooks, Peer
from .backend import MockBackend

log = logging.getLogger('mock')

SCENARIOS: dict[str, str] = {
    'default6': '집·시장·마트·노인정·교회·절(절은 비활성). 정상 동작',
    'empty': '빈 카탈로그. 첫 프리셋 추가 흐름',
    'conflict': '세션마다 첫 place.* / trip.start를 CATALOG_CONFLICT로 거절하고 최신 catalog를 보낸다',
    'limit': '프리셋 12개(상한)로 시작. 추가하면 PLACE_LIMIT',
    'in-use': '집→시장 주행 중으로 시작. 시장·집 삭제는 PLACE_IN_USE, 좌표 수정은 INVALID_STATE',
    'slow-ack': '모든 명령 응답을 2 s 늦게 보낸다(--ack-delay)',
    'error-ack': '주행·프리셋 명령을 error로 거절한다(--error-code로 코드 고정)',
    'flaky': '20 s마다 연결을 끊는다: close 1011 → TCP abort → 무응답(heartbeat 타임아웃) 순환',
    'sos': 'hello 5 s 뒤 tracking_lost SOS. ACK 없으면 2 s × 3 재전송. clear는 콘솔에서',
    'sos-x3': 'hello 5 s 뒤 같은 sos_id의 sos.request를 ACK와 무관하게 3번 보낸다',
    'sos-clear': 'hello 5 s 뒤 SOS, 10 s 뒤 sos.clear(tracking_recovered)',
    'garbage': '3 s마다 잘못된 JSON·알 수 없는 type/enum·binary frame 등을 하나씩 보낸다',
}

FIXTURE_REVISION = '2026-08-27T12:00:00Z'
EMPTY_REVISION = '2026-08-27T11:00:00Z'
IN_USE_TRIP_ID = 'bbef019a-dd3a-437e-8f5c-388913ae9eca'

# limit 시나리오에서 default6에 더하는 6곳
EXTRA_PLACES = [
    ('place_hospital', 'hospital', '병원', 36.1301, 128.5702),
    ('place_pharmacy', 'pharmacy', '약국', 36.1245, 128.5661),
    ('place_mart_2', 'mart', '큰마트', 36.1188, 128.5733),
    ('place_other', 'other', '공원', 36.1222, 128.5641),
    ('place_church_2', 'church', '새교회', 36.1311, 128.5759),
    ('place_senior_center_2', 'senior_center', '경로당', 36.1199, 128.5690),
]


@dataclass
class ScenarioOptions:
    ack_delay_s: float = 2.0
    error_code: str | None = None
    flaky_period_s: float = 20.0
    sos_delay_s: float = 5.0
    sos_clear_after_s: float = 10.0
    garbage_period_s: float = 3.0


def default6_places() -> dict[str, dict]:
    path = contracts_path() / 'fixtures' / 'place-catalog-default6.json'
    with open(path, encoding='utf-8') as f:
        payload = json.load(f)['payload']
    return {p['id']: {k: p[k] for k in FIELDS} for p in payload['places']}


def seed_places(scenario: str) -> tuple[dict[str, dict], str]:
    if scenario == 'empty':
        return {}, EMPTY_REVISION
    places = default6_places()
    if scenario == 'limit':
        for i, (pid, kind, label, lat, lon) in enumerate(EXTRA_PLACES):
            places[pid] = {'kind': kind, 'label': label, 'lat': lat, 'lon': lon, 'radius_m': 30,
                           'enabled': True, 'sort_order': 70 + 10 * i, 'is_default_origin': False,
                           'address': None}
    return places, FIXTURE_REVISION


def write_seed(path: Path, places: dict[str, dict], revision: str) -> None:
    """시나리오 초기 places.yaml. PlaceStore를 거치지 않아 fixture와 같은 revision을 유지한다."""
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = {'version': FILE_VERSION, 'revision': revision, 'places': places}
    tmp = path.with_name(path.name + '.seed.tmp')
    tmp.write_text(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False), encoding='utf-8')
    os.replace(tmp, path)


# error-ack: type → (code, retryable)
ERROR_ACK = {
    'trip.start': (NOT_READY, True),
    'trip.cancel': (INTERNAL_ERROR, True),
    'place.upsert': (INTERNAL_ERROR, False),
    'place.delete': (INTERNAL_ERROR, False),
}
FLAKY_MODES = ('close', 'abort', 'silent')


class ScenarioHooks(Hooks):

    def __init__(self, name: str, opts: ScenarioOptions, backend: MockBackend):
        if name not in SCENARIOS:
            raise ValueError(f'unknown scenario {name}')
        self.name = name
        self.opts = opts
        self.backend = backend
        self._flaky_i = 0
        self._garbage_i = 0

    async def on_session_open(self, core: BridgeCore, peer: Peer) -> None:
        if self.name == 'in-use' and self.backend.state.mission_state not in ('starting', 'active', 'cancelling'):
            self.backend.begin_trip(IN_USE_TRIP_ID, 'place_home', 'place_market', immediate=True)
            await core.notify_status()
        if self.name in ('sos', 'sos-x3', 'sos-clear'):
            peer.spawn(self._sos_flow(core, peer))
        elif self.name == 'flaky':
            peer.spawn(self._flaky(core, peer))
        elif self.name == 'garbage':
            peer.spawn(self._garbage(core, peer))

    async def before_command(self, core: BridgeCore, peer: Peer, msg: Message) -> None:
        if self.name == 'slow-ack':
            await asyncio.sleep(self.opts.ack_delay_s)
        elif self.name == 'error-ack' and msg.type in ERROR_ACK:
            code, retryable = ERROR_ACK[msg.type]
            if self.opts.error_code:
                code, retryable = self.opts.error_code, None
            raise ProtocolError(code, f'mock error-ack for {msg.type}', retryable=retryable)
        elif self.name == 'conflict' and msg.type in ('place.upsert', 'place.delete', 'trip.start') \
                and not peer.session.extra.get('conflicted'):
            peer.session.extra['conflicted'] = True
            cat = core.store.bump_revision()
            log.info('conflict: revision bumped to %s behind the app', cat.revision)
            raise ProtocolError(CATALOG_CONFLICT, 'catalog_revision is stale (changed by another editor)')

    # ------------------------------------------------------------ SOS

    async def _sos_flow(self, core: BridgeCore, peer: Peer) -> None:
        await asyncio.sleep(self.opts.sos_delay_s)
        if peer.session.state is not State.ACTIVE or core.sos.active:
            return
        sos_id = new_id()
        self.backend.set(tracking_state='lost')
        await core.request_sos(sos_id, 'tracking_lost', 'critical', self.backend.state.trip_id)
        if self.name == 'sos-clear':
            # 연결이 끊겨도 clear는 일어나야 한다. peer 태스크가 아니라 루프 타이머로 건다.
            asyncio.get_running_loop().call_later(
                self.opts.sos_clear_after_s,
                lambda: asyncio.ensure_future(self._clear(core, sos_id)))
        elif self.name == 'sos-x3':
            for _ in range(2):
                await asyncio.sleep(core.cfg.sos_retry_interval_s)
                await core.sos.resend(sos_id)

    async def _clear(self, core: BridgeCore, sos_id: str) -> None:
        await core.clear_sos(sos_id, 'tracking_recovered')
        active = self.backend.state.mission_state in ('starting', 'active')
        self.backend.set(tracking_state='following' if active else 'unavailable')

    # ------------------------------------------------------------ flaky

    async def _flaky(self, core: BridgeCore, peer: Peer) -> None:
        await asyncio.sleep(self.opts.flaky_period_s)
        mode = FLAKY_MODES[self._flaky_i % len(FLAKY_MODES)]
        self._flaky_i += 1
        log.warning('flaky: %s', {'close': 'close frame 1011', 'abort': 'TCP abort (no close frame)',
                                  'silent': 'going silent until heartbeat timeout'}[mode])
        if mode == 'close':
            core.close_peer(peer, CLOSE_INTERNAL, 'mock flaky: simulated restart')
        elif mode == 'abort':
            core.abort_peer(peer)
        else:
            peer.silent = True

    # ------------------------------------------------------------ garbage

    async def _garbage(self, core: BridgeCore, peer: Peer) -> None:
        while peer.session.state is State.ACTIVE:
            await asyncio.sleep(self.opts.garbage_period_s)
            label, frames = garbage_frames(core, self._garbage_i)
            self._garbage_i += 1
            log.info('garbage #%d: %s', self._garbage_i, label)
            for frame in frames:
                await core.send_raw(peer, frame)


def garbage_frames(core: BridgeCore, i: int) -> tuple[str, list]:
    """앱 견고성 시험용 잘못된 frame. 모두 한 사이클 안에 원래 상태로 돌아온다."""
    status = core.status_payload()
    catalog = core.store.catalog.to_payload()
    items = [
        ('malformed JSON', lambda: ['{"version":1,"type":"robot.status",']),
        ('JSON array instead of object', lambda: ['[1,2,3]']),
        ('unknown type robot.dance', lambda: [encode(make('robot.dance', {'speed': 'fast'}))]),
        ('robot.status with unknown enums', lambda: [encode(make('robot.status', {
            **status, 'bridge_state': 'confused', 'mission_state': 'teleporting',
            'tracking_state': 'psychic'}))]),
        ('unknown top-level and payload fields', lambda: [encode({
            **make('robot.status', {**status, 'x_future_field': {'a': 1}}), 'x_trace': 'abc'})]),
        ('version 2 envelope', lambda: [encode({**make('robot.status', status), 'version': 2})]),
        ('binary frame', lambda: [b'\x00\xffgarbage']),
        ('robot.status missing required fields', lambda: [encode(make('robot.status', {
            'robot_id': core.cfg.robot_id}))]),
        ('place.catalog with unknown kind (then the real catalog)', lambda: [
            encode(make('place.catalog', {**catalog, 'places': [
                {**p, 'kind': 'castle'} if n == 0 else p for n, p in enumerate(catalog['places'])]})),
            encode(make('place.catalog', catalog))]),
        ('error with unknown code', lambda: [encode(make('error', {
            'code': 'TEAPOT', 'message': 'mock garbage', 'retryable': False}))]),
        ('ack for an unknown request', lambda: [encode(make('ack', {'status': 'accepted'}, new_id()))]),
    ]
    label, fn = items[i % len(items)]
    return label, fn()
