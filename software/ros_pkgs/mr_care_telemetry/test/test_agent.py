"""TelemetryAgent: 이벤트 매핑·status·sos 노드·오프라인 큐. 메모리 RTDB + 가짜 시계로 전송 스텝을 직접 돈다."""
import pytest
from conftest import FakeClock, MemoryRtdb

from mr_care_telemetry.agent import AgentConfig, TelemetryAgent
from mr_care_telemetry.outbox import Outbox

TRIP = 'bbef019a-dd3a-437e-8f5c-388913ae9eca'
SOS = '835fcafe-60ac-4517-9f06-50bcd2486b6c'
LABELS = {'place_home': '집', 'place_market': '시장', 'place_mart': '마트'}


class Rig:
    def __init__(self, **cfg):
        self.db = MemoryRtdb()
        self.clock = FakeClock()
        self.ms = 1_790_000_000_000
        self.agent = TelemetryAgent(AgentConfig(**cfg), self.db, Outbox(), clock=self.clock,
                                    wall_ms=lambda: self.ms)

    def step(self, dt=0.25, n=1):
        for _ in range(n):
            self.clock.advance(dt)
            self.ms += int(dt * 1000)
            self.agent._step()

    def written(self, path):
        """마지막으로 PUT한 본문. RTDB는 null 필드를 저장하지 않으므로 스키마 검사는 저장본이 아니라 이것으로 한다."""
        return [c[2] for c in self.db.calls if c[0] == 'PUT' and c[1] == path][-1]

    def events(self):
        ev = self.db.at('robots/mr-001/events') or {}
        return [ev[k] for k in sorted(ev)]

    def kinds(self):
        return [e['kind'] for e in self.events()]

    def app(self, mission, tracking='unavailable', trip=None, frm=None, to=None, err=None):
        self.agent.on_app_status(mission_state=mission, tracking_state=tracking, trip_id=trip, from_place_id=frm,
                                 to_place_id=to, battery_percent=None, last_error=err)

    def link(self, connected, app='9e889e5d-1c2d-4572-9513-9cd8283c917a'):
        self.agent.on_link_status(connected=connected, app_instance_id=app, session_id='s' if connected else '',
                                  last_seen_ms=self.ms)


@pytest.fixture
def rig():
    r = Rig()
    r.agent.on_bridge_alive(True)
    r.agent.on_place_catalog(LABELS)
    r.agent._emit('agent.online', 'info', '로봇이 온라인이 되었어요', data={'version': '0.1.0'})
    return r


def test_meta_status_and_online_event(rig, schemas):
    rig.app('idle')
    rig.link(False)
    rig.step()
    meta = rig.written('robots/mr-001/meta')
    schemas.check('meta', meta)
    assert meta['display_name'] == '돌봄 로봇 1호' and rig.db.at('robots/mr-001/meta') == meta
    st = rig.written('robots/mr-001/status')
    schemas.check('status', st)
    assert (st['bridge_state'], st['mission_state'], st['phone_link']['connected']) == ('ready', 'idle', False)
    assert 'trip_id' not in rig.db.at('robots/mr-001/status')          # null은 저장되지 않는다
    assert rig.kinds() == ['agent.online']
    for e in rig.events():
        schemas.check('event', e)


def test_meta_keeps_first_seen():
    r = Rig()
    r.db.tree = {'robots': {'mr-001': {'meta': {'robot_id': 'mr-001', 'display_name': 'x', 'agent_version': '0',
                                                'first_seen_ms': 123}}}}
    r.step()
    assert r.db.at('robots/mr-001/meta')['first_seen_ms'] == 123


def test_trip_events(rig, schemas):
    rig.app('idle')
    rig.link(False)
    rig.link(True)
    rig.app('starting', 'searching', TRIP, 'place_home', 'place_market')
    rig.app('active', 'following', TRIP, 'place_home', 'place_market')
    rig.agent.on_trip_command(command='cancel', trip_id=TRIP, reason='user_request')
    rig.app('cancelling', 'following', TRIP, 'place_home', 'place_market')
    rig.app('cancelled', 'unavailable', TRIP, 'place_home', 'place_market')
    rig.app('idle')
    rig.link(False)
    rig.step()
    ev = rig.events()
    assert [e['kind'] for e in ev] == ['agent.online', 'link.connected', 'trip.start', 'trip.cancelled',
                                       'link.disconnected']
    start = ev[2]
    assert start['message'] == '집에서 시장으로 출발했어요' and start['trip_id'] == TRIP
    assert start['data'] == {'from_place_id': 'place_home', 'to_place_id': 'place_market'}
    assert ev[3]['data'] == {'reason': 'user_request'}
    assert ev[4]['severity'] == 'warning'
    for e in ev:
        schemas.check('event', e)


def test_trip_error_event(rig, schemas):
    rig.app('idle')
    rig.app('starting', 'searching', TRIP, 'place_home', 'place_mart')
    rig.app('error', 'unavailable', TRIP, 'place_home', 'place_mart', err='추종을 시작하지 못했어요')
    rig.app('error', 'unavailable', TRIP, 'place_home', 'place_mart', err='추종을 시작하지 못했어요')
    rig.step()
    ev = rig.events()
    assert [e['kind'] for e in ev] == ['agent.online', 'trip.start', 'trip.error']
    assert ev[1]['message'] == '집에서 마트로 출발했어요'
    assert ev[2]['message'] == '추종을 시작하지 못했어요' and ev[2]['severity'] == 'warning'
    schemas.check('event', ev[2])


def test_baseline_does_not_replay(rig):
    rig.app('active', 'following', TRIP, 'place_home', 'place_market')     # 기동 때 이미 주행 중
    rig.link(True)                                                          # 기동 때 이미 연결
    rig.step()
    assert rig.kinds() == ['agent.online']


def test_sos_flow(rig, schemas):
    rig.app('active', 'following', TRIP, 'place_home', 'place_market')
    a = rig.agent
    a.on_sos_request(sos_id=SOS, reason='tracking_lost', severity='critical', trip_id=TRIP)
    a.on_sos_request(sos_id=SOS, reason='tracking_lost', severity='critical', trip_id=TRIP)   # 중복
    rig.step()
    schemas.check('sos', rig.written(f'robots/mr-001/sos/{SOS}'))
    node = rig.db.at(f'robots/mr-001/sos/{SOS}')
    assert node['app_received'] is False and 'location' not in node        # null은 RTDB에 저장되지 않는다
    a.on_sos_ack(sos_id=SOS, app_received=True, retries=0, has_location=False, best_source='none')
    a.on_location(reason='sos', sos_id=SOS, has_fix=True, lat=36.1234, lon=128.5678, accuracy_m=4.2,
                  captured_at_ms=rig.ms - 800, source='cache', low_confidence=False)
    rig.step(0.25)
    assert 'sos.ack' not in rig.kinds()                                     # 1 s 모아서 낸다
    rig.step(0.25, n=4)
    ack = [e for e in rig.events() if e['kind'] == 'sos.ack'][0]
    assert ack['data'] == {'app_received': True, 'retries': 0, 'location': 'cache', 'accuracy_bucket': 'good'}
    a.on_location(reason='sos', sos_id=SOS, has_fix=True, lat=36.12341, lon=128.56779, accuracy_m=3.1,
                  captured_at_ms=rig.ms - 200, source='fresh', low_confidence=False)
    a.on_location(reason='sos', sos_id=SOS, has_fix=True, lat=36.2, lon=128.6, accuracy_m=3.0,
                  captured_at_ms=rig.ms, source='cache', low_confidence=False)          # 품질이 낮다: 무시
    rig.step()
    node = rig.db.at(f'robots/mr-001/sos/{SOS}')
    schemas.check('sos', {**node, 'trip_id': node.get('trip_id')})
    assert node['app_received'] is True and node['location']['source'] == 'fresh'
    assert node['location']['lat'] == 36.12341
    a.on_sos_clear(sos_id=SOS, resolution='tracking_recovered')
    a.on_sos_ack(sos_id=SOS, app_received=True, retries=0)                 # 종료 뒤 늦게 온 것: 무시
    rig.step()
    assert rig.db.at(f'robots/mr-001/sos/{SOS}') is None
    ev = rig.events()
    assert [e['kind'] for e in ev] == ['agent.online', 'sos.request', 'sos.ack', 'sos.clear']
    assert ev[1]['severity'] == 'critical' and ev[1]['message'] == '로봇이 어르신을 놓쳤어요'
    assert ev[3]['message'] == '어르신을 다시 찾았어요' and ev[3]['trip_id'] == TRIP
    for e in ev:
        schemas.check('event', e)
        assert 'lat' not in e.get('data', {}) and 'lon' not in e.get('data', {})


def test_sos_ack_failure_is_immediate_and_cli_ids(rig):
    a = rig.agent
    a.on_sos_request(sos_id='cli-test', reason='stuck', severity='warning', trip_id=None)
    a.on_sos_ack(sos_id='cli-test', app_received=False, retries=3)
    rig.step()
    ev = rig.events()
    assert [e['kind'] for e in ev] == ['agent.online', 'sos.request', 'sos.ack']
    assert ev[2]['data']['app_received'] is False and ev[2]['message'] == '어르신 폰이 응답하지 않아요'
    wire = ev[1]['sos_id']
    assert wire != 'cli-test' and rig.db.at(f'robots/mr-001/sos/{wire}')['severity'] == 'warning'
    # 앱의 위치는 wire id로 온다
    a.on_location(reason='sos', sos_id=wire, has_fix=True, lat=36.1, lon=128.5, accuracy_m=80.0,
                  captured_at_ms=rig.ms, source='cache', low_confidence=True)
    a.on_sos_ack(sos_id='cli-test', app_received=True, retries=3)          # 늦은 성공
    rig.step(0.5, n=3)
    acks = [e for e in rig.events() if e['kind'] == 'sos.ack']
    assert [x['data']['app_received'] for x in acks] == [False, True]
    assert acks[1]['data']['accuracy_bucket'] == 'poor'


def test_sos_location_updates_limited(rig):
    a = rig.agent
    a.on_sos_request(sos_id=SOS, reason='tracking_lost', severity='critical', trip_id=None)
    for i in range(6):
        a.on_location(reason='sos', sos_id=SOS, has_fix=True, lat=36.1 + i / 1000, lon=128.5, accuracy_m=5.0,
                      captured_at_ms=rig.ms, source='fresh', low_confidence=False)
    patches = [c for c in rig.agent.outbox.peek(50) if c.method == 'PATCH']
    assert len(patches) == 3


def test_periodic_location_never_written(rig):
    rig.agent.on_location(reason='periodic', sos_id=None, has_fix=True, lat=36.1, lon=128.5, accuracy_m=5.0,
                          captured_at_ms=rig.ms, source='cache', low_confidence=False)
    rig.step()
    assert 'lat' not in repr(rig.db.tree)


def test_offline_queue_backoff_and_order(rig):
    db, a = rig.db, rig.agent
    rig.step()
    db.offline = True
    rig.app('idle')
    rig.link(False)
    t_first = rig.ms
    a.on_sos_request(sos_id=SOS, reason='tracking_lost', severity='critical', trip_id=None)
    rig.step(0.1)
    t_clear = rig.ms
    a.on_sos_clear(sos_id=SOS, resolution='handled')
    backoffs = []
    for _ in range(8):
        rig.step(0.1)
        backoffs.append(a._backoff_s)
        rig.clock.advance(a._backoff_s)
    assert backoffs[:1] == [1.0] and max(backoffs) <= 60.0 and backoffs == sorted(backoffs)
    assert a.online is False and len(a.outbox) >= 4
    status_before = db.at('robots/mr-001/status')['updated_at_ms']
    db.offline = False
    rig.step(0.1)
    rig.step(0.1)
    assert len(a.outbox) == 0 and a.online is True
    ev = rig.events()
    assert [e['kind'] for e in ev] == ['agent.online', 'sos.request', 'sos.clear']
    assert ev[1]['ts_ms'] == t_first and ev[2]['ts_ms'] == t_clear          # 원래 시각 유지
    assert db.at(f'robots/mr-001/sos/{SOS}') is None                         # 생성·삭제 순서 유지
    assert db.at('robots/mr-001/status')['updated_at_ms'] > status_before


def test_permanent_error_drops_op(rig):
    rig.db.fail_status['robots/mr-001/events'] = 400
    rig.agent._emit('error', 'warning', 'x', data={'source': 't', 'error': 'e'})
    rig.step()
    assert len(rig.agent.outbox) == 0 and rig.agent.errors


def test_status_rules(rig, schemas):
    a = rig.agent
    a.on_base_status(agent_state='connected', cmd_watchdog_ok=True)
    rig.link(True)
    rig.app('active', 'following', TRIP, 'place_home', 'place_market')
    doc = a.status_doc()
    schemas.check('status', doc)
    assert (doc['bridge_state'], doc['mission_state'], doc['phone_link']['connected']) == ('ready', 'active', True)
    assert doc['base']['agent_state'] == 'connected'
    rig.clock.advance(7)                                                     # app_status 낡음
    doc = a.status_doc()
    assert (doc['bridge_state'], doc['mission_state'], doc['trip_id']) == ('degraded', 'unknown', None)
    a.on_marker(1)
    assert a.status_doc()['tracking_state'] == 'following'
    a.on_bridge_alive(False)                                                 # 브리지가 죽었다
    doc = a.status_doc()
    assert doc['bridge_state'] == 'unknown' and doc['phone_link']['connected'] is False
    schemas.check('status', doc)


def test_status_throttle_and_period(rig):
    rig.step()
    puts = lambda: [c for c in rig.db.calls if c[0] == 'PUT' and c[1].endswith('/status')]  # noqa: E731
    n = len(puts())
    rig.step(0.25, n=4)                          # 변화 없음: 5 s 주기 전에는 안 쓴다
    assert len(puts()) == n
    rig.app('idle')
    rig.step(0.25, n=2)                          # 변화: 곧바로(최소 0.5 s 간격)
    assert len(puts()) == n + 1
    rig.step(1.0, n=5)
    assert len(puts()) == n + 2


def test_prune_keeps_newest(rig):
    rig.agent.cfg.max_events = 10
    for i in range(15):
        rig.ms += 1
        rig.agent._emit('agent.online', 'info', f'e{i}', data={'version': 'x'})
    rig.step()
    msgs = [e['message'] for e in rig.events()]
    assert len(msgs) == 10 and msgs[-1] == 'e14' and 'e4' not in msgs


def test_diag_doc(rig):
    rig.agent.cfg.publish_diag = True
    rig.agent.on_base_status(agent_state='connected', cmd_watchdog_ok=True)
    rig.step()
    diag = rig.db.at('robots/mr-001/diag')
    assert set(diag['topics']) == {'/base/status', '/joint_states', '/marker_track', '/robot/app_status'}
    assert diag['topics']['/base/status']['last_seen_ms'] > 0
