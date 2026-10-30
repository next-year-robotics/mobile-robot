"""실제 Firebase RTDB 에뮬레이터(별도 포트)에 대고 도는 통합 시험. 서비스 계정 없이 M1-D를 검증한다.

- `--sim` 한 사이클을 실제 CLI로 돌리고, 에뮬레이터에 남은 문서를 계약 스키마로 검사한다.
- 에뮬레이터가 없던 동안 쌓인 이벤트가, 에뮬레이터가 뜬 뒤 원래 ts_ms·순서로 들어가는지 본다.
- 계약 보안 규칙(database.rules.json)을 올려 비로그인 읽기가 막히는지 본다.
java 또는 에뮬레이터 jar가 없으면 건너뛴다.
"""
import json
import logging
import os
import subprocess
import sys
import time
import urllib.error
import uuid

import pytest
from conftest import PKG, Emulator, contracts_dir, emulator_jar, free_port, load_json

from mr_care_telemetry.agent import AgentConfig, TelemetryAgent
from mr_care_telemetry.outbox import Outbox
from mr_care_telemetry.rtdb import RtdbClient

DB = 'https://senior-care-nyr-default-rtdb.asia-southeast1.firebasedatabase.app'


def test_sim_cycle_against_emulator(emulator, schemas, tmp_path):
    robot = f'mr-sim-{uuid.uuid4().hex[:6]}'
    out = subprocess.run([sys.executable, '-m', 'mr_care_telemetry', '--sim', '--emulator', emulator.host,
                          '--robot-id', robot, '--fast', '--cycles', '1', '--queue', str(tmp_path / 'q.db')],
                         cwd=PKG, capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, out.stderr[-2000:]
    root = emulator.rest('GET', f'robots/{robot}')
    schemas.check('meta', root['meta'])
    status = root['status']
    for k in ('trip_id', 'from_place_id', 'to_place_id', 'battery_percent', 'last_error'):
        status.setdefault(k, None)                  # RTDB는 null을 저장하지 않는다
    schemas.check('status', status)
    assert status['mission_state'] == 'idle' and status['agent']['version'] == 'sim'
    keys = sorted(root['events'])
    events = [root['events'][k] for k in keys]
    for e in events:
        schemas.check('event', e)
        assert not {'lat', 'lon'} & set(e.get('data', {}))
    assert [e['ts_ms'] for e in events] == sorted(e['ts_ms'] for e in events)     # push id 순 = 시간 순
    assert [e['kind'] for e in events] == [
        'agent.online', 'link.connected', 'trip.start', 'sos.request', 'sos.ack', 'sos.clear', 'trip.cancelled',
        'link.disconnected', 'agent.offline']
    ack = next(e for e in events if e['kind'] == 'sos.ack')
    assert ack['data'] == {'app_received': True, 'retries': 0, 'location': 'cache', 'accuracy_bucket': 'good'}
    assert 'sos' not in root                                                        # clear에서 삭제


def test_sos_node_live_on_emulator(emulator, schemas, tmp_path):
    robot = f'mr-sos-{uuid.uuid4().hex[:6]}'
    client = RtdbClient(DB, emulator_host=emulator.host)
    agent = TelemetryAgent(AgentConfig(robot_id=robot, loop_s=0.05, ack_debounce_s=0.2), client,
                           Outbox(tmp_path / 'q.db'))
    agent.start()
    try:
        sid = str(uuid.uuid4())
        agent.on_sos_request(sos_id=sid, reason='tracking_lost', severity='critical', trip_id=None)
        agent.on_sos_ack(sos_id=sid, app_received=True, retries=1)
        agent.on_location(reason='sos', sos_id=sid, has_fix=True, lat=36.1234, lon=128.5678, accuracy_m=4.2,
                          captured_at_ms=1790000000000, source='cache', low_confidence=False)
        node = wait_for(lambda: (emulator.rest('GET', f'robots/{robot}/sos/{sid}') or {}).get('location'))
        node = emulator.rest('GET', f'robots/{robot}/sos/{sid}')
        node.setdefault('trip_id', None)
        schemas.check('sos', node)
        assert node['app_received'] is True and node['location']['source'] == 'cache'
        agent.on_sos_clear(sos_id=sid, resolution='handled')
        wait_for(lambda: emulator.rest('GET', f'robots/{robot}/sos/{sid}') is None)
    finally:
        agent.stop()


def test_offline_then_online_keeps_ts_and_order(tmp_path, schemas):
    if emulator_jar() is None:
        pytest.skip('RTDB emulator jar not available')
    port = free_port()
    robot = 'mr-001'
    client = RtdbClient(DB, emulator_host=f'127.0.0.1:{port}', timeout_s=1.0)
    agent = TelemetryAgent(AgentConfig(robot_id=robot, loop_s=0.05, backoff_max_s=1.0), client,
                           Outbox(tmp_path / 'q.db'))
    agent.start()                                       # 에뮬레이터가 아직 없다 = 오프라인
    emu = None
    try:
        agent.on_bridge_alive(True)
        agent.on_app_status(mission_state='idle', tracking_state='unavailable', trip_id=None, from_place_id=None,
                            to_place_id=None, battery_percent=None, last_error=None)
        t0 = int(time.time() * 1000)
        sid = str(uuid.uuid4())
        agent.on_sos_request(sos_id=sid, reason='stuck', severity='warning', trip_id=None)
        time.sleep(0.3)
        agent.on_sos_clear(sos_id=sid, resolution='cancelled')
        wait_for(lambda: agent.online is False)
        assert len(agent.outbox) >= 4
        time.sleep(1.0)
        emu = Emulator(port).start()                     # 온라인이 됐다
        wait_for(lambda: len(agent.outbox) == 0, timeout=15)
        wait_for(lambda: emu.rest('GET', f'robots/{robot}/status'), timeout=10)     # status는 5 s 주기
        root = emu.rest('GET', f'robots/{robot}')
        events = [root['events'][k] for k in sorted(root['events'])]
        assert [e['kind'] for e in events] == ['agent.online', 'sos.request', 'sos.clear']
        assert t0 - 5000 < events[0]['ts_ms'] <= events[1]['ts_ms'] < events[2]['ts_ms']
        assert events[2]['ts_ms'] - events[1]['ts_ms'] >= 250                       # 원래 간격 유지
        assert 'sos' not in root and root['status']['bridge_state'] == 'ready'
        for e in events:
            schemas.check('event', e)
    finally:
        agent.stop()
        if emu:
            emu.stop()


def test_contract_rules_on_emulator(emulator):
    """계약 규칙: 비로그인 읽기 금지, 클라이언트 쓰기 금지, 에이전트(관리자)는 쓴다.
    역할별(README 4장): guardian·dev 모두 meta·status·events·sos를 읽고, diag는 dev만. 로봇 노드 통째 읽기는 아무도."""
    ns = f'rules{uuid.uuid4().hex[:6]}'
    rules = load_json(contracts_dir() / 'database.rules.json')
    emulator.rest('PUT', '.settings/rules', rules, ns=ns)
    emulator.rest('PUT', 'robots/mr-001/status', {'mission_state': 'idle'}, ns=ns)       # owner = 에이전트
    with pytest.raises(urllib.error.HTTPError) as ei:
        emulator.rest('GET', 'robots/mr-001', ns=ns, auth=None)
    assert ei.value.code in (401, 403)
    with pytest.raises(urllib.error.HTTPError) as ei:
        emulator.rest('PUT', 'robots/mr-001/status', {'x': 1}, ns=ns, auth=None)
    assert ei.value.code in (401, 403)
    assert emulator.rest('GET', 'robots/mr-001/status', ns=ns) == {'mission_state': 'idle'}

    emulator.rest('PATCH', '', {'access/mr-001': {'g1': 'guardian', 'd1': 'dev'},
                                'robots/mr-001/meta': {'robot_id': 'mr-001'},
                                'robots/mr-001/events/-N0000000000000000001': {'kind': 'agent.online'},
                                'robots/mr-001/sos/s1': {'reason': 'manual'},
                                'robots/mr-001/diag': {'updated_at_ms': 1}}, ns=ns)

    def can_read(uid, path):
        try:
            emulator.rest('GET', path, ns=ns, as_uid=uid)
            return True
        except urllib.error.HTTPError as e:
            assert e.code in (401, 403)
            return False

    kids = ['meta', 'status', 'events', 'sos', 'diag']
    assert [can_read('g1', f'robots/mr-001/{k}') for k in kids] == [True, True, True, True, False]
    assert [can_read('d1', f'robots/mr-001/{k}') for k in kids] == [True, True, True, True, True]
    assert [can_read('x9', f'robots/mr-001/{k}') for k in kids] == [False] * 5
    assert not can_read('g1', 'robots/mr-001') and not can_read('d1', 'robots/mr-001')
    assert can_read('g1', 'access/mr-001/g1') and not can_read('g1', 'access/mr-001/d1')
    with pytest.raises(urllib.error.HTTPError):                                       # dev도 쓰기는 못 한다
        emulator.rest('PUT', 'robots/mr-001/diag', {'x': 1}, ns=ns, as_uid='d1')


def test_cli_requires_explicit_target():
    env = {k: v for k, v in os.environ.items() if k != 'FIREBASE_DATABASE_EMULATOR_HOST'}
    out = subprocess.run([sys.executable, '-m', 'mr_care_telemetry', '--sim'], cwd=PKG, env=env,
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 2 and '--real' in out.stderr


def wait_for(pred, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        v = pred()
        if v:
            return v
        time.sleep(0.05)
    raise AssertionError('condition not met')


def test_dump_json_is_utf8(emulator):
    """한국어 문구가 그대로(이스케이프 없이도) 저장·조회된다."""
    ns = f'utf{uuid.uuid4().hex[:6]}'
    client = RtdbClient(DB, emulator_host=emulator.host, namespace=ns)
    client.put('x', {'message': '어르신을 다시 찾았어요'})
    assert json.dumps(emulator.rest('GET', 'x', ns=ns), ensure_ascii=False) == '{"message": "어르신을 다시 찾았어요"}'


def test_diag_on_emulator(emulator, schemas, tmp_path, agent_log):
    """2026-09-27 회귀: publish_diag인데 diag가 저장되지 않았다(키에 '/'). 실제 RTDB 규칙으로 쓰고 읽는다.
    옛 모양은 에뮬레이터가 400으로 거절하고, 그 오류가 로그 ERROR로 나와야 한다."""
    robot = f'mr-diag-{uuid.uuid4().hex[:6]}'
    client = RtdbClient(DB, emulator_host=emulator.host)
    agent = TelemetryAgent(AgentConfig(robot_id=robot, loop_s=0.05, publish_diag=True, diag_period_s=0.2), client,
                           Outbox(tmp_path / 'q.db'))
    agent.start()
    try:
        agent.on_base_status(agent_state='connected', cmd_watchdog_ok=True)
        agent.on_marker(1)
        diag = wait_for(lambda: emulator.rest('GET', f'robots/{robot}/diag'))
        schemas.check('diag', diag)
        rows = {}

        def walk(node, path):                       # 대시보드 topicRows와 같은 읽기 규칙
            if 'last_seen_ms' in node:
                rows['/' + '/'.join(path)] = node
                return
            for k, v in node.items():
                walk(v, path + [k])
        walk(diag['topics'], [])
        assert set(rows) == {'/base/status', '/joint_states', '/marker_track', '/robot/app_status'}
        assert rows['/base/status']['last_seen_ms'] > 0

        old_shape = {'updated_at_ms': 1, 'topics': {'/base/status': {'last_seen_ms': 1, 'hz': 1.0}}}
        agent.diag_doc = lambda elapsed_s: old_shape
        msg = wait_for(lambda: agent_log.messages(logging.ERROR, 'rejected diag'))[0]
        assert '400' in msg and "can't be empty or contain" in msg and '\n' not in msg
        assert agent.online is True
    finally:
        agent.stop()
