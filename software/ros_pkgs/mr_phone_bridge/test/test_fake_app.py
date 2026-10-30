"""tools/devpc_demo/fake_app.py(데모용 가짜 앱) 회귀 시험. 실제 브리지 코어(mock 백엔드)에 붙여 handshake·주행·
SOS 응답(ACK 함/안 함·위치)·연결 끊김을 본다. 도구가 계약과 어긋나면 여기서 먼저 깨진다."""
import asyncio
import importlib.util
from pathlib import Path

import pytest

try:
    import websockets  # noqa: F401
    HAVE_WS = True
except ImportError:          # ROS 셸(apt python3-websockets 미설치)에서는 건너뛴다
    HAVE_WS = False
needs_ws = pytest.mark.skipif(not HAVE_WS, reason='websockets not installed')

from conftest import run  # noqa: E402
if HAVE_WS:
    from wsclient import bridge, uid  # noqa: E402

TOOL = Path(__file__).resolve().parents[1] / 'tools' / 'devpc_demo' / 'fake_app.py'


def load_tool():
    spec = importlib.util.spec_from_file_location('fake_app_under_test', TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_refuses_default_bridge_port():
    fa = load_tool()
    for url in ('ws://127.0.0.1:8765/bridge/v1', 'ws://127.0.0.1/bridge/v1'):
        with pytest.raises(SystemExit):
            fa.refuse_default_port(url)
    fa.refuse_default_port('ws://127.0.0.1:18865/bridge/v1')


def sos_requests(app, sos_id):
    return [e for e in app.history if e['dir'] == 'in' and e['type'] == 'sos.request'
            and e['payload']['sos_id'] == sos_id]


@needs_ws
def test_trip_sos_and_link_drop(tmp_path):
    fa = load_tool()

    async def body():
        async with bridge(tmp_path, heartbeat_interval_ms=500, heartbeat_timeout_ms=1500) as h:
            app = fa.FakeApp(h.url, tmp_path, fresh_delay_s=0.2)
            main = asyncio.ensure_future(app.run())
            assert (await app.wait('connected=true', 3))['ok']
            first_session = app.session_id

            # 주행: 장소는 이름·짧은 id 둘 다
            res = await app.handle(['trip', 'start', '집', 'mart'])
            assert res['ok'] and res['payload']['trip_id']
            assert (await app.wait('mission_state=active', 3))['ok']
            with pytest.raises(fa.CmdError):
                await app.handle(['trip', 'start', '집', '달나라'])

            # SOS + ACK + cache → fresh: 재전송 없이 SosAck 3번(ACK, cache, fresh)
            sid = uid()
            await h.core.request_sos(sid, 'tracking_lost', 'critical', res['payload']['trip_id'])
            await asyncio.sleep(0.8)
            acks = [a for a in h.published('/safety/sos_ack') if a['sos_id'] == sid]
            assert [(a['app_received'], a['best_source']) for a in acks] == [(True, 'none'), (True, 'cache'),
                                                                             (True, 'fresh')]
            assert len(sos_requests(app, sid)) == 1
            out = [e for e in app.history if e['dir'] == 'out' and e['type'] == 'sos.ack']
            assert out[-1]['reply_to'] == sos_requests(app, sid)[0]['id']
            await h.core.clear_sos(sid, 'tracking_recovered')
            assert (await app.wait('sos.clear', 2))['ok']
            assert sid not in app.sos

            # ACK 안 함: 첫 전송 + 재시도 3 → app_received=false
            await app.handle(['ack', 'no'])
            sid2 = uid()
            await h.core.request_sos(sid2, 'tracking_lost', 'critical', None)
            await asyncio.sleep(1.2)
            acks = [a for a in h.published('/safety/sos_ack') if a['sos_id'] == sid2]
            assert [(a['app_received'], a['retries']) for a in acks] == [(False, 3)]
            assert len(sos_requests(app, sid2)) == 4
            # 무응답 확정 뒤 같은 sos_id가 다시 오면(재연결 복원·sos-x3) 늦게라도 ACK. 그다음 재요청에는
            # 알림 없이 ACK와 최선 위치(none)를 다시 보낸다(10.5)
            await app.handle(['ack', 'yes'])
            await app.handle(['location', 'none'])
            await h.core.sos.resend(sid2)
            await asyncio.sleep(0.3)
            await h.core.sos.resend(sid2)
            await asyncio.sleep(0.3)
            sent = [(e['type'], e['payload'].get('source')) for e in app.history
                    if e['dir'] == 'out' and e['payload'].get('sos_id') == sid2]
            assert sent == [('sos.ack', None), ('location.update', 'none')] * 2
            acks = [a for a in h.published('/safety/sos_ack') if a['sos_id'] == sid2]
            assert [(a['app_received'], a['retries']) for a in acks] == [(False, 3), (True, 3)]
            await h.core.clear_sos(sid2, 'unknown')

            # 끊기: abort → 브리지가 바로 링크 끊김, connect → 새 세션에서 같은 trip
            await app.handle(['drop', 'abort'])
            assert (await app.wait('connected=false', 2))['ok']
            await asyncio.sleep(0.2)
            assert h.published('/phone/link_status')[-1]['connected'] is False
            assert app.status is None                    # 끊긴 동안의 상태는 버린다
            await app.handle(['connect'])
            assert (await app.wait('connected=true', 3))['ok']
            assert app.session_id != first_session
            assert (await app.wait('mission_state=active', 3))['ok']       # handshake의 robot.status로 다시 맞춤
            assert app.status['trip_id'] == res['payload']['trip_id']

            # silent: 브리지가 heartbeat 타임아웃(4001)으로 닫는다. s초 뒤 스스로 다시 붙는다
            await app.handle(['drop', 'silent', '2.5'])
            for _ in range(60):
                if app.last_close and app.last_close['code'] == 4001:
                    break
                await asyncio.sleep(0.05)
            assert app.last_close['code'] == 4001
            assert (await app.wait('connected=true', 4))['ok']

            assert app.violations == 0
            await app.handle(['quit'])
            await asyncio.wait_for(main, 3)
    run(body())
