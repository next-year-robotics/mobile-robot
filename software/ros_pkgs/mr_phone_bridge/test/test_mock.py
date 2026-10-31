"""mock 시나리오 12종과 CLI 진입점."""
import asyncio
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import pytest

try:
    import websockets  # noqa: F401
    HAVE_WS = True
except ImportError:          # ROS 셸(apt python3-websockets 미설치)에서는 이 모듈만 건너뛴다
    HAVE_WS = False
# 모듈 수준 importorskip은 ROS의 pytest 7 + launch_testing 플러그인에서 세션 전체를 건너뛰게 만든다.
pytestmark = pytest.mark.skipif(not HAVE_WS, reason='websockets not installed')

from conftest import run  # noqa: E402
if HAVE_WS:
    from wsclient import Client, bridge, place_payload, uid  # noqa: E402

from mr_phone_bridge.mock.scenarios import SCENARIOS, ScenarioHooks, ScenarioOptions  # noqa: E402
from mr_phone_bridge.server.bridge import CLOSE_HEARTBEAT_TIMEOUT, CLOSE_INTERNAL  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
REV = '2026-08-27T12:00:00Z'
OPTS = ScenarioOptions(ack_delay_s=0.5, flaky_period_s=0.4, sos_delay_s=0.2, sos_clear_after_s=0.6,
                       garbage_period_s=0.05)


def scenario_bridge(tmp_path, name, opts=OPTS, **kw):
    from mr_phone_bridge.mock.backend import MockBackend
    backend = MockBackend(start_delay_s=0.1, cancel_delay_s=0.1, terminal_hold_s=0.3)
    return bridge(tmp_path, scenario=name, backend=backend, hooks=ScenarioHooks(name, opts, backend), **kw)


def test_scenario_list_matches_protocol():
    assert set(SCENARIOS) == {'default6', 'empty', 'conflict', 'limit', 'in-use', 'slow-ack', 'error-ack',
                              'flaky', 'sos', 'sos-x3', 'sos-clear', 'garbage'}


def test_empty(tmp_path):
    async def body():
        async with scenario_bridge(tmp_path, 'empty') as h:
            c = await h.client()
            await c.hello()
            assert c.catalog['places'] == [] and c.catalog['default_origin_place_id'] is None
            mid = await c.send('place.upsert', place_payload(c.catalog['revision'], kind='home', label='집'))
            ack = await c.reply(mid)
            cat = await c.expect('place.catalog')
            assert cat['payload']['default_origin_place_id'] == ack['payload']['place_id'] == 'place_home'
            await c.close()
    run(body())


def test_conflict(tmp_path):
    async def body():
        async with scenario_bridge(tmp_path, 'conflict') as h:
            c = await h.client()
            await c.hello()
            mid = await c.send('place.upsert', place_payload(REV))
            err = await c.reply(mid)
            assert err['payload']['code'] == 'CATALOG_CONFLICT' and err['payload']['retryable'] is True
            cat = await c.expect('place.catalog')
            new_rev = cat['payload']['revision']
            assert new_rev != REV
            ok = await c.send('place.upsert', place_payload(new_rev))
            assert (await c.reply(ok))['type'] == 'ack'
            await c.close()
    run(body())


def test_limit(tmp_path):
    async def body():
        async with scenario_bridge(tmp_path, 'limit') as h:
            c = await h.client()
            await c.hello()
            assert len(c.catalog['places']) == 12
            mid = await c.send('place.upsert', place_payload(REV))
            assert (await c.reply(mid))['payload']['code'] == 'PLACE_LIMIT'
            await c.close()
    run(body())


def test_in_use(tmp_path):
    async def body():
        async with scenario_bridge(tmp_path, 'in-use') as h:
            c = await h.client()
            await c.hello()
            st = await c.expect('robot.status', where=lambda m: m['payload']['mission_state'] == 'active')
            assert st['payload']['to_place_id'] == 'place_market'
            dl = await c.send('place.delete', {'place_id': 'place_market', 'catalog_revision': REV})
            assert (await c.reply(dl))['payload']['code'] == 'PLACE_IN_USE'
            market = next(p for p in c.catalog['places'] if p['id'] == 'place_market')
            fields = {k: v for k, v in market.items() if k != 'id'}
            mv = await c.send('place.upsert', place_payload(REV, place_id='place_market',
                                                            **{**fields, 'lat': 36.2}))
            assert (await c.reply(mv))['payload']['code'] == 'INVALID_STATE'
            ts = await c.send('trip.start', {'from_place_id': 'place_home', 'to_place_id': 'place_mart',
                                             'catalog_revision': REV})
            assert (await c.reply(ts))['payload']['code'] == 'INVALID_STATE'
            await c.close()
    run(body())


def test_slow_ack(tmp_path):
    async def body():
        async with scenario_bridge(tmp_path, 'slow-ack') as h:
            c = await h.client()
            await c.hello()
            t0 = time.monotonic()
            mid = await c.send('trip.start', {'from_place_id': 'place_home', 'to_place_id': 'place_market',
                                              'catalog_revision': REV})
            assert (await c.reply(mid, timeout=3))['type'] == 'ack'
            assert time.monotonic() - t0 >= 0.5
            await c.close()
    run(body())


def test_error_ack(tmp_path):
    async def body():
        async with scenario_bridge(tmp_path, 'error-ack') as h:
            c = await h.client()
            await c.hello()
            ts = await c.send('trip.start', {'from_place_id': 'place_home', 'to_place_id': 'place_market',
                                             'catalog_revision': REV})
            assert (await c.reply(ts))['payload']['code'] == 'NOT_READY'
            up = await c.send('place.upsert', place_payload(REV))
            e = await c.reply(up)
            assert e['payload']['code'] == 'INTERNAL_ERROR' and e['payload']['retryable'] is False
            loc = await c.send('location.update', {'reason': 'periodic', 'sos_id': None, 'has_fix': False,
                                                   'lat': None, 'lon': None, 'horizontal_accuracy_m': None,
                                                   'captured_at_ms': None, 'age_ms': None, 'source': 'none',
                                                   'failure_reason': 'unavailable'})
            assert (await c.reply(loc))['type'] == 'ack'
            await c.close()
        opts = ScenarioOptions(error_code='PERMISSION_DENIED')
        async with scenario_bridge(tmp_path, 'error-ack', opts) as h:
            c = await h.client()
            await c.hello()
            ts = await c.send('trip.cancel', {'trip_id': uid(), 'reason': 'user_request'})
            assert (await c.reply(ts))['payload']['code'] == 'PERMISSION_DENIED'
            await c.close()
    run(body())


def test_flaky_cycles_close_abort_silent(tmp_path):
    async def body():
        async with scenario_bridge(tmp_path, 'flaky') as h:
            codes = []
            for _ in range(3):
                c = await h.client()
                await c.hello()
                if len(codes) == 2:
                    # silent: 서버가 아무것도 보내지 않는다. heartbeat도 무시 → 서버 타임아웃
                    await asyncio.sleep(0.5)
                    hb = await c.send('heartbeat', {'session_id': c.session_id})
                    with pytest.raises(AssertionError):
                        await c.reply(hb, timeout=0.3)
                codes.append(await c.closed(4.0))
            assert codes == [CLOSE_INTERNAL, 1006, CLOSE_HEARTBEAT_TIMEOUT]
    run(body())


def test_sos(tmp_path):
    async def body():
        async with scenario_bridge(tmp_path, 'sos') as h:
            c = await h.client()
            await c.hello()
            req = await c.expect('sos.request', timeout=2)
            assert req['payload']['reason'] == 'tracking_lost' and req['payload']['severity'] == 'critical'
            await c.expect('robot.status', where=lambda m: m['payload']['tracking_state'] == 'lost')
            await c.send('sos.ack', {'sos_id': req['payload']['sos_id'], 'received': True}, reply_to=req['id'])
            await asyncio.sleep(0.6)
            assert not [m for m in c.backlog if m['type'] == 'sos.request']
            await c.close()
            c2 = await h.client()                    # 재연결 시 복원, 새 SOS는 만들지 않는다
            await c2.hello()
            again = await c2.expect('sos.request')
            assert again['payload']['sos_id'] == req['payload']['sos_id']
            await asyncio.sleep(0.5)
            assert not [m for m in c2.backlog if m['type'] == 'sos.request']
            await c2.close()
    run(body())


def test_sos_x3(tmp_path):
    async def body():
        async with scenario_bridge(tmp_path, 'sos-x3') as h:
            c = await h.client()
            await c.hello()
            first = await c.expect('sos.request', timeout=2)
            sid = first['payload']['sos_id']
            await c.send('sos.ack', {'sos_id': sid, 'received': True}, reply_to=first['id'])
            more = [await c.expect('sos.request', timeout=1.0) for _ in range(2)]
            assert {m['payload']['sos_id'] for m in more} == {sid}
            await c.close()
    run(body())


def test_sos_clear(tmp_path):
    async def body():
        async with scenario_bridge(tmp_path, 'sos-clear') as h:
            c = await h.client()
            await c.hello()
            req = await c.expect('sos.request', timeout=2)
            await c.send('sos.ack', {'sos_id': req['payload']['sos_id'], 'received': True}, reply_to=req['id'])
            clr = await c.expect('sos.clear', timeout=2)
            assert clr['payload'] == {'sos_id': req['payload']['sos_id'], 'resolution': 'tracking_recovered'}
            await c.close()
    run(body())


def test_garbage(tmp_path):
    async def body():
        async with scenario_bridge(tmp_path, 'garbage') as h:
            c = await h.client(strict=False)
            await c.hello()
            kinds = set()
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not {'bytes', '['} <= kinds:
                raw = await asyncio.wait_for(c.ws.recv(), 1.0)
                kinds.add(type(raw).__name__)
                if isinstance(raw, str):
                    kinds.add(raw[:1])
            assert 'bytes' in kinds and '[' in kinds
            # 서버는 여전히 정상 응답한다
            hb = await c.send('heartbeat', {'session_id': c.session_id})
            deadline = time.monotonic() + 2.0
            while True:
                raw = await asyncio.wait_for(c.ws.recv(), 1.0)
                if isinstance(raw, str) and hb in raw and 'heartbeat.ack' in raw:
                    break
                assert time.monotonic() < deadline
            await c.close()
    run(body())


def test_cli_list():
    out = subprocess.run([sys.executable, '-m', 'mr_phone_bridge.mock', '--list'], cwd=PKG,
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0
    assert len(out.stdout.strip().splitlines()) == 12


def test_cli_starts_and_serves(tmp_path):
    """앱 저장소 run.sh와 같은 진입점. --port 0으로 띄우고 ready 줄에서 포트를 읽는다."""
    places = tmp_path / 'places.yaml'
    proc = subprocess.Popen([sys.executable, '-m', 'mr_phone_bridge.mock', '--scenario', 'default6',
                             '--host', '127.0.0.1', '--port', '0', '--places', str(places), '--no-console'],
                            cwd=PKG, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                            env={**os.environ, 'PYTHONUNBUFFERED': '1'})
    try:
        line = proc.stdout.readline()
        m = re.search(r'ready: (ws://127\.0\.0\.1:(\d+)/bridge/v1)', line)
        assert m, line + proc.stderr.read() if proc.poll() is not None else line

        async def body():
            c = await Client.connect(m.group(1))
            ack = await c.hello()
            assert ack['payload']['robot_id'] == 'mr-001' and len(c.catalog['places']) == 6
            await c.close()
        run(body())
        assert places.exists()
    finally:
        proc.send_signal(2)
        try:
            proc.wait(10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        proc.stdout.close()
        proc.stderr.close()
    assert proc.returncode == 0


def test_app_sos_raise_and_cancel(tmp_path):
    """앱 SOS 버튼(sos.raise) → manual sos.request → ack·위치 → '괜찮아요'(sos.cancel) → sos.clear(cancelled)."""
    async def body():
        async with scenario_bridge(tmp_path, 'default6') as h:
            c = await h.client()
            await c.hello()
            rid = await c.send('sos.raise', {'reason': 'manual'})
            assert (await c.reply(rid))['type'] == 'ack'
            req = await c.expect('sos.request')
            sid = req['payload']['sos_id']
            assert req['payload']['reason'] == 'manual'
            await c.send('sos.ack', {'sos_id': sid, 'received': True}, reply_to=req['id'])
            again = await c.send('sos.raise', {'reason': 'manual'})       # 열린 SOS가 있으면 ack만
            assert (await c.reply(again))['type'] == 'ack'
            cid = await c.send('sos.cancel', {'sos_id': sid})
            assert (await c.reply(cid))['type'] == 'ack'
            clear = await c.expect('sos.clear')
            assert clear['payload'] == {'sos_id': sid, 'resolution': 'cancelled'}
            late = await c.send('sos.cancel', {'sos_id': sid})
            assert (await c.reply(late))['payload']['code'] == 'INVALID_STATE'
            await c.close()
    run(body())
