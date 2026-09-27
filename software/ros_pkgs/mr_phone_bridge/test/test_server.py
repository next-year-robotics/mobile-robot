"""websockets 서버 통합 시험. 실제 소켓으로 앱 흉내 클라이언트를 붙인다."""
import asyncio
import json

import pytest

try:
    import websockets  # noqa: F401
    HAVE_WS = True
except ImportError:          # ROS 셸(apt python3-websockets 미설치)에서는 이 모듈만 건너뛴다
    HAVE_WS = False
# 모듈 수준 importorskip은 ROS의 pytest 7 + launch_testing 플러그인에서 세션 전체를 건너뛰게 만든다.
pytestmark = pytest.mark.skipif(not HAVE_WS, reason='websockets not installed')

from conftest import load_fixture, run  # noqa: E402
if HAVE_WS:
    from wsclient import bridge, location_payload, place_payload, uid  # noqa: E402

from mr_phone_bridge.places.store import PlaceStore  # noqa: E402
from mr_phone_bridge.server.bridge import (CLOSE_HEARTBEAT_TIMEOUT, CLOSE_POLICY,  # noqa: E402
                                           CLOSE_SUPERSEDED, CLOSE_TOO_BIG)

REV = '2026-08-27T12:00:00Z'


def test_handshake_sequence(tmp_path):
    async def body():
        async with bridge(tmp_path) as h:
            c = await h.client()
            hid = await c.send('hello', load_fixture('hello')['payload'])
            ack = await c.recv()
            assert ack['type'] == 'hello.ack' and ack['reply_to'] == hid
            p = ack['payload']
            assert p['robot_id'] == 'mr-001' and p['catalog_revision'] == REV and p['max_places'] == 12
            assert p['heartbeat_interval_ms'] == 2000 and p['place_edit_enabled'] is True
            cat = await c.recv()
            assert cat['type'] == 'place.catalog'
            assert cat['payload'] == load_fixture('place-catalog-default6')['payload']
            st = await c.recv()
            assert st['type'] == 'robot.status'
            assert st['payload'] == load_fixture('robot-status-idle')['payload']
            link = h.published('/phone/link_status')
            assert link[0]['connected'] is False and link[-1]['connected'] is True
            await c.close()
    run(body())


def test_commands_before_hello_rejected(tmp_path):
    async def body():
        async with bridge(tmp_path) as h:
            c = await h.client()
            mid = await c.send('trip.start', {'from_place_id': 'place_home', 'to_place_id': 'place_market',
                                              'catalog_revision': REV})
            err = await c.reply(mid)
            assert err['type'] == 'error' and err['payload']['code'] == 'INVALID_STATE'
            assert h.published('/phone/trip_command') == []
            await c.close()
    run(body())


def test_heartbeat_and_timeout(tmp_path):
    async def body():
        async with bridge(tmp_path) as h:
            c = await h.client()
            await c.hello()
            for _ in range(3):
                r = await c.heartbeat()
                assert r['type'] == 'heartbeat.ack' and r['payload']['session_id'] == c.session_id
                await asyncio.sleep(0.3)
            wrong = await c.send('heartbeat', {'session_id': uid()})
            assert (await c.reply(wrong))['payload']['code'] == 'INVALID_STATE'
            # 이제 조용히 있으면 1 s 뒤 서버가 끊는다
            assert await c.closed(3.0) == CLOSE_HEARTBEAT_TIMEOUT
            assert h.published('/phone/link_status')[-1]['connected'] is False
            assert h.core.active is None
    run(body())


def test_hello_timeout_and_bad_path(tmp_path):
    async def body():
        async with bridge(tmp_path) as h:
            c = await h.client()
            assert await c.closed(3.0) == CLOSE_POLICY
            from wsclient import Client
            bad = await Client.connect(h.url.replace('/bridge/v1', '/other'))
            assert await bad.closed(3.0) == CLOSE_POLICY
    run(body())


def test_status_is_periodic(tmp_path):
    async def body():
        async with bridge(tmp_path) as h:
            c = await h.client()
            await c.hello()
            t0 = asyncio.get_running_loop().time()
            for _ in range(3):
                await c.expect('robot.status', timeout=1.0)
                await c.heartbeat()
            assert asyncio.get_running_loop().time() - t0 >= 0.7     # 0.4 s 주기
            await c.close()
    run(body())


def test_trip_start_cancel_flow(tmp_path):
    async def body():
        async with bridge(tmp_path) as h:
            c = await h.client()
            await c.hello()
            start = {'from_place_id': 'place_home', 'to_place_id': 'place_market', 'catalog_revision': REV}
            mid = await c.send('trip.start', start)
            ack = await c.reply(mid, timeout=1.0)                 # 1 s 안에 ack
            assert ack['type'] == 'ack'
            trip_id = ack['payload']['trip_id']
            await c.expect('robot.status', where=lambda m: m['payload']['mission_state'] == 'starting')
            st = await c.expect('robot.status', where=lambda m: m['payload']['mission_state'] == 'active')
            assert st['payload']['trip_id'] == trip_id and st['payload']['to_place_id'] == 'place_market'

            # 주행 중 두 번째 start는 거절
            again = await c.send('trip.start', start)
            assert (await c.reply(again))['payload']['code'] == 'INVALID_STATE'
            # 같은 id 재전송은 같은 ack, 부작용 없음
            dup = await c.send('trip.start', start, msg_id=mid)
            assert dup == mid
            assert (await c.reply(mid))['payload'] == ack['payload']
            assert len([d for d in h.published('/phone/trip_command') if d['command'] == 'START']) == 1

            wrong = await c.send('trip.cancel', {'trip_id': uid(), 'reason': 'user_request'})
            assert (await c.reply(wrong))['payload']['code'] == 'INVALID_STATE'
            cid = await c.send('trip.cancel', {'trip_id': trip_id, 'reason': 'user_request'})
            assert (await c.reply(cid))['payload'] == {'status': 'accepted'}
            await c.expect('robot.status', where=lambda m: m['payload']['mission_state'] == 'cancelling')
            await c.expect('robot.status', where=lambda m: m['payload']['mission_state'] == 'cancelled')
            await c.expect('robot.status', where=lambda m: m['payload']['mission_state'] == 'idle')
            cmds = h.published('/phone/trip_command')
            assert [d['command'] for d in cmds] == ['START', 'CANCEL']
            assert cmds[1]['trip_id'] == trip_id
            await c.close()
    run(body())


def test_cancel_right_after_start_is_accepted(tmp_path):
    """mission이 아직 반영하기 전(pending trip)에도 실행 취소가 된다."""
    async def body():
        from mr_phone_bridge.mock.backend import MockBackend
        from mr_phone_bridge.server.backend import MissionSnapshot

        class Recording(MockBackend):
            """명령을 기록만 하고 mission 상태는 idle 그대로 둔다(느린 mission 노드)."""

            def snapshot(self):
                return MissionSnapshot(mission_state='idle')

            def start_trip(self, **kw):
                self._record('/phone/trip_command', {'command': 'START', **kw})
        async with bridge(tmp_path, backend=Recording()) as h:
            c = await h.client()
            await c.hello()
            mid = await c.send('trip.start', {'from_place_id': 'place_home', 'to_place_id': 'place_market',
                                              'catalog_revision': REV})
            trip_id = (await c.reply(mid))['payload']['trip_id']
            again = await c.send('trip.start', {'from_place_id': 'place_home', 'to_place_id': 'place_mart',
                                                'catalog_revision': REV})
            assert (await c.reply(again))['payload']['code'] == 'INVALID_STATE'
            dl = await c.send('place.delete', {'place_id': 'place_market', 'catalog_revision': REV})
            assert (await c.reply(dl))['payload']['code'] == 'PLACE_IN_USE'
            cid = await c.send('trip.cancel', {'trip_id': trip_id, 'reason': 'user_request'})
            assert (await c.reply(cid))['type'] == 'ack'
            await c.close()
    run(body())


@pytest.mark.parametrize('payload,code', [
    ({'from_place_id': 'place_home', 'to_place_id': 'place_home', 'catalog_revision': REV}, 'INVALID_MESSAGE'),
    ({'from_place_id': 'place_home', 'to_place_id': 'place_moon', 'catalog_revision': REV}, 'UNKNOWN_PLACE'),
    ({'from_place_id': 'place_home', 'to_place_id': 'place_temple', 'catalog_revision': REV}, 'INVALID_STATE'),
])
def test_trip_start_rejections(tmp_path, payload, code):
    async def body():
        async with bridge(tmp_path) as h:
            c = await h.client()
            await c.hello()
            mid = await c.send('trip.start', payload)
            err = await c.reply(mid)
            assert err['type'] == 'error' and err['payload']['code'] == code
            await c.close()
    run(body())


def test_trip_start_stale_revision_sends_catalog(tmp_path):
    async def body():
        async with bridge(tmp_path) as h:
            c = await h.client()
            await c.hello()
            mid = await c.send('trip.start', {'from_place_id': 'place_home', 'to_place_id': 'place_market',
                                              'catalog_revision': '2020-01-01T00:00:00Z'})
            err = await c.reply(mid)
            assert err['payload'] == {'code': 'CATALOG_CONFLICT', 'message': err['payload']['message'],
                                      'retryable': True}
            await c.expect('place.catalog')
            await c.close()
    run(body())


def test_trip_start_not_ready_without_mission(tmp_path):
    async def body():
        from mr_phone_bridge.mock.backend import MockBackend
        from mr_phone_bridge.server.backend import MissionSnapshot

        class NoMission(MockBackend):
            def snapshot(self):
                return MissionSnapshot()          # mission_state=unknown
        async with bridge(tmp_path, backend=NoMission()) as h:
            c = await h.client()
            await c.hello()
            mid = await c.send('trip.start', {'from_place_id': 'place_home', 'to_place_id': 'place_market',
                                              'catalog_revision': REV})
            err = await c.reply(mid)
            assert err['payload']['code'] == 'NOT_READY' and err['payload']['retryable'] is True
            # 정지는 mission 상태를 몰라도 전달한다
            cid = await c.send('trip.cancel', {'trip_id': uid(), 'reason': 'user_request'})
            assert (await c.reply(cid))['type'] == 'ack'
            assert h.published('/phone/trip_command')[-1]['command'] == 'CANCEL'
            await c.close()
    run(body())


def test_place_upsert_delete_broadcast(tmp_path):
    async def body():
        async with bridge(tmp_path) as h:
            c = await h.client()
            await c.hello()
            mid = await c.send('place.upsert', place_payload(REV))
            ack = await c.reply(mid)
            assert ack['type'] == 'ack' and ack['payload']['place_id'] == 'place_hospital'
            rev = ack['payload']['catalog_revision']
            assert rev != REV
            cat = await c.expect('place.catalog')
            assert cat['payload']['revision'] == rev
            assert [p['id'] for p in cat['payload']['places']][-1] == 'place_hospital'
            assert PlaceStore(h.places_path).catalog.revision == rev          # 파일에 반영
            assert h.published('/robot/place_catalog')[-1]['revision'] == rev

            # 같은 id 재전송: 같은 ack, catalog 재방송 없음, 파일 변화 없음
            await c.send('place.upsert', place_payload(REV), msg_id=mid)
            assert (await c.reply(mid))['payload'] == ack['payload']

            old = await c.send('place.delete', {'place_id': 'place_temple', 'catalog_revision': REV})
            assert (await c.reply(old))['payload']['code'] == 'CATALOG_CONFLICT'
            await c.expect('place.catalog')
            dl = await c.send('place.delete', {'place_id': 'place_temple', 'catalog_revision': rev})
            dack = await c.reply(dl)
            assert dack['payload']['place_id'] == 'place_temple'
            cat = await c.expect('place.catalog', where=lambda m: m['payload']['revision'] ==
                                 dack['payload']['catalog_revision'])
            assert 'place_temple' not in [p['id'] for p in cat['payload']['places']]
            await c.close()
    run(body())


def test_place_edit_disabled(tmp_path):
    async def body():
        async with bridge(tmp_path, place_edit_enabled=False) as h:
            c = await h.client()
            ack = await c.hello()
            assert ack['payload']['place_edit_enabled'] is False
            mid = await c.send('place.upsert', place_payload(REV))
            assert (await c.reply(mid))['payload']['code'] == 'INVALID_STATE'
            await c.close()
    run(body())


def test_location_and_low_confidence(tmp_path):
    async def body():
        async with bridge(tmp_path) as h:
            c = await h.client()
            await c.hello()
            a = await c.send('location.update', location_payload('periodic', None, 'cache'))
            b = await c.send('location.update', location_payload('periodic', None, 'cache', age_ms=400000))
            d = await c.send('location.update', location_payload('periodic', None, 'fresh',
                                                                 horizontal_accuracy_m=80.0))
            n = await c.send('location.update', location_payload('periodic', None, 'none'))
            for mid in (a, b, d, n):
                assert (await c.reply(mid))['type'] == 'ack'
            locs = h.published('/phone/location')
            assert [x['low_confidence'] for x in locs] == [False, True, True, False]
            await c.close()
    run(body())


def test_bad_frames(tmp_path):
    async def body():
        async with bridge(tmp_path) as h:
            c = await h.client()
            await c.hello()
            await c.send_raw(b'\x00\x01')
            e = await c.expect('error')
            assert e['payload']['code'] == 'INVALID_MESSAGE' and e['reply_to'] is None
            await c.send_raw('{"broken"')
            assert (await c.expect('error'))['payload']['code'] == 'INVALID_MESSAGE'
            mid = await c.send('robot.dance', {})
            assert (await c.reply(mid))['payload']['code'] == 'UNSUPPORTED_TYPE'
            await c.send('error', {'code': 'INTERNAL_ERROR', 'message': 'app side', 'retryable': False})
            assert (await c.heartbeat())['type'] == 'heartbeat.ack'          # 여전히 연결됨
            await c.send_raw('x' * (17 * 1024))
            assert await c.closed(3.0) == CLOSE_TOO_BIG
    run(body())


def test_unsupported_version_closes(tmp_path):
    async def body():
        async with bridge(tmp_path) as h:
            c = await h.client()
            hello = load_fixture('hello')
            hello['version'] = 2
            await c.send_raw(json.dumps(hello))
            e = await c.recv()
            assert e['payload']['code'] == 'UNSUPPORTED_VERSION' and e['reply_to'] == hello['id']
            assert await c.closed(3.0) == CLOSE_POLICY
    run(body())


def test_new_session_supersedes_old(tmp_path):
    async def body():
        async with bridge(tmp_path) as h:
            a = await h.client()
            await a.hello()
            b = await h.client()
            await b.hello()
            assert await a.closed(3.0) == CLOSE_SUPERSEDED
            assert (await b.heartbeat())['type'] == 'heartbeat.ack'
            assert h.core.active is not None and h.core.active.session.session_id == b.session_id
            links = h.published('/phone/link_status')
            assert [x['connected'] for x in links] == [False, True, True]     # 사이에 끊김 없음
            await b.close()
    run(body())


def test_rate_limit(tmp_path):
    async def body():
        async with bridge(tmp_path, rate_limit_per_s=5.0, rate_limit_burst=10) as h:
            c = await h.client()
            await c.hello()
            for _ in range(30):
                try:
                    await c.send('heartbeat', {'session_id': c.session_id})
                except Exception:
                    break
            assert await c.closed(3.0) == CLOSE_POLICY
    run(body())


def test_sos_round_trip(tmp_path):
    async def body():
        async with bridge(tmp_path) as h:
            c = await h.client()
            await c.hello()
            sid = uid()
            await h.core.request_sos(sid, 'tracking_lost', 'critical', None)
            req = await c.expect('sos.request')
            assert req['payload'] == {'sos_id': sid, 'reason': 'tracking_lost', 'severity': 'critical',
                                      'trip_id': None}
            aid = await c.send('sos.ack', {'sos_id': sid, 'received': True}, reply_to=req['id'])
            assert (await c.reply(aid))['type'] == 'ack'
            lid = await c.send('location.update', location_payload('sos', sid, 'cache'))
            await c.reply(lid)
            fid = await c.send('location.update', location_payload('sos', sid, 'fresh'))
            await c.reply(fid)
            await asyncio.sleep(0.5)
            acks = h.published('/safety/sos_ack')
            assert [(a['app_received'], a['best_source']) for a in acks] == [(True, 'none'), (True, 'cache'),
                                                                             (True, 'fresh')]
            assert len([m for m in c.backlog if m['type'] == 'sos.request']) == 0   # 재전송 없음
            await h.core.clear_sos(sid, 'handled')
            clr = await c.expect('sos.clear')
            assert clr['payload'] == {'sos_id': sid, 'resolution': 'handled'}
            await c.close()
    run(body())


def test_sos_without_ack_retries_and_restores(tmp_path):
    async def body():
        async with bridge(tmp_path) as h:
            c = await h.client()
            await c.hello()
            sid = uid()
            await h.core.request_sos(sid, 'stuck', 'warning', None)
            reqs = []
            for _ in range(4):
                reqs.append(await c.expect('sos.request', timeout=1.0))
            assert len({r['id'] for r in reqs}) == 4 and {r['payload']['sos_id'] for r in reqs} == {sid}
            await asyncio.sleep(0.4)
            fail = h.published('/safety/sos_ack')
            assert fail == [{'sos_id': sid, 'app_received': False, 'retries': 3, 'has_location': False,
                             'best_source': 'none'}]
            await c.close()
            # 재연결하면 활성 SOS를 다시 보낸다
            c2 = await h.client()
            await c2.hello()
            again = await c2.expect('sos.request')
            assert again['payload']['sos_id'] == sid
            await c2.send('sos.ack', {'sos_id': sid, 'received': True}, reply_to=again['id'])
            await asyncio.sleep(0.2)
            assert h.published('/safety/sos_ack')[-1]['app_received'] is True
            await c2.close()
    run(body())


def test_sos_clear_while_offline_delivered_on_reconnect(tmp_path):
    async def body():
        async with bridge(tmp_path) as h:
            sid = uid()
            await h.core.request_sos(sid, 'tracking_lost', 'critical', None)
            await h.core.clear_sos(sid, 'tracking_recovered')
            c = await h.client()
            await c.hello()
            clr = await c.expect('sos.clear')
            assert clr['payload']['sos_id'] == sid
            await asyncio.sleep(0.3)
            assert not [m for m in c.backlog if m['type'] == 'sos.request']
            await c.close()
    run(body())


def test_request_sos_validates_ids(tmp_path):
    async def body():
        async with bridge(tmp_path) as h:
            with pytest.raises(ValueError):
                await h.core.request_sos('not-a-uuid', 'tracking_lost', 'critical', None)
            sid = uid()
            await h.core.request_sos(sid, 'alien', 'huge', 'not-a-trip')
            s = h.core.sos.active[sid]
            assert (s.reason, s.severity, s.trip_id) == ('unknown', 'critical', None)
    run(body())
