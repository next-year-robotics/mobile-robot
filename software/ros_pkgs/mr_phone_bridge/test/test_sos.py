"""SosManager 단위 시험(가짜 시계). websockets 없이 돈다."""
import pytest
from conftest import run

from mr_phone_bridge.protocol.envelope import check_outgoing, new_id
from mr_phone_bridge.server.sos import SosManager


class Env:
    def __init__(self, clock, connected=True):
        self.clock = clock
        self.connected = connected
        self.sent = []
        self.acks = []
        self.mgr = SosManager(send=self.send, connected=lambda: self.connected, publish_ack=self.publish,
                              retry_interval_s=2.0, max_retries=3, clock=clock)

    async def send(self, msg):
        assert check_outgoing(msg) is None
        if not self.connected:
            return False
        self.sent.append(msg)
        return True

    def publish(self, **kw):
        self.acks.append(kw)

    def types(self):
        return [m['type'] for m in self.sent]

    async def ticks(self, seconds, step=0.5):
        for _ in range(int(seconds / step)):
            self.clock.advance(step)
            await self.mgr.tick()


@pytest.fixture
def env(clock):
    return Env(clock)


def test_retries_then_failure(env):
    async def body():
        sid = new_id()
        await env.mgr.request(sid, 'tracking_lost', 'critical', None)
        await env.ticks(20)
        assert env.types() == ['sos.request'] * 4          # 첫 전송 + 재시도 3
        assert len({m['id'] for m in env.sent}) == 4        # envelope id는 매번 새로
        assert {m['payload']['sos_id'] for m in env.sent} == {sid}
        assert env.acks == [dict(sos_id=sid, app_received=False, retries=3, has_location=False,
                                 best_source='none')]
    run(body())


def test_ack_stops_retries_and_location_updates(env):
    async def body():
        sid = new_id()
        await env.mgr.request(sid, 'tracking_lost', 'critical', None)
        await env.ticks(2.5)                                # 재시도 1회
        env.mgr.on_location(sid, 'cache')                   # ACK 전 위치: 기록만
        assert env.acks == []
        assert env.mgr.on_ack(sid, True)
        await env.ticks(10)
        assert env.types() == ['sos.request'] * 2
        assert env.acks[-1] == dict(sos_id=sid, app_received=True, retries=1, has_location=True,
                                    best_source='cache')
        env.mgr.on_location(sid, 'cache')                   # 같은 품질: 발행 없음
        env.mgr.on_location(sid, 'fresh')
        env.mgr.on_location(sid, 'none')
        assert [a['best_source'] for a in env.acks] == ['cache', 'fresh']
        assert env.mgr.on_ack(sid, True) and len(env.acks) == 2   # 중복 ACK
    run(body())


def test_duplicate_request_ignored(env):
    async def body():
        sid = new_id()
        assert await env.mgr.request(sid, 'stuck', 'warning', None)
        assert not await env.mgr.request(sid, 'stuck', 'warning', None)
        assert len(env.sent) == 1
    run(body())


def test_disconnected_counts_attempts_and_restores(env):
    async def body():
        env.connected = False
        sid = new_id()
        await env.mgr.request(sid, 'tracking_lost', 'critical', None)
        await env.ticks(10)
        assert env.sent == [] and env.acks[0]['app_received'] is False
        env.connected = True
        await env.mgr.on_session_open()                     # 재연결: 활성 SOS 복원
        assert env.types() == ['sos.request']
        env.mgr.on_ack(sid, True)
        assert env.acks[-1]['app_received'] is True
    run(body())


def test_clear_while_disconnected_is_delivered_later(env):
    async def body():
        sid = new_id()
        await env.mgr.request(sid, 'tracking_lost', 'critical', None)
        env.mgr.on_ack(sid, True)
        env.connected = False
        await env.mgr.clear(sid, 'tracking_recovered')
        assert sid not in env.mgr.active
        env.connected = True
        await env.mgr.on_session_open()
        assert env.types() == ['sos.request', 'sos.clear']
        assert env.sent[-1]['payload'] == {'sos_id': sid, 'resolution': 'tracking_recovered'}
        await env.mgr.on_session_open()                     # 한 번만
        assert env.types().count('sos.clear') == 1
    run(body())


def test_resend_does_not_count(env):
    async def body():
        sid = new_id()
        await env.mgr.request(sid, 'tracking_lost', 'critical', None)
        await env.mgr.resend(sid)
        await env.mgr.resend(sid)
        assert env.mgr.active[sid].retries == 0 and len(env.sent) == 3
    run(body())
