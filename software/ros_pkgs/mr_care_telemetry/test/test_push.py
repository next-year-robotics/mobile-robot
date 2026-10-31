"""보호자 웹 푸시(push.py): 받는 사람 찾기·FCM 메시지 모양·만료 토큰 정리·에이전트 연결. 네트워크 없이 돈다."""
from conftest import FakeClock, MemoryRtdb

from mr_care_telemetry.agent import AgentConfig, TelemetryAgent
from mr_care_telemetry.outbox import Outbox
from mr_care_telemetry.push import PushNotifier, josa_ro, sos_text, trip_start_text

TRIP = 'bbef019a-dd3a-437e-8f5c-388913ae9eca'
SOS = '835fcafe-60ac-4517-9f06-50bcd2486b6c'
LINK = 'https://senior-care-dashboard.vercel.app/'


class FakeSender:
    def __init__(self, replies=None):
        self.sent = []
        self.replies = replies or {}

    def send(self, message, *, validate_only=False):
        self.sent.append(message)
        return self.replies.get(message['token'], (200, ''))


def db_with_tokens():
    db = MemoryRtdb()
    db.put('access/mr-001', {'uid-a': 'guardian', 'uid-b': 'dev'})
    db.put('users/uid-a', {'email': 'a@x', 'push': {'k1': {'token': 'tok-a'}}})
    db.put('users/uid-b', {'email': 'b@x', 'push': {'k2': {'token': 'tok-b'}, 'k3': {'token': 'tok-dead'}}})
    db.put('users/uid-c', {'email': 'c@x', 'push': {'k4': {'token': 'tok-no-access'}}})
    return db


def test_text():
    assert josa_ro('시장') == '으로' and josa_ro('마트') == '로' and josa_ro('서울') == '로'
    assert trip_start_text('아버지', '시장') == ('외출 시작', '아버지께서 시장으로 외출을 시작하셨어요.')
    assert sos_text('어르신', 'manual')[1].startswith('어르신께서 도움을 요청하셨어요')


def test_delivers_to_guardians_with_access_and_drops_dead_tokens():
    db, clock = db_with_tokens(), FakeClock()
    sender = FakeSender({'tok-dead': (404, 'UNREGISTERED')})
    p = PushNotifier(db, sender, robot_id='mr-001', link=LINK, clock=clock)
    assert p.deliver(clock(), 'sos.request', 'SOS 긴급 알림', '본문', 'sos-1', True, {'sos_id': SOS}) == 2
    assert sorted(m['token'] for m in sender.sent) == ['tok-a', 'tok-b', 'tok-dead']
    m = sender.sent[0]
    assert m['webpush'] == {'headers': {'Urgency': 'high', 'TTL': '120'}}
    assert m['data'] == {'sos_id': SOS, 'kind': 'sos.request', 'robot_id': 'mr-001', 'title': 'SOS 긴급 알림',
                         'body': '본문', 'url': LINK + '?sos=' + SOS, 'tag': 'sos-1', 'urgent': '1'}
    assert all(isinstance(v, str) for v in m['data'].values())       # FCM data 값은 문자열만
    assert db.at('users/uid-b/push/k3') is None and db.at('users/uid-b/push/k2') is not None


def test_stale_and_no_subscribers():
    clock = FakeClock()
    sender = FakeSender()
    p = PushNotifier(MemoryRtdb(), sender, robot_id='mr-001', link=LINK, clock=clock)
    assert p.deliver(clock(), 'trip.start', 't', 'b', 'trip-1', False, {}) == 0
    old = clock()
    clock.advance(200)
    p.client = db_with_tokens()
    assert p.deliver(old, 'trip.start', 't', 'b', 'trip-1', False, {}) == 0
    assert sender.sent == []


class RecordingPush:
    def __init__(self):
        self.jobs = []

    def start(self):
        pass

    def stop(self):
        pass

    def notify(self, kind, title, body, **kw):
        self.jobs.append((kind, title, body, kw))


def test_agent_pushes_trip_start_and_sos_only():
    push = RecordingPush()
    a = TelemetryAgent(AgentConfig(senior_name='아버지'), MemoryRtdb(), Outbox(), clock=FakeClock(),
                       wall_ms=lambda: 1_790_000_000_000, push=push)
    a.on_place_catalog({'place_home': '집', 'place_market': '시장'})
    st = dict(battery_percent=None, last_error=None)
    a.on_app_status(mission_state='idle', tracking_state='idle', trip_id=None, from_place_id=None,
                    to_place_id=None, **st)
    a.on_app_status(mission_state='starting', tracking_state='searching', trip_id=TRIP, from_place_id='place_home',
                    to_place_id='place_market', **st)
    a.on_app_status(mission_state='active', tracking_state='following', trip_id=TRIP, from_place_id='place_home',
                    to_place_id='place_market', **st)
    a.on_sos_request(sos_id=SOS, reason='manual', severity='critical', trip_id=TRIP)
    a.on_sos_clear(sos_id=SOS, resolution='cancelled')
    a.on_app_status(mission_state='cancelled', tracking_state='idle', trip_id=TRIP, from_place_id='place_home',
                    to_place_id='place_market', **st)
    assert [j[0] for j in push.jobs] == ['trip.start', 'sos.request']
    assert push.jobs[0][2] == '아버지께서 시장으로 외출을 시작하셨어요.'
    assert push.jobs[1][3]['urgent'] is True and push.jobs[1][3]['data']['sos_id'] == SOS
