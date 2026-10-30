"""model·outbox·rtdb 단위 시험. ROS·에뮬레이터 없이 돈다."""
import base64
import importlib.util
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs

import pytest
from conftest import contracts_dir, load_json

from mr_care_telemetry import model as M
from mr_care_telemetry.outbox import Outbox
from mr_care_telemetry.rtdb import RtdbClient, RtdbError, ServiceAccountToken, namespace_of

DB = 'https://senior-care-nyr-default-rtdb.asia-southeast1.firebasedatabase.app'


# ------------------------------------------------------------------ model

@pytest.mark.parametrize('word,josa', [('시장', '으로'), ('마트', '로'), ('노인정', '으로'), ('서울', '로'),
                                       ('절', '로'), ('교회', '로'), ('market', '(으)로')])
def test_josa(word, josa):
    assert M.josa_euro(word) == josa


def test_trip_message_matches_fixture():
    fx = load_json(contracts_dir() / 'fixtures' / 'event-trip-start.json')
    assert M.trip_start_message('집', '시장') == fx['message']


def test_topic_tree_nests_ros_names():
    tree = M.topic_tree({'/base/status': {'last_seen_ms': 1, 'hz': 2.0}, '/joint_states': {'last_seen_ms': 0, 'hz': 0},
                         '/robot/app_status': {'last_seen_ms': 3, 'hz': 0.5}})
    assert tree == {'base': {'status': {'last_seen_ms': 1, 'hz': 2.0}},
                    'joint_states': {'last_seen_ms': 0, 'hz': 0},
                    'robot': {'app_status': {'last_seen_ms': 3, 'hz': 0.5}}}
    with pytest.raises(ValueError):                  # README 3.5: 한 토픽이 다른 토픽의 앞부분 경로면 안 된다
        M.topic_tree({'/robot': {'last_seen_ms': 1, 'hz': 1}, '/robot/app_status': {'last_seen_ms': 1, 'hz': 1}})
    with pytest.raises(ValueError):
        M.topic_tree({'/robot/app_status': {'last_seen_ms': 1, 'hz': 1}, '/robot': {'last_seen_ms': 1, 'hz': 1}})
    from mr_care_telemetry.agent import DIAG_TOPICS
    M.topic_tree({t: {'last_seen_ms': 0, 'hz': 0} for t in DIAG_TOPICS})


def test_push_ids_sort_by_time_and_within_ms():
    p = M.PushIds()
    ids = [p(1790000000000), p(1790000000000), p(1790000000000), p(1790000000001), p(1789999999999)]
    assert all(len(i) == 20 for i in ids)
    assert ids[0] < ids[1] < ids[2] < ids[3]
    assert ids[4] < ids[0]
    assert len(set(ids)) == 5


def test_accuracy_bucket():
    assert M.accuracy_bucket(False, None, False) == 'none'
    assert M.accuracy_bucket(True, 4.2, False) == 'good'
    assert M.accuracy_bucket(True, 30, False) == 'fair'
    assert M.accuracy_bucket(True, 80, False) == 'poor'
    assert M.accuracy_bucket(True, 4.2, True) == 'poor'


def test_sos_wire_id_matches_bridge():
    for rid in ('cli-test', 'test1', '835fcafe-60ac-4517-9f06-50bcd2486b6c', ''):
        assert M.UUID_RE.match(M.sos_wire_id(rid))
    if importlib.util.find_spec('mr_phone_bridge') is None:
        pytest.skip('mr_phone_bridge not importable')
    from mr_phone_bridge.ros.backend import sos_wire_id
    for rid in ('cli-test', 'test1', 'ABC', '835FCAFE-60AC-4517-9F06-50BCD2486B6C', ''):
        assert M.sos_wire_id(rid) == sos_wire_id(rid)


def test_safe_key_and_finite():
    assert M.safe_key('a.b$c#d[e]f/g') == 'a_b_c_d_e_f_g'
    assert M.finite_or_none(float('nan')) is None and M.finite_or_none('x') is None and M.finite_or_none(3) == 3.0


def test_contract_fixtures_valid(schemas):
    for fx in sorted((contracts_dir() / 'fixtures').glob('*.json')):
        name = fx.stem.split('-')[0]
        schemas.check(name, load_json(fx))


# ------------------------------------------------------------------ outbox

def test_outbox_order_and_persistence(tmp_path):
    q = Outbox(tmp_path / 'q.db')
    for i in range(5):
        q.put('PUT', f'a/{i}', {'i': i, '한글': '값'})
    ops = q.peek(3)
    assert [o.path for o in ops] == ['a/0', 'a/1', 'a/2'] and ops[0].body == {'i': 0, '한글': '값'}
    q.done(ops[0].id)
    q.failed(ops[1].id)
    q.close()
    q = Outbox(tmp_path / 'q.db')                       # 재기동해도 남아 있다
    ops = q.peek(10)
    assert [o.path for o in ops] == ['a/1', 'a/2', 'a/3', 'a/4'] and ops[0].attempts == 1
    q.put('DELETE', 'a/9')
    assert q.peek(10)[-1].body is None and len(q) == 5


def test_outbox_bounded():
    q = Outbox(max_ops=3)
    for i in range(5):
        q.put('PUT', f'a/{i}', i)
    assert [o.path for o in q.peek(10)] == ['a/2', 'a/3', 'a/4']


def test_outbox_rejects_nan():
    with pytest.raises(ValueError):
        Outbox().put('PUT', 'x', {'v': float('nan')})


# ------------------------------------------------------------------ rtdb

def test_urls():
    assert namespace_of(DB) == 'senior-care-nyr-default-rtdb'
    emu = RtdbClient(DB, emulator_host='127.0.0.1:9100')
    assert emu.url('robots/mr-001/status', {'print': 'silent'}) == \
        'http://127.0.0.1:9100/robots/mr-001/status.json?print=silent&ns=senior-care-nyr-default-rtdb'
    with pytest.raises(ValueError):
        RtdbClient(DB)                                   # 실제 DB에는 서비스 계정이 필요


class FakeGoogle(BaseHTTPRequestHandler):
    """토큰 엔드포인트 + RTDB REST 흉내. JWT 서명을 공개키로 검증한다."""
    public_key = None
    tokens_issued = 0
    writes = []
    status_override = {}

    def log_message(self, *a):
        pass

    def _reply(self, code, body=b''):
        self.send_response(code)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path == '/token-denied':
            self.rfile.read(int(self.headers['Content-Length']))
            self._reply(400, b'{"error": "invalid_grant", "error_description": "Invalid JWT Signature."}')
            return
        from cryptography.hazmat.primitives import hashes
        from cryptography.hazmat.primitives.asymmetric import padding
        form = parse_qs(self.rfile.read(int(self.headers['Content-Length'])).decode())
        assert form['grant_type'] == ['urn:ietf:params:oauth:grant-type:jwt-bearer']
        head, claims, sig = form['assertion'][0].split('.')
        pad = lambda s: s + '=' * (-len(s) % 4)  # noqa: E731
        FakeGoogle.public_key.verify(base64.urlsafe_b64decode(pad(sig)), f'{head}.{claims}'.encode(),
                                     padding.PKCS1v15(), hashes.SHA256())
        c = json.loads(base64.urlsafe_b64decode(pad(claims)))
        assert c['iss'] == 'telemetry@test.iam.gserviceaccount.com' and 'firebase.database' in c['scope']
        assert c['exp'] - c['iat'] == 3600
        FakeGoogle.tokens_issued += 1
        self._reply(200, json.dumps({'access_token': f'tok{FakeGoogle.tokens_issued}',
                                     'expires_in': 3600}).encode())

    def do_PUT(self):
        body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        FakeGoogle.writes.append((self.path, self.headers['Authorization'], body))
        code = FakeGoogle.status_override.pop(self.path.split('?')[0], 204)
        self._reply(code, b'{"error":"x"}' if code >= 400 else b'')


@pytest.fixture
def fake_google(tmp_path):
    from cryptography.hazmat.primitives import serialization
    from cryptography.hazmat.primitives.asymmetric import rsa
    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    FakeGoogle.public_key = key.public_key()
    FakeGoogle.tokens_issued = 0
    FakeGoogle.writes = []
    FakeGoogle.status_override = {}
    srv = HTTPServer(('127.0.0.1', 0), FakeGoogle)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f'http://127.0.0.1:{srv.server_address[1]}'
    sa = tmp_path / 'sa.json'
    sa.write_text(json.dumps({
        'type': 'service_account', 'client_email': 'telemetry@test.iam.gserviceaccount.com',
        'private_key': key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                         serialization.NoEncryption()).decode(),
        'token_uri': f'{base}/token'}))
    yield base, sa
    srv.shutdown()


def test_service_account_jwt_flow(fake_google):
    base, sa = fake_google
    client = RtdbClient(base, namespace='x', token=ServiceAccountToken(str(sa)))
    client.put('robots/mr-001/status', {'a': 1})
    client.put('robots/mr-001/meta', {'b': 2})
    assert FakeGoogle.tokens_issued == 1                        # 캐시
    path, auth, body = FakeGoogle.writes[0]
    assert path == '/robots/mr-001/status.json?print=silent' and auth == 'Bearer tok1' and body == {'a': 1}
    FakeGoogle.status_override['/robots/mr-001/status.json'] = 401
    with pytest.raises(RtdbError) as ei:
        client.put('robots/mr-001/status', {'a': 2})
    assert ei.value.status == 401 and ei.value.transient
    client.put('robots/mr-001/status', {'a': 3})               # 401 뒤에는 새 토큰
    assert FakeGoogle.tokens_issued == 2 and FakeGoogle.writes[-1][1] == 'Bearer tok2'
    FakeGoogle.status_override['/robots/mr-001/status.json'] = 400
    with pytest.raises(RtdbError) as ei:
        client.put('robots/mr-001/status', {'a': 4})
    assert ei.value.status == 400 and not ei.value.transient
    assert str(ei.value) == 'PUT robots/mr-001/status -> 400 x'                 # 본문의 error 문구만 한 줄로
    FakeGoogle.status_override['/robots/mr-001/status.json'] = 404            # DB URL 오류: 큐를 버리지 않는다
    with pytest.raises(RtdbError) as ei:
        client.put('robots/mr-001/status', {'a': 5})
    assert ei.value.status == 404 and ei.value.transient


def test_token_endpoint_rejection_is_transient(fake_google, tmp_path):
    """토큰 발급 거절(키 오류·시계 어긋남)은 쓰려던 작업의 잘못이 아니다. 큐에서 버리면 안 된다."""
    base, sa = fake_google
    info = json.loads(sa.read_text())
    info['token_uri'] = f'{base}/token-denied'
    bad = tmp_path / 'bad.json'
    bad.write_text(json.dumps(info))
    client = RtdbClient(base, namespace='x', token=ServiceAccountToken(str(bad)))
    with pytest.raises(RtdbError) as ei:
        client.put('robots/mr-001/status', {'a': 1})
    assert ei.value.transient and ei.value.status == 400 and 'invalid_grant' in str(ei.value)


def test_missing_service_account_is_transient(tmp_path):
    client = RtdbClient(DB, token=ServiceAccountToken(str(tmp_path / 'nope.json')))
    with pytest.raises(RtdbError) as ei:
        client.put('robots/mr-001/status', {})
    assert ei.value.transient and 'not found' in str(ei.value)


def test_unreachable_is_transient():
    client = RtdbClient(DB, emulator_host='127.0.0.1:9', timeout_s=1.0)
    with pytest.raises(RtdbError) as ei:
        client.put('x', 1)
    assert ei.value.transient
    with pytest.raises(RtdbError) as ei:
        client.put('x', float('nan'))
    assert not ei.value.transient
