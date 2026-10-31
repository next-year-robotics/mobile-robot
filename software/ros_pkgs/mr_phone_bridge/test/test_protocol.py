"""envelope·fixture·dedupe·session. websockets 없이 돈다."""
import json

import pytest
from conftest import load_fixture

from mr_phone_bridge.protocol import contracts_path
from mr_phone_bridge.protocol import errors as E
from mr_phone_bridge.protocol.dedupe import DedupeCache
from mr_phone_bridge.protocol.envelope import (CLIENT_TYPES, SERVER_TYPES, check_outgoing, encode, is_uuid, make,
                                               make_error, parse_frame)
from mr_phone_bridge.protocol.errors import ProtocolError
from mr_phone_bridge.protocol.session import ServerInfo, Session, State

FIXTURES = sorted(p.stem for p in (contracts_path() / 'fixtures').glob('*.json'))


def test_fixture_count():
    assert len(FIXTURES) == 28


@pytest.mark.parametrize('name', FIXTURES)
def test_fixture_round_trip(name):
    """앱→서버 fixture는 parse_frame을, 서버→앱 fixture는 발신 검사를 통과한다. 재인코딩해도 같다."""
    msg = load_fixture(name)
    again = json.loads(encode(msg))
    assert again == msg
    if msg['type'] in SERVER_TYPES:
        assert check_outgoing(msg) is None
    if msg['type'] in CLIENT_TYPES:
        parsed = parse_frame(encode(msg))
        assert parsed.type == msg['type']
        assert parsed.id == msg['id']
        assert parsed.payload == msg['payload']


def _frame(**over):
    msg = load_fixture('trip-start')
    msg.update(over)
    return json.dumps(msg)


def _err(data) -> ProtocolError:
    with pytest.raises(ProtocolError) as ei:
        parse_frame(data)
    return ei.value


def test_binary_frame_rejected():
    assert _err(b'{}').code == E.INVALID_MESSAGE


def test_malformed_json():
    e = _err('{"version":1,')
    assert e.code == E.INVALID_MESSAGE and e.reply_to is None


@pytest.mark.parametrize('const', ['NaN', 'Infinity', '-Infinity'])
def test_nan_infinity_rejected(const):
    raw = encode(load_fixture('location-cache')).replace('36.1234', const)
    assert const in raw
    assert _err(raw).code == E.INVALID_MESSAGE


def test_frame_size_limit():
    msg = load_fixture('trip-cancel')
    msg['payload']['reason'] = 'x' * (16 * 1024)
    e = _err(json.dumps(msg))
    assert e.code == E.INVALID_MESSAGE and e.close


def test_non_object():
    assert _err('[1,2]').code == E.INVALID_MESSAGE


def test_unsupported_version_closes():
    e = _err(_frame(version=2))
    assert e.code == E.UNSUPPORTED_VERSION and e.close
    assert e.reply_to == load_fixture('trip-start')['id']


def test_bad_uuid_is_invalid_and_reply_to_null():
    e = _err(_frame(id='E87864C5-3C72-42F8-9565-CF9219AA7CF2'))
    assert e.code == E.INVALID_MESSAGE and e.reply_to is None


def test_unknown_type():
    e = _err(_frame(type='robot.dance'))
    assert e.code == E.UNSUPPORTED_TYPE and is_uuid(e.reply_to)


def test_server_only_type_from_client():
    assert _err(encode(load_fixture('robot-status'))).code == E.UNSUPPORTED_TYPE


def test_missing_payload_field():
    msg = load_fixture('trip-start')
    del msg['payload']['to_place_id']
    e = _err(json.dumps(msg))
    assert e.code == E.INVALID_MESSAGE and 'to_place_id' in e.message


def test_out_of_range():
    msg = load_fixture('place-upsert')
    msg['payload']['lat'] = 91
    assert _err(json.dumps(msg)).code == E.INVALID_MESSAGE


def test_location_fix_consistency():
    msg = load_fixture('location-none')
    msg['payload']['lat'] = 36.0
    assert _err(json.dumps(msg)).code == E.INVALID_MESSAGE
    msg = load_fixture('location-periodic')
    msg['payload']['sos_id'] = msg['id']
    assert _err(json.dumps(msg)).code == E.INVALID_MESSAGE


def test_unknown_fields_ignored():
    msg = load_fixture('trip-start')
    msg['x_future'] = {'a': 1}
    msg['payload']['x_new'] = True
    assert parse_frame(json.dumps(msg)).type == 'trip.start'


def test_label_length_counts_characters():
    msg = load_fixture('place-upsert')
    msg['payload']['label'] = '가' * 20
    parse_frame(json.dumps(msg, ensure_ascii=False))
    msg['payload']['label'] = '가' * 21
    assert _err(json.dumps(msg, ensure_ascii=False)).code == E.INVALID_MESSAGE


def test_lone_surrogate_rejected():
    raw = encode(load_fixture('place-upsert')).replace('노인정', '\\ud800')
    assert _err(raw).code == E.INVALID_MESSAGE


def test_make_and_error_are_contract_valid():
    ack = make('ack', {'status': 'accepted'}, reply_to=load_fixture('trip-cancel')['id'])
    assert check_outgoing(ack) is None
    for code in E.ALL_CODES:
        err = make_error(ProtocolError(code, 'x'), reply_to=None)
        assert check_outgoing(err) is None
        assert err['payload']['retryable'] is E.DEFAULT_RETRYABLE[code]


def test_error_message_truncated():
    assert len(ProtocolError(E.INTERNAL_ERROR, 'x' * 1000).message) == E.MAX_MESSAGE_CHARS


# ------------------------------------------------------------------ dedupe

def test_dedupe_ttl(clock):
    c = DedupeCache(ttl_s=600, clock=clock)
    c.put('a', ['f1'])
    clock.advance(599)
    assert c.get('a') == ('f1',)
    clock.advance(2)
    assert c.get('a') is None


def test_dedupe_bounded(clock):
    c = DedupeCache(ttl_s=600, max_entries=3, clock=clock)
    for k in 'abcd':
        c.put(k, [k])
    assert c.get('a') is None and c.get('d') == ('d',) and len(c) == 3


# ------------------------------------------------------------------ session

INFO = ServerInfo(robot_id='mr-001', server_version='0.1.0')


def test_session_hello_and_heartbeat(clock):
    s = Session(opened_at=clock())
    hello = parse_frame(encode(load_fixture('hello')))
    ack = s.accept_hello(hello, INFO, '2026-08-27T12:00:00Z')
    assert check_outgoing(ack) is None
    assert ack['reply_to'] == hello.id and s.state is State.ACTIVE
    assert ack['payload']['heartbeat_timeout_ms'] == 10000
    with pytest.raises(ProtocolError) as ei:
        s.accept_hello(hello, INFO, 'r')
    assert ei.value.code == E.INVALID_STATE

    hb = load_fixture('heartbeat')
    with pytest.raises(ProtocolError):
        s.accept_heartbeat(parse_frame(encode(hb)))      # 다른 session_id
    hb['payload']['session_id'] = s.session_id
    reply = s.accept_heartbeat(parse_frame(encode(hb)))
    assert reply['type'] == 'heartbeat.ack' and reply['payload']['session_id'] == s.session_id


def test_session_timeouts(clock):
    s = Session(opened_at=clock())
    clock.advance(4.9)
    assert not s.hello_expired(clock(), 5.0)
    clock.advance(0.2)
    assert s.hello_expired(clock(), 5.0)
    s.accept_hello(parse_frame(encode(load_fixture('hello'))), INFO, 'r')
    s.touch(clock())
    clock.advance(9.9)
    assert not s.heartbeat_expired(clock(), 10000)
    clock.advance(0.2)
    assert s.heartbeat_expired(clock(), 10000)
