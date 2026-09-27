"""ROS 어댑터: 변환표(순수), RosBackend(순수), .msg 상수 대조(ROS 필요)."""
import importlib.util
import math

import pytest

from mr_phone_bridge.protocol.envelope import is_uuid
from mr_phone_bridge.ros import enums
from mr_phone_bridge.ros.backend import RosBackend, sos_wire_id

HAVE_MSGS = importlib.util.find_spec('mr_care_msgs') is not None


def test_enum_round_trip_and_unknown():
    for table in (enums.MISSION, enums.TRACKING, enums.TRIP_COMMAND, enums.LOCATION_REASON, enums.LOCATION_SOURCE,
                  enums.SOS_REASON, enums.SOS_RESOLUTION):
        for k, v in table.items():
            assert enums.to_int(table, k) == v and enums.to_str(table, v) == k
        assert enums.to_str(table, 99) == 'unknown'
        with pytest.raises(ValueError):
            enums.to_int(table, 'bogus')
    assert enums.to_str(enums.SOS_SEVERITY, 0) == 'critical'
    assert enums.to_str(enums.LOCATION_FAILURE, 99) is None
    assert enums.to_int(enums.LOCATION_FAILURE, None) == 0


def test_enum_strings_match_contract():
    from mr_phone_bridge.protocol.envelope import schemas
    d = schemas()._schema['$defs']
    assert set(enums.MISSION) == set(d['robot.status']['properties']['payload']['properties']['mission_state']['enum'])
    assert set(enums.TRACKING) == set(d['robot.status']['properties']['payload']['properties']['tracking_state']['enum'])
    sos = d['sos.request']['properties']['payload']['properties']
    assert set(enums.SOS_REASON) == set(sos['reason']['enum'])
    assert set(enums.SOS_SEVERITY) == set(sos['severity']['enum'])
    clear = d['sos.clear']['properties']['payload']['properties']
    assert set(enums.SOS_RESOLUTION) == set(clear['resolution']['enum'])
    loc = d['location.update']['properties']['payload']['properties']
    assert set(enums.LOCATION_SOURCE) == set(loc['source']['enum'])
    assert set(enums.LOCATION_FAILURE) - {None} == set(loc['failure_reason']['oneOf'][0]['enum'])


@pytest.mark.skipif(not HAVE_MSGS, reason='mr_care_msgs not built/sourced')
def test_enum_values_match_msg_constants():
    from mr_care_msgs.msg import PhoneLocation, RobotAppStatus, SosClear, SosRequest, TripCommand
    for name, v in enums.MISSION.items():
        assert getattr(RobotAppStatus, f'MISSION_{name.upper()}') == v
    for name, v in enums.TRACKING.items():
        assert getattr(RobotAppStatus, f'TRACKING_{name.upper()}') == v
    for name, v in enums.TRIP_COMMAND.items():
        assert getattr(TripCommand, f'COMMAND_{name.upper()}') == v
    for name, v in enums.LOCATION_REASON.items():
        assert getattr(PhoneLocation, f'REASON_{name.upper()}') == v
    for name, v in enums.LOCATION_SOURCE.items():
        assert getattr(PhoneLocation, f'SOURCE_{name.upper()}') == v
    for name, v in enums.LOCATION_FAILURE.items():
        assert getattr(PhoneLocation, f'FAILURE_{(name or "none").upper()}') == v
    for name, v in enums.SOS_REASON.items():
        assert getattr(SosRequest, f'REASON_{name.upper()}') == v
    for name, v in enums.SOS_SEVERITY.items():
        assert getattr(SosRequest, f'SEVERITY_{name.upper()}') == v
    for name, v in enums.SOS_RESOLUTION.items():
        assert getattr(SosClear, f'RESOLUTION_{name.upper()}') == v


def test_sos_wire_id():
    u = '835fcafe-60ac-4517-9f06-50bcd2486b6c'
    assert sos_wire_id(u) == u and sos_wire_id(u.upper()) == u
    a = sos_wire_id('cli-test')
    assert is_uuid(a) and a == sos_wire_id('cli-test') and a != sos_wire_id('cli-test-2')
    assert is_uuid(sos_wire_id(''))


def test_snapshot_rules(clock):
    b = RosBackend(app_status_stale_s=6.0, marker_stale_s=1.0, clock=clock)
    s = b.snapshot()
    assert (s.bridge_state, s.mission_state, s.tracking_state) == ('degraded', 'unknown', 'unavailable')

    b.update_marker(1)
    s = b.snapshot()
    assert (s.bridge_state, s.mission_state, s.tracking_state) == ('ready', 'unknown', 'following')
    b.update_marker(0)
    assert b.snapshot().tracking_state == 'lost'
    clock.advance(1.5)
    assert b.snapshot().tracking_state == 'unavailable' and b.snapshot().bridge_state == 'degraded'

    b.update_app_status({'mission_state': 'active', 'tracking_state': 'following', 'trip_id': 'x',
                         'from_place_id': 'place_home', 'to_place_id': 'place_market', 'battery_percent': None,
                         'last_error': None})
    s = b.snapshot()
    assert (s.bridge_state, s.mission_state, s.to_place_id) == ('ready', 'active', 'place_market')
    clock.advance(6.5)
    s = b.snapshot()
    assert (s.bridge_state, s.mission_state) == ('degraded', 'unknown')
    # header.stamp 기준으로 이미 낡은 latched 메시지
    b.update_app_status({'mission_state': 'idle', 'tracking_state': 'unavailable'}, age_s=30.0)
    assert b.snapshot().mission_state == 'unknown'
    b.internal_error = 'boom'
    assert b.snapshot().bridge_state == 'error'


def test_outbox_and_sos_id_mapping():
    b = RosBackend()
    wire = b.remember_sos('cli-test')
    b.publish_sos_ack(sos_id=wire, app_received=True, retries=1, has_location=False, best_source='none')
    b.start_trip(request_id='r', trip_id='t', from_place_id='a', to_place_id='b')
    kind, d = b.outbox.get_nowait()
    assert kind == 'sos_ack' and d['sos_id'] == 'cli-test'
    kind, d = b.outbox.get_nowait()
    assert kind == 'trip_command' and d['command'] == 'start'


@pytest.mark.skipif(not HAVE_MSGS, reason='mr_care_msgs not built/sourced')
def test_msg_conversions():
    from mr_phone_bridge.ros.node import PhoneBridgeNode
    from builtin_interfaces.msg import Time
    none = {'reason': 'sos', 'sos_id': '835fcafe-60ac-4517-9f06-50bcd2486b6c', 'has_fix': False, 'lat': None,
            'lon': None, 'horizontal_accuracy_m': None, 'captured_at_ms': None, 'age_ms': None, 'source': 'none',
            'failure_reason': 'timeout'}
    m = PhoneBridgeNode.location_msg({'message_id': 'm', 'payload': none, 'low_confidence': False}, Time())
    assert math.isnan(m.lat) and math.isnan(m.horizontal_accuracy_m) and m.age_ms == -1 and m.captured_at_ms == 0
    assert m.source == 0 and m.failure_reason == 3 and m.reason == 2
    fix = {**none, 'has_fix': True, 'lat': 36.1, 'lon': 128.5, 'horizontal_accuracy_m': 1e40, 'captured_at_ms': 10**20,
           'age_ms': 5, 'source': 'fresh', 'failure_reason': None}
    m = PhoneBridgeNode.location_msg({'message_id': 'm', 'payload': fix, 'low_confidence': True}, Time())
    assert m.lat == 36.1 and m.source == 2 and m.failure_reason == 0 and m.captured_at_ms == 2**63 - 1

    from conftest import load_fixture
    cat = load_fixture('place-catalog-default6')['payload']
    cat['places'][0]['sort_order'] = 2**40
    m = PhoneBridgeNode.catalog_msg(cat, Time())
    assert len(m.places) == 6 and m.default_origin_place_id == 'place_home'
    assert m.places[0].sort_order == 2**31 - 1 and m.places[1].address == ''
