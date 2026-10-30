"""mr_care_telemetry 노드: ROS 토픽 → 에이전트 → (메모리) RTDB. 격리 도메인 + localhost 발견."""
import importlib.util
import os
import threading
import time

import pytest

MISSING = [m for m in ('rclpy', 'mr_care_msgs', 'mr_msgs') if importlib.util.find_spec(m) is None]
pytestmark = pytest.mark.skipif(bool(MISSING), reason=f'missing {MISSING}')
TEST_DOMAIN_ID = 100 + (os.getpid() + 71) % 100


def test_enum_tables_from_msg_constants():
    from mr_care_telemetry import node as N
    assert N.MISSION[0] == 'idle' and N.MISSION[255] == 'unknown' and N.MISSION[4] == 'cancelling'
    assert N.TRACKING[2] == 'following' and N.SOS_REASON[1] == 'tracking_lost' and N.SOS_SEVERITY[2] == 'warning'
    assert N.SOS_RESOLUTION[1] == 'tracking_recovered' and N.LOC_SOURCE == {0: 'none', 1: 'cache', 2: 'fresh'}
    assert N.TRIP_COMMAND == {1: 'start', 2: 'cancel'} and N.LOC_REASON == {1: 'periodic', 2: 'sos'}
    assert N.BASE_AGENT == {0: 'waiting', 1: 'available', 2: 'connected', 3: 'disconnected'}


def test_node_topics_to_rtdb(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    import rclpy
    from builtin_interfaces.msg import Time
    from conftest import MemoryRtdb
    from mr_care_msgs.msg import (PhoneLinkStatus, PhoneLocation, Place, PlaceCatalog, RobotAppStatus, SosAck,
                                  SosClear, SosRequest, TripCommand)
    from mr_msgs.msg import BaseStatus
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    from mr_care_telemetry.model import sos_wire_id
    from mr_care_telemetry.node import BASE_QOS, LATCHED, RELIABLE, TelemetryNode
    from mr_care_telemetry.outbox import Outbox

    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=TEST_DOMAIN_ID)
    db = MemoryRtdb()
    tel = TelemetryNode(client=db, outbox=Outbox(), context=ctx,
                        parameter_overrides=[Parameter('status_period_s', value=0.5)])
    tel.agent.cfg.loop_s = 0.05
    probe = Node('probe', context=ctx)
    pubs = {
        'status': probe.create_publisher(RobotAppStatus, '/robot/app_status', LATCHED),
        'link': probe.create_publisher(PhoneLinkStatus, '/phone/link_status', LATCHED),
        'catalog': probe.create_publisher(PlaceCatalog, '/robot/place_catalog', LATCHED),
        'trip': probe.create_publisher(TripCommand, '/phone/trip_command', RELIABLE),
        'loc': probe.create_publisher(PhoneLocation, '/phone/location', RELIABLE),
        'sos': probe.create_publisher(SosRequest, '/safety/sos_request', RELIABLE),
        'ack': probe.create_publisher(SosAck, '/safety/sos_ack', RELIABLE),
        'clear': probe.create_publisher(SosClear, '/safety/sos_clear', RELIABLE),
        'base': probe.create_publisher(BaseStatus, '/base/status', BASE_QOS),
    }
    ex = MultiThreadedExecutor(context=ctx)
    ex.add_node(tel)
    ex.add_node(probe)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    def events():
        ev = db.at('robots/mr-001/events') or {}
        return [ev[k] for k in sorted(ev)]

    def wait(pred, timeout=8.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            v = pred()
            if v:
                return v
            time.sleep(0.05)
        raise AssertionError(f'timeout; events={[e["kind"] for e in events()]} tree={db.tree}')

    def status(mission, tracking=0, trip=''):
        m = RobotAppStatus()
        m.header.stamp = tel.get_clock().now().to_msg()
        m.mission_state, m.tracking_state, m.trip_id = mission, tracking, trip
        m.from_place_id, m.to_place_id = ('place_home', 'place_market') if trip else ('', '')
        m.battery_percent = -1
        pubs['status'].publish(m)

    def link(connected):
        m = PhoneLinkStatus()
        m.connected, m.app_instance_id, m.session_id = connected, 'app-1', 's-1' if connected else ''
        m.last_seen = Time(sec=1790000000, nanosec=0)
        pubs['link'].publish(m)

    try:
        cat = PlaceCatalog()
        cat.places = [Place(id='place_home', label='집'), Place(id='place_market', label='시장')]
        pubs['catalog'].publish(cat)
        pubs['base'].publish(BaseStatus(agent_state=2, cmd_watchdog_ok=True))
        status(0)
        link(False)
        # 브리지(= /phone/link_status 발행자) 살아 있음 → bridge_state ready
        st = wait(lambda: (db.at('robots/mr-001/status') or {}).get('bridge_state') == 'ready'
                  and db.at('robots/mr-001/status'))
        assert st['mission_state'] == 'idle' and st['base'] == {**st['base'], 'agent_state': 'connected',
                                                                   'cmd_watchdog_ok': True}
        # 기준값(연결 없음)이 에이전트에 들어간 뒤에 연결을 알린다. 그래프 발견이 reader 매칭보다 먼저일 수 있어서.
        wait(lambda: tel.agent.link is not None and tel.agent.link['connected'] is False)
        link(True)
        trip = 'bbef019a-dd3a-437e-8f5c-388913ae9eca'
        status(1, 1, trip)
        status(2, 2, trip)
        wait(lambda: 'trip.start' in [e['kind'] for e in events()])
        sos = SosRequest()
        sos.sos_id, sos.reason, sos.severity = 'cli-test', 2, 2
        pubs['sos'].publish(sos)
        wire = sos_wire_id('cli-test')
        wait(lambda: db.at(f'robots/mr-001/sos/{wire}'))
        pubs['ack'].publish(SosAck(sos_id='cli-test', app_received=True, retries=0, has_location=False, best_source=0))
        loc = PhoneLocation(message_id='m', reason=2, sos_id=wire, has_fix=True, lat=36.1234, lon=128.5678,
                            horizontal_accuracy_m=4.2, captured_at_ms=1790000000000, age_ms=100, source=1,
                            failure_reason=0, low_confidence=False)
        pubs['loc'].publish(loc)
        node = wait(lambda: (db.at(f'robots/mr-001/sos/{wire}') or {}).get('location') and
                    db.at(f'robots/mr-001/sos/{wire}'))
        assert node['reason'] == 'stuck' and node['severity'] == 'warning' and node['app_received'] is True
        wait(lambda: 'sos.ack' in [e['kind'] for e in events()])
        pubs['clear'].publish(SosClear(sos_id='cli-test', resolution=2))
        wait(lambda: db.at(f'robots/mr-001/sos/{wire}') is None and 'sos.clear' in [e['kind'] for e in events()])
        pubs['trip'].publish(TripCommand(request_id='r', trip_id=trip, command=2, reason='user_request'))
        time.sleep(0.2)
        status(4, 2, trip)
        status(5, 0, trip)
        wait(lambda: 'trip.cancelled' in [e['kind'] for e in events()])
        kinds = [e['kind'] for e in events()]
        assert kinds == ['agent.online', 'link.connected', 'trip.start', 'sos.request', 'sos.ack', 'sos.clear',
                         'trip.cancelled']
        ev = {e['kind']: e for e in events()}
        assert ev['trip.start']['message'] == '집에서 시장으로 출발했어요'
        assert ev['trip.cancelled']['data'] == {'reason': 'user_request'}
        assert ev['sos.ack']['data']['location'] == 'cache' and ev['sos.request']['sos_id'] == wire
    finally:
        ex.shutdown()
        tel.agent.stop(flush_timeout_s=1.0)
        tel.destroy_node()
        probe.destroy_node()
        rclpy.try_shutdown(context=ctx)
        th.join(5)


def test_agent_logs_reach_rosout(monkeypatch):
    """에이전트 로그가 ROS 로거로 간다: RTDB가 diag를 400으로 거절하면 /rosout에 ERROR가 뜬다
    (2026-09-27 전에는 노드 로그에 에이전트 줄이 하나도 없었고, 오류는 diag.recent_errors에만 묻혔다)."""
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    import rclpy
    from conftest import MemoryRtdb
    from rcl_interfaces.msg import Log
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    from mr_care_telemetry.node import TelemetryNode
    from mr_care_telemetry.outbox import Outbox

    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=TEST_DOMAIN_ID)
    probe = Node('probe_log', context=ctx)
    logs: list = []
    # /rosout 발행자는 TransientLocal(depth 1000)이다. 매칭 전에 나간 로그(기동 직후 1회뿐인 ERROR)도 받는다
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    probe.create_subscription(Log, '/rosout', logs.append, QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1000, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    ex = MultiThreadedExecutor(context=ctx)
    ex.add_node(probe)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    db = MemoryRtdb()
    db.fail_status['robots/mr-001/diag'] = 400
    tel = None
    try:
        time.sleep(0.5)                                  # /rosout 구독이 먼저 매칭되게
        tel = TelemetryNode(client=db, outbox=Outbox(), context=ctx,
                            parameter_overrides=[Parameter('publish_diag', value=True)])
        tel.agent.cfg.loop_s = 0.05
        ex.add_node(tel)
        end = time.monotonic() + 8.0
        while time.monotonic() < end and not any('rejected diag' in m.msg for m in logs):
            time.sleep(0.05)
        mine = [m for m in logs if m.name == 'mr_care_telemetry']
        err = [m for m in mine if 'rejected diag' in m.msg]
        assert err and err[0].level == Log.ERROR and '400' in err[0].msg, [(m.name, m.msg) for m in logs]
        assert any(m.level == Log.INFO and 'agent.online' in m.msg for m in mine)
    finally:
        ex.shutdown()
        if tel is not None:
            tel.agent.stop(flush_timeout_s=0.5)
            tel.destroy_node()
        probe.destroy_node()
        rclpy.try_shutdown(context=ctx)
        th.join(5)
