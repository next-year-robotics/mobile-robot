"""mr_phone_bridge 노드 통합 시험: 실제 rclpy 노드 + 실제 WebSocket 클라이언트.

안전: 격리된 DDS 도메인에서 localhost 발견만 쓴다. 같은 LAN의 로봇(ROS_DOMAIN_ID=0)에 명령이 새지 않는다.
rclpy·mr_care_msgs·websockets가 모두 있을 때만 돈다(colcon test는 PYTHONPATH로 websockets를 넣어야 한다).
"""
import asyncio
import importlib.util
import math
import os
import threading
import time

import pytest

NEEDS = ('rclpy', 'mr_care_msgs', 'websockets')
MISSING = [m for m in NEEDS if importlib.util.find_spec(m) is None]
pytestmark = pytest.mark.skipif(bool(MISSING), reason=f'missing {MISSING}')

TEST_DOMAIN_ID = 100 + os.getpid() % 100


@pytest.fixture
def ros_env(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter

    from mr_phone_bridge.mock.scenarios import default6_places, write_seed
    from mr_phone_bridge.ros.node import PhoneBridgeNode

    write_seed(tmp_path / 'places.yaml', default6_places(), '2026-08-27T12:00:00Z')
    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=TEST_DOMAIN_ID)
    bridge = PhoneBridgeNode(context=ctx, parameter_overrides=[
        Parameter('bind_host', value='127.0.0.1'), Parameter('port', value=0),
        Parameter('places_path', value=str(tmp_path / 'places.yaml')),
        Parameter('status_period_s', value=0.5), Parameter('sos_retry_interval_s', value=0.5),
    ])
    probe = Probe(ctx)
    ex = MultiThreadedExecutor(context=ctx)
    ex.add_node(bridge)
    ex.add_node(probe.node)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    try:
        yield bridge, probe
    finally:
        bridge.shutdown()
        ex.shutdown()
        bridge.destroy_node()
        probe.node.destroy_node()
        rclpy.try_shutdown(context=ctx)
        t.join(5)


class Probe:
    """mission·safety 계층 흉내. 브리지가 내보내는 토픽을 모두 기록한다."""

    def __init__(self, ctx):
        from mr_care_msgs.msg import (PhoneLinkStatus, PhoneLocation, PlaceCatalog, RobotAppStatus, SosAck,
                                      SosClear, SosRequest, TripCommand)
        from rclpy.node import Node

        from mr_phone_bridge.ros.node import LATCHED, RELIABLE
        self.node = Node('probe', context=ctx)
        self.got = {k: [] for k in ('trip', 'location', 'link', 'catalog', 'sos_ack')}
        for key, typ, topic, qos in (('trip', TripCommand, '/phone/trip_command', RELIABLE),
                                     ('location', PhoneLocation, '/phone/location', RELIABLE),
                                     ('link', PhoneLinkStatus, '/phone/link_status', LATCHED),
                                     ('catalog', PlaceCatalog, '/robot/place_catalog', LATCHED),
                                     ('sos_ack', SosAck, '/safety/sos_ack', RELIABLE)):
            self.node.create_subscription(typ, topic, lambda m, k=key: self.got[k].append(m), qos)
        self.pub_status = self.node.create_publisher(RobotAppStatus, '/robot/app_status', LATCHED)
        self.pub_sos = self.node.create_publisher(SosRequest, '/safety/sos_request', RELIABLE)
        self.pub_clear = self.node.create_publisher(SosClear, '/safety/sos_clear', RELIABLE)
        self.RobotAppStatus, self.SosRequest, self.SosClear = RobotAppStatus, SosRequest, SosClear

    async def wait(self, key, pred=lambda m: True, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for m in self.got[key]:
                if pred(m):
                    return m
            await asyncio.sleep(0.02)
        raise AssertionError(f'no matching {key}; got {self.got[key]}')

    def status(self, mission, tracking=0, trip_id=''):
        m = self.RobotAppStatus()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.robot_id = 'mr-001'
        m.mission_state = mission
        m.tracking_state = tracking
        m.trip_id = trip_id
        m.battery_percent = -1
        self.pub_status.publish(m)


def test_node_end_to_end(ros_env):
    from conftest import run
    from wsclient import Client, location_payload, place_payload

    from mr_phone_bridge.ros.backend import sos_wire_id
    bridge, probe = ros_env
    url = f'ws://127.0.0.1:{bridge.server.port}/bridge/v1'

    async def body():
        cat = await probe.wait('catalog')
        assert cat.revision == '2026-08-27T12:00:00Z' and len(cat.places) == 6
        assert (await probe.wait('link')).connected is False

        c = await Client.connect(url)
        await c.hello()
        # mission 노드·ArUco 없음 → unknown / unavailable / degraded
        st = await c.expect('robot.status', where=lambda m: m['payload']['mission_state'] == 'unknown')
        assert st['payload']['tracking_state'] == 'unavailable' and st['payload']['bridge_state'] == 'degraded'
        link = await probe.wait('link', lambda m: m.connected)
        assert link.session_id == c.session_id and link.last_seen.sec > 0

        ts = await c.send('trip.start', {'from_place_id': 'place_home', 'to_place_id': 'place_market',
                                         'catalog_revision': '2026-08-27T12:00:00Z'})
        assert (await c.reply(ts))['payload']['code'] == 'NOT_READY'

        probe.status(0)                                     # mission idle
        st = await c.expect('robot.status', where=lambda m: m['payload']['mission_state'] == 'idle')
        assert st['payload']['bridge_state'] == 'ready'
        ts = await c.send('trip.start', {'from_place_id': 'place_home', 'to_place_id': 'place_market',
                                         'catalog_revision': '2026-08-27T12:00:00Z'})
        trip_id = (await c.reply(ts))['payload']['trip_id']
        cmd = await probe.wait('trip', lambda m: m.command == 1)
        assert (cmd.trip_id, cmd.request_id, cmd.from_place_id, cmd.to_place_id) == \
            (trip_id, ts, 'place_home', 'place_market')
        probe.status(2, 2, trip_id)                          # active / following
        st = await c.expect('robot.status', where=lambda m: m['payload']['mission_state'] == 'active')
        assert st['payload']['trip_id'] == trip_id and st['payload']['tracking_state'] == 'following'
        tc = await c.send('trip.cancel', {'trip_id': trip_id, 'reason': 'user_request'})
        assert (await c.reply(tc))['type'] == 'ack'
        cmd = await probe.wait('trip', lambda m: m.command == 2)
        assert cmd.trip_id == trip_id and cmd.reason == 'user_request'

        # SOS: CLI처럼 UUID가 아닌 id로 발행해도 앱에는 UUID로, SosAck는 원래 id로
        req = probe.SosRequest()
        req.sos_id, req.reason, req.severity = 'cli-test', 1, 1
        probe.pub_sos.publish(req)
        sreq = await c.expect('sos.request', timeout=3)
        assert sreq['payload']['sos_id'] == sos_wire_id('cli-test')
        assert sreq['payload']['reason'] == 'tracking_lost' and sreq['payload']['trip_id'] is None
        await c.send('sos.ack', {'sos_id': sreq['payload']['sos_id'], 'received': True}, reply_to=sreq['id'])
        ack = await probe.wait('sos_ack')
        assert ack.sos_id == 'cli-test' and ack.app_received and ack.best_source == 0
        lid = await c.send('location.update', location_payload('sos', sreq['payload']['sos_id'], 'none'))
        await c.reply(lid)
        loc = await probe.wait('location')
        assert math.isnan(loc.lat) and loc.source == 0 and loc.failure_reason == 3 and loc.reason == 2
        lid = await c.send('location.update', location_payload('sos', sreq['payload']['sos_id'], 'cache'))
        await c.reply(lid)
        ack = await probe.wait('sos_ack', lambda m: m.best_source == 1)
        assert ack.has_location
        clr = probe.SosClear()
        clr.sos_id, clr.resolution = 'cli-test', 2
        probe.pub_clear.publish(clr)
        done = await c.expect('sos.clear', timeout=3)
        assert done['payload'] == {'sos_id': sos_wire_id('cli-test'), 'resolution': 'handled'}

        up = await c.send('place.upsert', place_payload('2026-08-27T12:00:00Z'))
        rev = (await c.reply(up))['payload']['catalog_revision']
        cat = await probe.wait('catalog', lambda m: m.revision == rev)
        assert len(cat.places) == 7 and any(p.id == 'place_hospital' for p in cat.places)

        await c.close()
        await probe.wait('link', lambda m: not m.connected and m.session_id == c.session_id)

    run(body(), timeout=60)
