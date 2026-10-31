"""mr_mission 노드 통합 시험: 가짜 follow 서비스·가짜 /odom과 함께 격리 도메인에서 돈다.

마지막 시험은 mr_phone_bridge와 함께 앱 WebSocket → 브리지 → mission → 브리지 → 앱 전체 경로를 본다
(mr_phone_bridge·websockets가 있을 때만).
안전: domain_id를 격리하고 localhost 발견만 쓴다. 실제 follow 노드·로봇에 닿지 않는다.
"""
import importlib.util
import os
import sys
import threading
import time
from pathlib import Path

import pytest

MISSING = [m for m in ('rclpy', 'mr_care_msgs', 'std_srvs', 'nav_msgs') if importlib.util.find_spec(m) is None]
pytestmark = pytest.mark.skipif(bool(MISSING), reason=f'missing {MISSING}')
TEST_DOMAIN_ID = 100 + (os.getpid() + 37) % 100


class FakeWorld:
    """follow 노드(/follow/enable)와 베이스(/odom) 흉내. enable이면 0.2 m/s로 움직이고 disable이면 선다."""

    def __init__(self, ctx, with_service=True):
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from std_srvs.srv import SetBool
        self.node = Node('fake_world', context=ctx)
        self.enabled = False
        self.calls = []
        if with_service:
            self.node.create_service(SetBool, '/follow/enable', self.on_enable)
        self.odom_pub = self.node.create_publisher(Odometry, '/odom', 10)
        self.node.create_timer(0.02, self.on_odom)
        self.Odometry = Odometry

    def on_enable(self, req, resp):
        self.calls.append(req.data)
        self.enabled = req.data
        resp.success = True
        resp.message = 'ok'
        return resp

    def on_odom(self):
        m = self.Odometry()
        m.twist.twist.linear.x = 0.2 if self.enabled else 0.0
        self.odom_pub.publish(m)


class Env:
    def __init__(self, ctx, **overrides):
        from mr_care_msgs.msg import RobotAppStatus, TripCommand
        from rclpy.node import Node
        from rclpy.parameter import Parameter

        from mr_mission.node import LATCHED, RELIABLE, MissionNode
        self.mission = MissionNode(context=ctx, parameter_overrides=[
            Parameter(k, value=v) for k, v in {'status_period_s': 0.5, **overrides}.items()])
        self.probe = Node('probe', context=ctx)
        self.statuses = []
        self.probe.create_subscription(RobotAppStatus, '/robot/app_status', self.statuses.append, LATCHED)
        self.cmd_pub = self.probe.create_publisher(TripCommand, '/phone/trip_command', RELIABLE)
        self.TripCommand, self.RobotAppStatus = TripCommand, RobotAppStatus

    def send(self, command, trip_id, from_id='place_home', to_id='place_market'):
        m = self.TripCommand()
        m.request_id, m.trip_id, m.command = 'req', trip_id, command
        m.from_place_id, m.to_place_id, m.reason = from_id, to_id, 'user_request'
        self.cmd_pub.publish(m)

    def wait_state(self, state, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.statuses and self.statuses[-1].mission_state == state:
                return self.statuses[-1]
            time.sleep(0.02)
        raise AssertionError(f'no mission_state {state}; last={[s.mission_state for s in self.statuses[-5:]]}')


@pytest.fixture
def ros(monkeypatch):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=TEST_DOMAIN_ID)
    ex = MultiThreadedExecutor(context=ctx)
    nodes = []

    def add(*ns):
        for n in ns:
            nodes.append(n)
            ex.add_node(n)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    try:
        yield ctx, add
    finally:
        ex.shutdown()
        for n in nodes:
            n.destroy_node()
        rclpy.try_shutdown(context=ctx)
        t.join(5)


def test_trip_start_cancel_with_follow(ros):
    ctx, add = ros
    world = FakeWorld(ctx)
    env = Env(ctx)
    add(world.node, env.mission, env.probe)
    st = env.wait_state(env.RobotAppStatus.MISSION_IDLE)
    assert st.robot_id == 'mr-001' and st.battery_percent == -1
    assert world.calls[:1] == [False]                         # 기동 시 follow를 끈다
    env.send(env.TripCommand.COMMAND_START, 'trip-1')
    st = env.wait_state(env.RobotAppStatus.MISSION_ACTIVE)
    assert (st.trip_id, st.from_place_id, st.to_place_id) == ('trip-1', 'place_home', 'place_market')
    assert world.enabled
    env.send(env.TripCommand.COMMAND_CANCEL, 'trip-1')
    env.wait_state(env.RobotAppStatus.MISSION_CANCELLING)
    st = env.wait_state(env.RobotAppStatus.MISSION_CANCELLED)
    assert not world.enabled and st.trip_id == 'trip-1'
    st = env.wait_state(env.RobotAppStatus.MISSION_IDLE, timeout=6)
    assert st.trip_id == ''
    seen = [s.mission_state for s in env.statuses]
    assert env.RobotAppStatus.MISSION_ARRIVED not in seen


def test_no_follow_service_is_error(ros):
    ctx, add = ros
    world = FakeWorld(ctx, with_service=False)
    env = Env(ctx, enable_timeout_s=0.5)
    add(world.node, env.mission, env.probe)
    env.wait_state(env.RobotAppStatus.MISSION_IDLE)
    env.send(env.TripCommand.COMMAND_START, 'trip-x')
    st = env.wait_state(env.RobotAppStatus.MISSION_ERROR, timeout=5)
    assert st.last_error == '추종을 시작하지 못했어요'


def test_sos_lost_recover_and_no_ack_safety_stop(monkeypatch):
    """M3(2026-09-27 결정): 주행 중 LOST 지속 → SosRequest, 회복 지속 → SosClear, 해제 즉시 재무장,
    SosAck(app_received=false) → follow disable + trip error, 마커가 다시 보여도 자동 재개 없음.
    mr_aruco_msgs가 없어도 돌게 마커 입력은 probe 타이머가 노드 콜백을 직접 부른다. 경합이 없게 단일 스레드 executor."""
    from types import SimpleNamespace

    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    import rclpy
    from mr_care_msgs.msg import SosAck, SosClear, SosRequest
    from rclpy.executors import SingleThreadedExecutor

    from mr_mission.node import RELIABLE

    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=TEST_DOMAIN_ID)
    ex = SingleThreadedExecutor(context=ctx)
    world = FakeWorld(ctx)
    env = Env(ctx, sos_lost_s=1.0, sos_recover_s=0.5)
    reqs, clears = [], []
    env.probe.create_subscription(SosRequest, '/safety/sos_request', reqs.append, RELIABLE)
    env.probe.create_subscription(SosClear, '/safety/sos_clear', clears.append, RELIABLE)
    ack_pub = env.probe.create_publisher(SosAck, '/safety/sos_ack', RELIABLE)
    marker = {'status': None}

    def feed():
        if marker['status'] is not None:
            env.mission.on_marker(SimpleNamespace(status=marker['status']))
    env.probe.create_timer(0.1, feed)
    for n in (world.node, env.mission, env.probe):
        ex.add_node(n)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    def wait(pred, what, timeout=6.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pred():
                return
            time.sleep(0.02)
        raise AssertionError(f'{what}; reqs={len(reqs)} clears={[(c.sos_id, c.resolution) for c in clears]} '
                             f'states={[x.mission_state for x in env.statuses[-4:]]}')
    try:
        env.wait_state(env.RobotAppStatus.MISSION_IDLE)
        env.send(env.TripCommand.COMMAND_START, 'trip-s')
        env.wait_state(env.RobotAppStatus.MISSION_ACTIVE)
        marker['status'] = 1                                  # 잡았다
        time.sleep(0.5)
        marker['status'] = 0                                  # 놓쳤다
        wait(lambda: reqs, 'no SosRequest after LOST')
        r1 = reqs[0]
        assert (r1.reason, r1.severity, r1.trip_id) == (SosRequest.REASON_TRACKING_LOST,
                                                        SosRequest.SEVERITY_CRITICAL, 'trip-s')
        assert len(r1.sos_id) == 36 and world.enabled          # SOS만으로는 trip을 멈추지 않는다(D5)
        ack_pub.publish(SosAck(sos_id=r1.sos_id, app_received=True, retries=0))
        marker['status'] = 1
        wait(lambda: clears, 'no SosClear after recovery')
        assert (clears[0].sos_id, clears[0].resolution) == (r1.sos_id, SosClear.RESOLUTION_TRACKING_RECOVERED)
        marker['status'] = 0                                  # 다시 놓침 → 재무장돼 있다(D2)
        wait(lambda: len(reqs) == 2, 'no second SosRequest')
        r2 = reqs[1]
        assert r2.sos_id != r1.sos_id and world.enabled
        ack_pub.publish(SosAck(sos_id=r2.sos_id, app_received=False, retries=3))   # 폰 무응답(OPEN-R3)
        st = env.wait_state(env.RobotAppStatus.MISSION_ERROR, timeout=6)
        assert st.last_error == '어르신 폰이 응답하지 않아 멈췄어요' and not world.enabled
        marker['status'] = 1                                  # 다시 보여도
        wait(lambda: len(clears) == 2, 'no SosClear after second recovery')
        time.sleep(0.5)
        assert not world.enabled and env.statuses[-1].mission_state == env.RobotAppStatus.MISSION_ERROR
        assert world.calls.count(True) == 1                   # 자동 재개 없음
    finally:
        ex.shutdown()
        for n in (world.node, env.mission, env.probe):
            n.destroy_node()
        rclpy.try_shutdown(context=ctx)
        th.join(5)


def test_link_loss_stops_trip_without_resume(ros):
    """D6(2026-09-27 결정): 주행 중 앱 링크가 유예(여기서는 1 s) 안에 돌아오면 계속, 넘기면 follow off + trip error,
    다시 붙어도 자동 재개 없음."""
    from mr_care_msgs.msg import PhoneLinkStatus

    from mr_mission.node import LATCHED
    ctx, add = ros
    world = FakeWorld(ctx)
    env = Env(ctx, link_lost_stop_s=1.0)
    link_pub = env.probe.create_publisher(PhoneLinkStatus, '/phone/link_status', LATCHED)
    add(world.node, env.mission, env.probe)

    def link(up):
        link_pub.publish(PhoneLinkStatus(connected=up, app_instance_id='app', session_id='s'))
    link(True)
    env.wait_state(env.RobotAppStatus.MISSION_IDLE)
    env.send(env.TripCommand.COMMAND_START, 'trip-l')
    env.wait_state(env.RobotAppStatus.MISSION_ACTIVE)
    link(False)
    time.sleep(0.5)
    link(True)                                            # 유예 안에 재연결 → 계속
    time.sleep(1.2)
    assert world.enabled and env.statuses[-1].mission_state == env.RobotAppStatus.MISSION_ACTIVE
    link(False)
    st = env.wait_state(env.RobotAppStatus.MISSION_ERROR, timeout=5)
    assert st.last_error == '폰 연결이 끊겨 멈췄어요' and st.trip_id == 'trip-l' and not world.enabled
    link(True)
    time.sleep(1.0)
    assert env.statuses[-1].mission_state == env.RobotAppStatus.MISSION_ERROR
    assert not world.enabled and world.calls.count(True) == 1      # 자동 재개 없음
    env.send(env.TripCommand.COMMAND_CANCEL, 'trip-l')              # 앱 멈춤으로 정리
    env.wait_state(env.RobotAppStatus.MISSION_CANCELLED)


BRIDGE_MISSING = [m for m in ('mr_phone_bridge', 'websockets') if importlib.util.find_spec(m) is None]


@pytest.mark.skipif(bool(BRIDGE_MISSING), reason=f'missing {BRIDGE_MISSING}')
def test_app_to_mission_end_to_end(ros, tmp_path):
    """앱(WebSocket) → 브리지 → mission → follow → 브리지 → 앱."""
    import asyncio

    from rclpy.parameter import Parameter

    from mr_phone_bridge.mock.scenarios import default6_places, write_seed
    from mr_phone_bridge.ros.node import PhoneBridgeNode
    bridge_test = Path(importlib.util.find_spec('mr_phone_bridge').origin).resolve().parents[1] / 'test'
    if not (bridge_test / 'wsclient.py').exists():
        pytest.skip('mr_phone_bridge test helpers not found (symlink install needed)')
    sys.path.insert(0, str(bridge_test))
    from wsclient import Client

    ctx, add = ros
    write_seed(tmp_path / 'places.yaml', default6_places(), '2026-08-27T12:00:00Z')
    world = FakeWorld(ctx)
    env = Env(ctx)
    bridge = PhoneBridgeNode(context=ctx, parameter_overrides=[
        Parameter('bind_host', value='127.0.0.1'), Parameter('port', value=0),
        Parameter('places_path', value=str(tmp_path / 'places.yaml'))])
    add(world.node, env.mission, env.probe, bridge)
    try:
        async def body():
            c = await Client.connect(f'ws://127.0.0.1:{bridge.server.port}/bridge/v1')
            await c.hello()
            await c.expect('robot.status', timeout=5, where=lambda m: m['payload']['mission_state'] == 'idle')
            ts = await c.send('trip.start', {'from_place_id': 'place_home', 'to_place_id': 'place_market',
                                             'catalog_revision': '2026-08-27T12:00:00Z'})
            trip_id = (await c.reply(ts))['payload']['trip_id']
            st = await c.expect('robot.status', timeout=5,
                                where=lambda m: m['payload']['mission_state'] == 'active')
            assert st['payload']['trip_id'] == trip_id and st['payload']['bridge_state'] == 'ready'
            assert world.enabled
            dl = await c.send('place.delete', {'place_id': 'place_market', 'catalog_revision': '2026-08-27T12:00:00Z'})
            assert (await c.reply(dl))['payload']['code'] == 'PLACE_IN_USE'
            tc = await c.send('trip.cancel', {'trip_id': trip_id, 'reason': 'user_request'})
            assert (await c.reply(tc))['type'] == 'ack'
            await c.expect('robot.status', timeout=5, where=lambda m: m['payload']['mission_state'] == 'cancelled')
            assert not world.enabled
            # 앱 SOS 버튼(D7): sos.raise → mission이 연 manual SOS → sos.cancel → sos.clear(cancelled)
            sr = await c.send('sos.raise', {'reason': 'manual'})
            assert (await c.reply(sr))['type'] == 'ack'
            req = await c.expect('sos.request', timeout=5)
            assert req['payload']['reason'] == 'manual'
            sid = req['payload']['sos_id']
            await c.send('sos.ack', {'sos_id': sid, 'received': True}, reply_to=req['id'])
            sc = await c.send('sos.cancel', {'sos_id': sid})
            assert (await c.reply(sc))['type'] == 'ack'
            clear = await c.expect('sos.clear', timeout=5)
            assert clear['payload'] == {'sos_id': sid, 'resolution': 'cancelled'}
            await c.close()
        asyncio.run(asyncio.wait_for(body(), 30))
    finally:
        bridge.shutdown()
