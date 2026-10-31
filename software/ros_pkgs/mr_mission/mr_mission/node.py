"""mr_mission 노드. /phone/trip_command → follow 노드 enable 서비스, /robot/app_status 발행(2 s 주기 + 변경 즉시).
SOS 판단(M3): /marker_track LOST 지속 → /safety/sos_request, 회복 → /safety/sos_clear,
/safety/sos_ack(app_received=false) → 안전 정지(follow disable, trip error).
앱 연결 끊김(D6): 주행 중 /phone/link_status가 연속 끊김(기본 30 s) → 안전 정지(follow disable, trip error).
앱 SOS 버튼(D7): /phone/sos_raise → manual SOS, /phone/sos_cancel → sos_clear(cancelled).
배터리: /base/status supply_voltage → battery_percent(battery.py).

    ros2 launch mr_mission mission.launch.py

**이 노드는 follow를 켜서 로봇을 움직인다.** 실기 시험은 사용자가 지켜볼 때만 한다(AGENTS.md 5장).
상태 전이 규칙은 machine.py(trip)·safety.py(SOS·링크). 여기는 배선·서비스 호출·발행만 한다.
"""
from __future__ import annotations

import signal
import time
from collections import deque

import rclpy
from mr_care_msgs.msg import PhoneLinkStatus, RobotAppStatus, SosAck, SosClear, SosRequest, TripCommand
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_srvs.srv import SetBool

try:  # ArUco 브랜치가 설치돼 있을 때만
    from mr_aruco_msgs.msg import MarkerTrack
except ImportError:  # pragma: no cover
    MarkerTrack = None
try:  # 베이스 메시지(mr_base). 없으면 battery_percent는 -1
    from mr_msgs.msg import BaseStatus
except ImportError:  # pragma: no cover
    BaseStatus = None

from .battery import BatteryEstimator
from .machine import Mission, MissionConfig
from .safety import LinkConfig, LinkWatch, SafetyConfig, SafetyMonitor

RELIABLE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)
LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

MISSION_CODES = {
    'idle': RobotAppStatus.MISSION_IDLE, 'starting': RobotAppStatus.MISSION_STARTING,
    'active': RobotAppStatus.MISSION_ACTIVE, 'arrived': RobotAppStatus.MISSION_ARRIVED,
    'cancelling': RobotAppStatus.MISSION_CANCELLING, 'cancelled': RobotAppStatus.MISSION_CANCELLED,
    'error': RobotAppStatus.MISSION_ERROR,
}
TRACKING_CODES = {
    'unavailable': RobotAppStatus.TRACKING_UNAVAILABLE, 'searching': RobotAppStatus.TRACKING_SEARCHING,
    'following': RobotAppStatus.TRACKING_FOLLOWING, 'lost': RobotAppStatus.TRACKING_LOST,
    'error': RobotAppStatus.TRACKING_ERROR, 'idle': RobotAppStatus.TRACKING_IDLE,
}
RESOLUTION_CODES = {
    'tracking_recovered': SosClear.RESOLUTION_TRACKING_RECOVERED, 'handled': SosClear.RESOLUTION_HANDLED,
    'cancelled': SosClear.RESOLUTION_CANCELLED, 'unknown': SosClear.RESOLUTION_UNKNOWN,
}
SEVERITY_CODES = {'critical': SosRequest.SEVERITY_CRITICAL, 'warning': SosRequest.SEVERITY_WARNING}
REASON_CODES = {'tracking_lost': SosRequest.REASON_TRACKING_LOST, 'manual': SosRequest.REASON_MANUAL}
DISABLE_CALL_TIMEOUT_S = 1.0


class MissionNode(Node):

    def __init__(self, **kwargs):
        super().__init__('mr_mission', **kwargs)
        p = self._declare_params()
        self.robot_id = p['robot_id']
        self.status_period_s = p['status_period_s']
        self.enable_timeout_s = p['enable_timeout_s']
        self.mission = Mission(MissionConfig(
            enable_timeout_s=p['enable_timeout_s'], stop_timeout_s=p['stop_timeout_s'],
            stop_fallback_s=p['stop_fallback_s'], stop_linear_eps=p['stop_linear_eps'],
            stop_angular_eps=p['stop_angular_eps'], stop_hold_s=p['stop_hold_s'],
            terminal_hold_s=p['terminal_hold_s'], marker_stale_s=p['marker_stale_s']))

        self.safety = None
        if p['sos_enabled']:
            self.safety = SafetyMonitor(SafetyConfig(
                lost_to_sos_s=p['sos_lost_s'], recover_to_clear_s=p['sos_recover_s'],
                marker_stale_s=p['marker_stale_s'], stop_on_no_ack=p['sos_stop_on_no_ack']))
            self.sos_req_pub = self.create_publisher(SosRequest, '/safety/sos_request', RELIABLE)
            self.sos_clear_pub = self.create_publisher(SosClear, '/safety/sos_clear', RELIABLE)
            self.create_subscription(SosAck, '/safety/sos_ack', self.on_sos_ack, RELIABLE)
            self.create_subscription(SosRequest, '/phone/sos_raise', self.on_sos_raise, RELIABLE)
            self.create_subscription(SosClear, '/phone/sos_cancel', self.on_sos_cancel, RELIABLE)

        self.link = None
        if p['link_lost_stop_s'] > 0:
            self.link = LinkWatch(LinkConfig(stop_after_s=p['link_lost_stop_s'], stale_s=p['link_stale_s']),
                                  now=time.monotonic())
            self.create_subscription(PhoneLinkStatus, '/phone/link_status', self.on_link, LATCHED)

        self.pub = self.create_publisher(RobotAppStatus, '/robot/app_status', LATCHED)
        self.create_subscription(TripCommand, '/phone/trip_command', self.on_trip, RELIABLE)
        self.create_subscription(Odometry, p['odom_topic'], self.on_odom, 10)
        if MarkerTrack is not None and p['marker_track_topic']:
            self.create_subscription(MarkerTrack, p['marker_track_topic'], self.on_marker, 10)
        else:
            self.get_logger().info('mr_aruco_msgs not available: tracking_state stays "unavailable"')
        self.battery = BatteryEstimator(diode_drop_v=p['battery_diode_drop_v'], tau_s=p['battery_filter_tau_s'])
        if BaseStatus is not None and p['base_status_topic']:
            # micro-ROS가 BEST_EFFORT로 낸다. RELIABLE로 받으면 QoS 불일치로 하나도 안 온다(2026-11-01 실기)
            self.create_subscription(BaseStatus, p['base_status_topic'], self.on_base_status, qos_profile_sensor_data)
        self.client = self.create_client(SetBool, p['follow_enable_service'])
        self.calls: deque[dict] = deque()
        self._last_status: tuple | None = None
        self._last_pub_at = 0.0

        self.create_timer(0.05, self.on_tick)
        # 기동 시 follow가 꺼져 있게 한 번 끈다(상태에는 영향 없음)
        self._execute([('enable', False)])
        sos = (f'SOS after {p["sos_lost_s"]:g} s lost, clear after {p["sos_recover_s"]:g} s, '
               f'stop on no ack={p["sos_stop_on_no_ack"]}') if self.safety else 'SOS off'
        link = f'stop after {p["link_lost_stop_s"]:g} s of phone link loss' if self.link else 'phone link watch off'
        self.get_logger().info(f'mission ready: robot_id={self.robot_id} follow={p["follow_enable_service"]}; {sos}; {link}')

    def _declare_params(self) -> dict:
        defaults = {
            'robot_id': 'mr-001', 'status_period_s': 2.0, 'follow_enable_service': '/follow/enable',
            'enable_timeout_s': 3.0, 'stop_timeout_s': 5.0, 'stop_fallback_s': 1.0, 'stop_linear_eps': 0.02,
            'stop_angular_eps': 0.05, 'stop_hold_s': 0.3, 'terminal_hold_s': 3.0, 'marker_stale_s': 1.0,
            'odom_topic': '/odom', 'marker_track_topic': '/marker_track',
            'sos_enabled': True, 'sos_lost_s': 10.0, 'sos_recover_s': 5.0, 'sos_stop_on_no_ack': True,
            'link_lost_stop_s': 30.0, 'link_stale_s': 12.0,
            'base_status_topic': '/base/status', 'battery_diode_drop_v': 0.30, 'battery_filter_tau_s': 20.0,
        }
        out = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            out[name] = self.get_parameter(name).value
        return out

    # ------------------------------------------------------------ 입력

    def on_trip(self, msg: TripCommand):
        now = time.monotonic()
        if msg.command == TripCommand.COMMAND_START:
            self.get_logger().info(f'trip start {msg.trip_id}: {msg.from_place_id} -> {msg.to_place_id}')
            actions = self.mission.start(now, msg.trip_id, msg.from_place_id, msg.to_place_id)
        elif msg.command == TripCommand.COMMAND_CANCEL:
            self.get_logger().info(f'trip cancel {msg.trip_id} ({msg.reason})')
            actions = self.mission.cancel(now, msg.trip_id)
        else:
            self.get_logger().warning(f'unknown TripCommand.command {msg.command}')
            return
        self._execute(actions)
        if self.safety is not None:          # SOS 중 앱 조작 → 해제(D3b)
            self._safety(now, self.safety.trip_command(
                now, 'start' if msg.command == TripCommand.COMMAND_START else 'cancel'))
        self._publish(now)

    def on_odom(self, msg: Odometry):
        self.mission.odom(time.monotonic(), msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_marker(self, msg):
        now = time.monotonic()
        self.mission.marker(now, int(msg.status))
        if self.safety is not None:
            self._safety(now, self.safety.marker(now, int(msg.status)))

    def on_sos_ack(self, msg: SosAck):
        now = time.monotonic()
        self._safety(now, self.safety.sos_ack(now, msg.sos_id, msg.app_received))

    def on_sos_raise(self, msg: SosRequest):
        now = time.monotonic()
        self._safety(now, self.safety.manual(now, msg.sos_id))

    def on_sos_cancel(self, msg: SosClear):
        now = time.monotonic()
        self._safety(now, self.safety.cancel(now, msg.sos_id))

    def on_base_status(self, msg):
        self.battery.update(time.monotonic(), float(msg.supply_voltage))

    def on_link(self, msg: PhoneLinkStatus):
        now = time.monotonic()
        self._link(now, self.link.link(now, msg.connected))

    def on_tick(self):
        now = time.monotonic()
        self._pump(now)
        self._execute(self.mission.tick(now))
        if self.safety is not None:
            self._safety(now, self.safety.mission(now, self.mission.state, self.mission.trip_id))
        if self.link is not None:
            self._link(now, self.link.mission(now, self.mission.state, self.mission.trip_id))
        self._publish(now)

    # ------------------------------------------------------------ 앱 연결 끊김(D6)

    def _link(self, now: float, actions):
        for a in actions:
            if a[0] == 'stop':
                self._stop(now, a[1])
        for line in self.link.log:           # 끊김·정지는 warning, 회복은 info (호출 위치를 나눈다: rclpy 심각도 규칙)
            if 'restored' in line:
                self.get_logger().info(line)
            else:
                self.get_logger().warning(line)
        self.link.log.clear()

    def _stop(self, now: float, message: str):
        self.get_logger().warning(f'safety stop: {message}')
        self._execute(self.mission.abort(now, message))

    # ------------------------------------------------------------ SOS

    def _safety(self, now: float, actions):
        for a in actions:
            if a[0] == 'sos_request':
                _, sos_id, reason, severity, trip_id = a
                m = SosRequest()
                m.header.stamp = self.get_clock().now().to_msg()
                m.sos_id, m.reason = sos_id, REASON_CODES.get(reason, SosRequest.REASON_UNKNOWN)
                m.severity = SEVERITY_CODES.get(severity, SosRequest.SEVERITY_CRITICAL)
                m.trip_id = trip_id or ''
                self.sos_req_pub.publish(m)
            elif a[0] == 'sos_clear':
                self._publish_clear(a[1], a[2])
            elif a[0] == 'stop':
                self._stop(now, a[1])
        for line in self.safety.log:         # SOS 발생·무응답은 warning, 해제는 info
            # rclpy는 한 호출 위치에서 심각도가 바뀌면 ValueError를 낸다: 호출 위치를 나눈다
            if 'cleared' in line:
                self.get_logger().info(line)
            else:
                self.get_logger().warning(line)
        self.safety.log.clear()

    def _publish_clear(self, sos_id: str, resolution: str):
        m = SosClear()
        m.header.stamp = self.get_clock().now().to_msg()
        m.sos_id = sos_id
        m.resolution = RESOLUTION_CODES.get(resolution, SosClear.RESOLUTION_UNKNOWN)
        self.sos_clear_pub.publish(m)

    def clear_sos_on_exit(self):
        """종료 시 활성 SOS를 닫는다. 이 노드가 없으면 아무도 clear를 내지 못해 앱·대시보드에 SOS가 영영 남는다."""
        if self.safety is not None and self.safety.sos_id is not None:
            self.get_logger().warning(f'exiting with active SOS {self.safety.sos_id}: sending sos_clear(unknown)')
            self._publish_clear(self.safety.sos_id, 'unknown')

    # ------------------------------------------------------------ follow enable 서비스

    def _execute(self, actions):
        now = time.monotonic()
        for kind, want in actions:
            if kind == 'enable':
                timeout = self.enable_timeout_s if want else DISABLE_CALL_TIMEOUT_S
                self.calls.append({'want': want, 'deadline': now + timeout, 'future': None})
        self._pump(now)

    def _pump(self, now: float):
        """요청을 하나씩 순서대로 보낸다. 결과는 machine에 돌려준다."""
        while self.calls:
            c = self.calls[0]
            if c['future'] is None:
                if self.client.service_is_ready():
                    req = SetBool.Request()
                    req.data = c['want']
                    c['future'] = self.client.call_async(req)
                elif now > c['deadline']:
                    self.calls.popleft()
                    self._result(now, c['want'], False, 'follow service not available')
                    continue
                else:
                    return
            if c['future'].done():
                self.calls.popleft()
                try:
                    resp = c['future'].result()
                    ok, detail = bool(resp.success), resp.message
                except Exception as e:
                    ok, detail = False, repr(e)
                self._result(now, c['want'], ok, detail)
                continue
            if now > c['deadline']:
                self.calls.popleft()
                self._result(now, c['want'], False, 'follow service timeout')
                continue
            return

    def _result(self, now: float, want: bool, ok: bool, detail: str):
        self._execute(self.mission.enable_result(now, want, ok, detail))
        self._publish(now)

    # ------------------------------------------------------------ 발행

    def _publish(self, now: float):
        for line in self.mission.log:
            self.get_logger().info(line)
        self.mission.log.clear()
        st = self.mission.status(now)
        battery = self.battery.percent(now)
        key = tuple(sorted(st.items())) + (battery,)
        if key == self._last_status and now - self._last_pub_at < self.status_period_s:
            return
        self._last_status, self._last_pub_at = key, now
        m = RobotAppStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.robot_id = self.robot_id
        m.mission_state = MISSION_CODES.get(st['mission_state'], RobotAppStatus.MISSION_UNKNOWN)
        m.tracking_state = TRACKING_CODES.get(st['tracking_state'], RobotAppStatus.TRACKING_UNKNOWN)
        m.trip_id = st['trip_id'] or ''
        m.from_place_id = st['from_place_id'] or ''
        m.to_place_id = st['to_place_id'] or ''
        m.battery_percent = battery
        m.last_error = st['last_error'] or ''
        self.pub.publish(m)

    def disable_on_exit(self):
        """종료 시 주행 중이면 follow를 끈다(최선 노력). mission이 죽은 채로 추종이 계속되지 않게."""
        if self.mission.state not in ('starting', 'active', 'cancelling'):
            return
        if not self.client.service_is_ready():
            self.get_logger().error('exiting during a trip and follow service is unavailable')
            return
        req = SetBool.Request()
        req.data = False
        fut = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
        self.get_logger().warning(f'exiting during a trip: follow disable {"sent" if fut.done() else "timed out"}')


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # rclpy 기본 SIGINT 처리는 context를 먼저 닫아 종료 정리(발행·서비스 호출)를 막는다.
    # 파이썬 기본 처리(KeyboardInterrupt)를 쓰고, 짧은 타이머가 spin을 깨워 인터럽트가 전달되게 한다.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    # 백그라운드 셸에서 띄우면 SIGINT가 SIG_IGN으로 상속돼 파이썬이 처리기를 달지 않는다. 명시적으로 단다.
    # SIGTERM(launch 에스컬레이션·systemd)도 같은 정리 경로를 탄다.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = None
    try:
        node = MissionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            if rclpy.ok():
                node.disable_on_exit()
                node.clear_sos_on_exit()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
