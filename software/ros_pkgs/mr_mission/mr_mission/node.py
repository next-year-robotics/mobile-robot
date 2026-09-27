"""mr_mission 노드. /phone/trip_command → follow 노드 enable 서비스, /robot/app_status 발행(2 s 주기 + 변경 즉시).

    ros2 launch mr_mission mission.launch.py

**이 노드는 follow를 켜서 로봇을 움직인다.** 실기 시험은 사용자가 지켜볼 때만 한다(AGENTS.md 5장).
상태 전이 규칙은 machine.py. 여기는 배선·서비스 호출·발행만 한다.
"""
from __future__ import annotations

import signal
import time
from collections import deque

import rclpy
from mr_care_msgs.msg import RobotAppStatus, TripCommand
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

try:  # ArUco 브랜치가 설치돼 있을 때만
    from mr_aruco_msgs.msg import MarkerTrack
except ImportError:  # pragma: no cover
    MarkerTrack = None

from .machine import Mission, MissionConfig

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
    'error': RobotAppStatus.TRACKING_ERROR,
}
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

        self.pub = self.create_publisher(RobotAppStatus, '/robot/app_status', LATCHED)
        self.create_subscription(TripCommand, '/phone/trip_command', self.on_trip, RELIABLE)
        self.create_subscription(Odometry, p['odom_topic'], self.on_odom, 10)
        if MarkerTrack is not None and p['marker_track_topic']:
            self.create_subscription(MarkerTrack, p['marker_track_topic'], self.on_marker, 10)
        else:
            self.get_logger().info('mr_aruco_msgs not available: tracking_state stays "unavailable"')
        self.client = self.create_client(SetBool, p['follow_enable_service'])
        self.calls: deque[dict] = deque()
        self._last_status: tuple | None = None
        self._last_pub_at = 0.0

        self.create_timer(0.05, self.on_tick)
        # 기동 시 follow가 꺼져 있게 한 번 끈다(상태에는 영향 없음)
        self._execute([('enable', False)])
        self.get_logger().info(f'mission ready: robot_id={self.robot_id} follow={p["follow_enable_service"]}')

    def _declare_params(self) -> dict:
        defaults = {
            'robot_id': 'mr-001', 'status_period_s': 2.0, 'follow_enable_service': '/follow/enable',
            'enable_timeout_s': 3.0, 'stop_timeout_s': 5.0, 'stop_fallback_s': 1.0, 'stop_linear_eps': 0.02,
            'stop_angular_eps': 0.05, 'stop_hold_s': 0.3, 'terminal_hold_s': 3.0, 'marker_stale_s': 1.0,
            'odom_topic': '/odom', 'marker_track_topic': '/marker_track',
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
        self._publish(now)

    def on_odom(self, msg: Odometry):
        self.mission.odom(time.monotonic(), msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_marker(self, msg):
        self.mission.marker(time.monotonic(), int(msg.status))

    def on_tick(self):
        now = time.monotonic()
        self._pump(now)
        self._execute(self.mission.tick(now))
        self._publish(now)

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
        key = tuple(sorted(st.items()))
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
        m.battery_percent = -1           # 배터리 인터페이스 없음
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
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
