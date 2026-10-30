"""mr_care_telemetry ROS 2 노드. ROS 토픽을 TelemetryAgent 입력으로 옮기기만 한다. RTDB 쓰기는 에이전트 스레드.

    ros2 launch mr_care_telemetry care_telemetry.launch.py

구독(모두 읽기만): /robot/app_status, /phone/link_status, /robot/place_catalog(TransientLocal),
/base/status(BEST_EFFORT), /phone/trip_command, /phone/location, /safety/sos_request·sos_ack·sos_clear,
/marker_track(있으면), /joint_states(publish_diag일 때만). 아무것도 발행하지 않는다.
텔레메트리가 죽거나 느려져도 브리지·베이스 제어에는 영향이 없다(별도 프로세스·systemd 유닛).
에이전트의 파이썬 logging은 ROS 로거(/rosout, launch 출력, journald)로 옮긴다.
"""
from __future__ import annotations

import logging
import os
import signal

import rclpy
from mr_care_msgs.msg import (PhoneLinkStatus, PhoneLocation, PlaceCatalog, RobotAppStatus, SosAck, SosClear,
                              SosRequest, TripCommand)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions

try:
    from mr_msgs.msg import BaseStatus
except ImportError:  # pragma: no cover
    BaseStatus = None
try:
    from mr_aruco_msgs.msg import MarkerTrack
except ImportError:  # pragma: no cover
    MarkerTrack = None

from .agent import AgentConfig, TelemetryAgent
from .outbox import Outbox
from .rtdb import RtdbClient

RELIABLE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)
LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
BASE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)       # base_contract: /base/status BEST_EFFORT, KEEP_LAST(1)


def _names(cls, prefix: str) -> dict[int, str]:
    """메시지 상수에서 정수→문자열 표를 만든다(예: MISSION_IDLE → 'idle'). 숫자를 여기서 다시 적지 않는다."""
    return {getattr(cls, a): a[len(prefix):].lower() for a in dir(cls) if a.startswith(prefix) and a.isupper()}


MISSION = _names(RobotAppStatus, 'MISSION_')
TRACKING = _names(RobotAppStatus, 'TRACKING_')
SOS_REASON = _names(SosRequest, 'REASON_')
SOS_SEVERITY = _names(SosRequest, 'SEVERITY_')
SOS_RESOLUTION = _names(SosClear, 'RESOLUTION_')
LOC_REASON = _names(PhoneLocation, 'REASON_')
LOC_SOURCE = _names(PhoneLocation, 'SOURCE_')
TRIP_COMMAND = _names(TripCommand, 'COMMAND_')
BASE_AGENT = _names(BaseStatus, 'AGENT_') if BaseStatus is not None else {}


def stamp_ms(t) -> int:
    return t.sec * 1000 + t.nanosec // 1_000_000


# 노드에서 로그를 남기는 모듈. 패키지 로거 하나에 달지 않고 모듈 로거마다 단다: launch가 import된 프로세스
# (launch_testing pytest 플러그인)에서는 logging 계층이 깨져 자식 로거의 부모가 root가 된다.
PY_LOGGERS = ('mr_care_telemetry.agent', 'mr_care_telemetry.outbox')


class RosLogHandler(logging.Handler):
    """에이전트의 파이썬 logging을 ROS 로거로 보낸다. 이것이 없으면 INFO("RTDB online", 이벤트)는 사라지고
    ERROR("RTDB rejected …")도 형식 없이 stderr로만 간다(2026-09-27 이전)."""

    def __init__(self, logger):
        super().__init__()
        self._logger = logger

    def emit(self, record):
        try:
            msg = self.format(record)
            if record.levelno >= logging.ERROR:
                self._logger.error(msg)
            elif record.levelno >= logging.WARNING:
                self._logger.warning(msg)
            elif record.levelno >= logging.INFO:
                self._logger.info(msg)
            else:
                self._logger.debug(msg)
        except Exception:
            pass


class TelemetryNode(Node):

    def __init__(self, *, client: RtdbClient | None = None, outbox: Outbox | None = None, **kwargs):
        super().__init__('mr_care_telemetry', **kwargs)
        self._log_handler = RosLogHandler(self.get_logger())
        for name in PY_LOGGERS:
            lg = logging.getLogger(name)
            lg.addHandler(self._log_handler)
            lg.setLevel(logging.INFO)
            lg.propagate = False
        p = self._declare_params()
        cfg = AgentConfig(robot_id=p['robot_id'], display_name=p['display_name'],
                          status_period_s=p['status_period_s'], app_status_stale_s=p['app_status_stale_s'],
                          publish_sos_location=p['publish_sos_location'], publish_diag=p['publish_diag'],
                          max_events=p['max_events'])
        if client is None:
            client = RtdbClient.from_settings(p['database_url'], p['service_account_path'],
                                              emulator_host=p['emulator_host'] or None)
        self.agent = TelemetryAgent(cfg, client, outbox or Outbox(p['queue_db_path']))
        a = self.agent

        self.create_subscription(RobotAppStatus, '/robot/app_status', self.on_app_status, LATCHED)
        self.create_subscription(PhoneLinkStatus, '/phone/link_status', self.on_link, LATCHED)
        self.create_subscription(PlaceCatalog, '/robot/place_catalog',
                                 lambda m: a.on_place_catalog({pl.id: pl.label for pl in m.places}), LATCHED)
        self.create_subscription(TripCommand, '/phone/trip_command', lambda m: a.on_trip_command(
            command=TRIP_COMMAND.get(m.command, 'unknown'), trip_id=m.trip_id, reason=m.reason), RELIABLE)
        self.create_subscription(PhoneLocation, '/phone/location', self.on_location, RELIABLE)
        self.create_subscription(SosRequest, '/safety/sos_request', lambda m: a.on_sos_request(
            sos_id=m.sos_id, reason=SOS_REASON.get(m.reason, 'unknown'),
            severity=SOS_SEVERITY.get(m.severity, 'critical'), trip_id=m.trip_id or None), RELIABLE)
        self.create_subscription(SosAck, '/safety/sos_ack', lambda m: a.on_sos_ack(
            sos_id=m.sos_id, app_received=m.app_received, retries=m.retries, has_location=m.has_location,
            best_source=LOC_SOURCE.get(m.best_source, 'none')), RELIABLE)
        self.create_subscription(SosClear, '/safety/sos_clear', lambda m: a.on_sos_clear(
            sos_id=m.sos_id, resolution=SOS_RESOLUTION.get(m.resolution, 'unknown')), RELIABLE)
        if BaseStatus is not None:
            self.create_subscription(BaseStatus, '/base/status', lambda m: a.on_base_status(
                agent_state=BASE_AGENT.get(m.agent_state, 'unknown'), cmd_watchdog_ok=m.cmd_watchdog_ok), BASE_QOS)
        if MarkerTrack is not None and p['marker_track_topic']:
            self.create_subscription(MarkerTrack, p['marker_track_topic'], lambda m: a.on_marker(m.status), 10)
        if p['publish_diag']:
            from sensor_msgs.msg import JointState
            self.create_subscription(JointState, '/joint_states', lambda m: a.topic_seen('/joint_states'),
                                     qos_profile_sensor_data)
        self.create_timer(1.0, self.on_bridge_check)
        self.on_bridge_check()
        a.start()

    def _declare_params(self) -> dict:
        defaults = {
            'robot_id': 'mr-001', 'display_name': '돌봄 로봇 1호',
            'database_url': 'https://senior-care-nyr-default-rtdb.asia-southeast1.firebasedatabase.app',
            'service_account_path': '~/.config/mr_care_telemetry/service-account.json',
            'emulator_host': os.environ.get('FIREBASE_DATABASE_EMULATOR_HOST', ''),
            'queue_db_path': '~/.local/state/mr_care_telemetry/queue.db',
            'status_period_s': 5.0, 'app_status_stale_s': 6.0,
            'publish_sos_location': True, 'publish_diag': False, 'max_events': 1000,
            'marker_track_topic': '/marker_track',
        }
        out = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            out[name] = self.get_parameter(name).value
        return out

    def on_app_status(self, m: RobotAppStatus):
        age = 0.0
        if m.header.stamp.sec or m.header.stamp.nanosec:
            age = max(0.0, (self.get_clock().now().nanoseconds - (m.header.stamp.sec * 1_000_000_000
                                                                  + m.header.stamp.nanosec)) / 1e9)
        self.agent.on_app_status(
            mission_state=MISSION.get(m.mission_state, 'unknown'), tracking_state=TRACKING.get(m.tracking_state, 'unknown'),
            trip_id=m.trip_id or None, from_place_id=m.from_place_id or None, to_place_id=m.to_place_id or None,
            battery_percent=None if m.battery_percent < 0 else float(m.battery_percent),
            last_error=m.last_error or None, age_s=age)

    def on_link(self, m: PhoneLinkStatus):
        self.agent.on_link_status(connected=m.connected, app_instance_id=m.app_instance_id,
                                  session_id=m.session_id, last_seen_ms=stamp_ms(m.last_seen))

    def on_location(self, m: PhoneLocation):
        self.agent.on_location(reason=LOC_REASON.get(m.reason, 'unknown'), sos_id=m.sos_id or None,
                               has_fix=m.has_fix, lat=m.lat, lon=m.lon, accuracy_m=m.horizontal_accuracy_m,
                               captured_at_ms=m.captured_at_ms, source=LOC_SOURCE.get(m.source, 'none'),
                               low_confidence=m.low_confidence)

    def on_bridge_check(self):
        # 브리지가 떠 있으면 /phone/link_status 발행자가 있다(TransientLocal로 항상 광고).
        self.agent.on_bridge_alive(self.count_publishers('/phone/link_status') > 0)

    def destroy_node(self):
        for name in PY_LOGGERS:
            lg = logging.getLogger(name)
            if self._log_handler in lg.handlers:
                lg.removeHandler(self._log_handler)
                lg.propagate = True
        return super().destroy_node()


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = None
    try:
        node = TelemetryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.agent.stop()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
