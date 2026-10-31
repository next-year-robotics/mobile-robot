"""mr_phone_bridge ROS 2 노드. WebSocket 서버(asyncio)를 별도 스레드에서 돌리고 ROS executor와 큐로 잇는다.

    ros2 launch mr_phone_bridge phone_bridge.launch.py

발행: /phone/trip_command, /phone/location, /phone/link_status(TransientLocal), /robot/place_catalog(TransientLocal),
      /safety/sos_ack
구독: /robot/app_status(TransientLocal), /safety/sos_request, /safety/sos_clear, /marker_track(있으면, 폴백)
토픽·QoS 정본: PROTOCOL.md 6장 15.2, 7.2. 이 노드는 /cmd_vel을 발행하지도 구독하지도 않는다.
"""
from __future__ import annotations

import asyncio
import logging
import math
import queue
import signal
import threading

import rclpy
from builtin_interfaces.msg import Time
from mr_care_msgs.msg import (PhoneLinkStatus, PhoneLocation, Place, PlaceCatalog, RobotAppStatus, SosAck,
                              SosClear, SosRequest, TripCommand)
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

try:  # ArUco 브랜치(mr_aruco_msgs)가 설치돼 있을 때만 폴백 구독
    from mr_aruco_msgs.msg import MarkerTrack
except ImportError:  # pragma: no cover
    MarkerTrack = None

from ..places.store import PlaceStore, PlaceStoreError
from ..server.bridge import BridgeConfig, BridgeCore
from . import enums
from .backend import RosBackend

RELIABLE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)
LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

INT32 = (-2**31, 2**31 - 1)
INT64 = (-2**63, 2**63 - 1)
FLOAT32_MAX = 3.4e38


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def ms_to_time(ms: int) -> Time:
    t = Time()
    if ms > 0:
        t.sec = int(ms // 1000)
        t.nanosec = int(ms % 1000) * 1_000_000
    return t


class RosLogHandler(logging.Handler):
    """코어의 파이썬 logging을 ROS 로거(/rosout)로 보낸다."""

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


class AsyncioThread:
    """asyncio 루프 하나를 데몬 스레드에서 돌린다."""

    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run, name='phone-bridge-ws', daemon=True)

    def _run(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def start(self):
        self._thread.start()

    def run(self, coro, timeout: float | None = None):
        return asyncio.run_coroutine_threadsafe(coro, self.loop).result(timeout)

    def stop(self):
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
        self._thread.join(5.0)
        if not self.loop.is_running():
            self.loop.close()


class PhoneBridgeNode(Node):

    def __init__(self, **kwargs):
        super().__init__('mr_phone_bridge', **kwargs)       # kwargs: 시험용 context·parameter_overrides
        p = self._declare_params()
        self._setup_logging()

        self.backend = RosBackend(app_status_stale_s=p['app_status_stale_s'],
                                  marker_stale_s=p['marker_stale_s'])
        store = PlaceStore(p['places_path'], max_places=p['max_places'])
        store.load()          # 손상돼 있으면 여기서 PlaceStoreError → 기동 실패
        cfg = BridgeConfig(robot_id=p['robot_id'], path=p['path'], place_edit_enabled=p['place_edit_enabled'],
                           heartbeat_interval_ms=p['heartbeat_interval_ms'],
                           heartbeat_timeout_ms=p['heartbeat_timeout_ms'], status_period_s=p['status_period_s'],
                           sos_retry_interval_s=p['sos_retry_interval_s'], sos_max_retries=p['sos_max_retries'],
                           dedupe_ttl_s=p['dedupe_ttl_s'], max_frame_bytes=p['max_frame_bytes'])
        self.core = BridgeCore(cfg, store, self.backend)

        self.pub_trip = self.create_publisher(TripCommand, '/phone/trip_command', RELIABLE)
        self.pub_location = self.create_publisher(PhoneLocation, '/phone/location', RELIABLE)
        self.pub_link = self.create_publisher(PhoneLinkStatus, '/phone/link_status', LATCHED)
        self.pub_catalog = self.create_publisher(PlaceCatalog, '/robot/place_catalog', LATCHED)
        self.pub_sos_ack = self.create_publisher(SosAck, '/safety/sos_ack', RELIABLE)
        # 앱의 SOS 버튼·'괜찮아요'(sos.raise/sos.cancel). mission이 받아 /safety/sos_request·sos_clear로 낸다
        self.pub_sos_raise = self.create_publisher(SosRequest, '/phone/sos_raise', RELIABLE)
        self.pub_sos_cancel = self.create_publisher(SosClear, '/phone/sos_cancel', RELIABLE)

        self.create_subscription(RobotAppStatus, '/robot/app_status', self.on_app_status, LATCHED)
        self.create_subscription(SosRequest, '/safety/sos_request', self.on_sos_request, RELIABLE)
        self.create_subscription(SosClear, '/safety/sos_clear', self.on_sos_clear, RELIABLE)
        if MarkerTrack is not None and p['marker_track_topic']:
            self.create_subscription(MarkerTrack, p['marker_track_topic'], self.on_marker, 10)
        else:
            self.get_logger().info('mr_aruco_msgs not available: tracking_state falls back to "unavailable"')

        self.create_timer(0.02, self.drain)

        try:
            from ..server.ws import BridgeServer
        except ImportError as e:
            raise RuntimeError(f'{e}: install python3-websockets (sudo apt install python3-websockets)') from None
        self.server = BridgeServer(self.core, p['bind_host'], p['port'])
        self.aio = AsyncioThread()
        self.aio.start()
        try:
            self.aio.run(self.server.start(), timeout=10.0)
        except OSError as e:
            self.aio.stop()
            raise RuntimeError(f'cannot listen on {p["bind_host"]}:{p["port"]}: {e.strerror or e}') from None
        self.get_logger().info(
            f'phone-bridge/1 on ws://{p["bind_host"]}:{self.server.port}{p["path"]} robot_id={p["robot_id"]} '
            f'places={store.path}')

    def _declare_params(self) -> dict:
        defaults = {
            'robot_id': 'mr-001', 'bind_host': '0.0.0.0', 'port': 8765, 'path': '/bridge/v1',
            'places_path': '~/ros2_ws/config/places.yaml', 'max_places': 12,
            'heartbeat_interval_ms': 2000, 'heartbeat_timeout_ms': 10000, 'status_period_s': 2.0,
            'sos_retry_interval_s': 2.0, 'sos_max_retries': 3, 'dedupe_ttl_s': 600.0, 'max_frame_bytes': 16384,
            'place_edit_enabled': True, 'app_status_stale_s': 6.0, 'marker_stale_s': 1.0,
            'marker_track_topic': '/marker_track',
        }
        out = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            out[name] = self.get_parameter(name).value
        return out

    def _setup_logging(self):
        root = logging.getLogger('mr_phone_bridge')
        root.handlers[:] = [RosLogHandler(self.get_logger())]
        root.setLevel(logging.INFO)
        root.propagate = False
        logging.getLogger('websockets').setLevel(logging.WARNING)

    # ------------------------------------------------------------ ROS → 코어

    def _age_s(self, stamp) -> float:
        if stamp.sec == 0 and stamp.nanosec == 0:
            return 0.0
        now = self.get_clock().now().nanoseconds
        return max(0.0, (now - (stamp.sec * 1_000_000_000 + stamp.nanosec)) / 1e9)

    def on_app_status(self, msg: RobotAppStatus):
        self.backend.update_app_status({
            'mission_state': enums.to_str(enums.MISSION, msg.mission_state),
            'tracking_state': enums.to_str(enums.TRACKING, msg.tracking_state),
            'trip_id': msg.trip_id or None,
            'from_place_id': msg.from_place_id or None,
            'to_place_id': msg.to_place_id or None,
            'battery_percent': None if msg.battery_percent < 0 else int(msg.battery_percent),
            'last_error': msg.last_error or None,
        }, self._age_s(msg.header.stamp))
        self.core.call_threadsafe(self.core.notify_status)

    def on_marker(self, msg):
        self.backend.update_marker(int(msg.status))     # 변화는 코어의 0.1 s 틱이 반영한다

    def on_sos_request(self, msg: SosRequest):
        wire = self.backend.remember_sos(msg.sos_id)
        self.core.call_threadsafe(self.core.request_sos, wire, enums.to_str(enums.SOS_REASON, msg.reason),
                                  enums.to_str(enums.SOS_SEVERITY, msg.severity), msg.trip_id or None)

    def on_sos_clear(self, msg: SosClear):
        wire = self.backend.remember_sos(msg.sos_id)
        self.core.call_threadsafe(self.core.clear_sos, wire, enums.to_str(enums.SOS_RESOLUTION, msg.resolution))

    # ------------------------------------------------------------ 코어 → ROS

    def drain(self):
        for _ in range(200):
            try:
                kind, data = self.backend.outbox.get_nowait()
            except queue.Empty:
                return
            try:
                self._publish(kind, data)
            except Exception as e:           # 변환 버그가 브리지를 멈추지 않게
                self.get_logger().error(f'publish {kind} failed: {e!r}')
                self.backend.internal_error = f'publish {kind} failed'

    def _publish(self, kind: str, d: dict):
        stamp = self.get_clock().now().to_msg()
        if kind == 'trip_command':
            m = TripCommand()
            m.header.stamp = stamp
            m.request_id = d['request_id']
            m.trip_id = d['trip_id']
            m.command = enums.to_int(enums.TRIP_COMMAND, d['command'])
            m.from_place_id = d['from_place_id']
            m.to_place_id = d['to_place_id']
            m.reason = d['reason']
            self.pub_trip.publish(m)
        elif kind == 'location':
            self.pub_location.publish(self.location_msg(d, stamp))
        elif kind == 'link_status':
            m = PhoneLinkStatus()
            m.header.stamp = stamp
            m.connected = d['connected']
            m.app_instance_id = d['app_instance_id']
            m.session_id = d['session_id']
            m.last_seen = ms_to_time(d['last_seen_ms'])
            self.pub_link.publish(m)
        elif kind == 'catalog':
            self.pub_catalog.publish(self.catalog_msg(d, stamp))
        elif kind == 'sos_ack':
            m = SosAck()
            m.header.stamp = stamp
            m.sos_id = d['sos_id']
            m.app_received = d['app_received']
            m.retries = clamp(d['retries'], 0, 255)
            m.has_location = d['has_location']
            m.best_source = enums.to_int(enums.LOCATION_SOURCE, d['best_source'])
            self.pub_sos_ack.publish(m)
        elif kind == 'sos_raise':
            m = SosRequest()
            m.header.stamp = stamp
            m.sos_id = d['sos_id']
            m.reason = SosRequest.REASON_MANUAL
            m.severity = SosRequest.SEVERITY_CRITICAL
            m.trip_id = d['trip_id']
            self.pub_sos_raise.publish(m)
        elif kind == 'sos_cancel':
            m = SosClear()
            m.header.stamp = stamp
            m.sos_id = d['sos_id']
            m.resolution = SosClear.RESOLUTION_CANCELLED
            self.pub_sos_cancel.publish(m)
        else:
            raise ValueError(f'unknown outbox kind {kind}')

    @staticmethod
    def location_msg(d: dict, stamp) -> PhoneLocation:
        p = d['payload']
        m = PhoneLocation()
        m.header.stamp = stamp
        m.message_id = d['message_id']
        m.reason = enums.to_int(enums.LOCATION_REASON, p['reason'])
        m.sos_id = p['sos_id'] or ''
        m.has_fix = p['has_fix']
        m.lat = float(p['lat']) if p['lat'] is not None else math.nan
        m.lon = float(p['lon']) if p['lon'] is not None else math.nan
        acc = p['horizontal_accuracy_m']
        m.horizontal_accuracy_m = min(float(acc), FLOAT32_MAX) if acc is not None else math.nan
        m.captured_at_ms = clamp(int(p['captured_at_ms']), *INT64) if p['captured_at_ms'] is not None else 0
        m.age_ms = clamp(int(p['age_ms']), *INT64) if p['age_ms'] is not None else -1
        m.source = enums.to_int(enums.LOCATION_SOURCE, p['source'])
        m.failure_reason = enums.to_int(enums.LOCATION_FAILURE, p['failure_reason'])
        m.low_confidence = d['low_confidence']
        return m

    @staticmethod
    def catalog_msg(payload: dict, stamp) -> PlaceCatalog:
        m = PlaceCatalog()
        m.header.stamp = stamp
        m.revision = payload['revision']
        m.max_places = clamp(payload['max_places'], 0, 255)
        m.default_origin_place_id = payload['default_origin_place_id'] or ''
        for p in payload['places']:
            pl = Place()
            pl.id = p['id']
            pl.kind = p['kind']
            pl.label = p['label']
            pl.lat = float(p['lat'])
            pl.lon = float(p['lon'])
            pl.radius_m = float(p['radius_m'])
            pl.enabled = p['enabled']
            pl.sort_order = clamp(int(p['sort_order']), *INT32)
            pl.is_default_origin = p['is_default_origin']
            pl.address = p['address'] or ''
            m.places.append(pl)
        return m

    # ------------------------------------------------------------ 종료

    def shutdown(self):
        try:
            self.aio.run(self.server.stop(), timeout=8.0)
        except Exception as e:
            self.get_logger().warning(f'bridge stop: {e!r}')
        self.aio.stop()
        self.drain()          # 마지막 링크 끊김 발행


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
        node = PhoneBridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (PlaceStoreError, RuntimeError) as e:
        rclpy.logging.get_logger('mr_phone_bridge').fatal(str(e))
        raise SystemExit(1)
    finally:
        # launch는 그룹 kill(-INT -PGID) 뒤 SIGINT를 한 번 더 전달한다. 정리 중 두 번째 SIGINT로 끊기지 않게 한다.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
