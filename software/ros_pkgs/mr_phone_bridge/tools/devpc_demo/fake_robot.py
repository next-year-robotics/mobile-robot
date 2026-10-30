"""개발 PC 데모용 가짜 로봇. demo.sh가 격리 도메인에서 띄운다. 실제 로봇 도메인(0)에서 띄우지 않는다.

- /follow/enable (std_srvs/SetBool): mr_mission이 부른다. 켜지면 /odom 속도 0.5 m/s, 꺼지면 0.
- /marker_track (mr_aruco_msgs/MarkerTrack) 10 Hz: 기본 TRACKING. mr_aruco_msgs가 없으면 발행하지 않는다
  (그러면 tracking_state는 unavailable).
- /odom (nav_msgs/Odometry) 20 Hz: mission의 정지 확인용.
- /demo/cmd (std_msgs/String): lost | found | sos [reason] | clear [resolution]
  sos·clear는 /safety/sos_request·/safety/sos_clear를 직접 낸다(SOS 화면 시험용). mission도 lost가 10 s 이어지면
  스스로 SOS를 낸다(M3, 2026-09-27).
"""
import signal
import uuid

import rclpy
from mr_care_msgs.msg import RobotAppStatus, SosClear, SosRequest
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool

try:  # ArUco 브랜치 메시지. 없으면 마커 없이 돈다.
    from mr_aruco_msgs.msg import MarkerTrack
except ImportError:
    MarkerTrack = None

MARKER_LOST, MARKER_TRACKING = 0, 1
REASONS = {'tracking_lost': SosRequest.REASON_TRACKING_LOST, 'stuck': SosRequest.REASON_STUCK,
           'low_battery': SosRequest.REASON_LOW_BATTERY, 'manual': SosRequest.REASON_MANUAL}
RESOLUTIONS = {'tracking_recovered': SosClear.RESOLUTION_TRACKING_RECOVERED,
               'handled': SosClear.RESOLUTION_HANDLED, 'cancelled': SosClear.RESOLUTION_CANCELLED}


class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot')
        self.following = False
        self.marker_status = MARKER_TRACKING
        self.last_sos = None
        self.trip_id = ''
        self.create_service(SetBool, '/follow/enable', self.on_enable)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.sos_pub = self.create_publisher(SosRequest, '/safety/sos_request', 10)
        self.clear_pub = self.create_publisher(SosClear, '/safety/sos_clear', 10)
        self.create_subscription(String, '/demo/cmd', self.on_cmd, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(RobotAppStatus, '/robot/app_status',
                                 lambda m: setattr(self, 'trip_id', m.trip_id), latched)
        self.create_timer(0.05, self.pub_odom)
        if MarkerTrack is not None:
            self.marker_pub = self.create_publisher(MarkerTrack, '/marker_track', 10)
            self.create_timer(0.1, self.pub_marker)
        self.get_logger().info('fake robot ready: follow off, marker '
                               + ('tracking' if MarkerTrack is not None else 'unavailable (no mr_aruco_msgs)'))

    def on_enable(self, req, res):
        self.following = bool(req.data)
        res.success, res.message = True, 'fake follow ' + ('on' if req.data else 'off')
        self.get_logger().info(res.message)
        return res

    def pub_marker(self):
        m = MarkerTrack()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_footprint'
        m.status = self.marker_status
        m.locked = self.marker_status != MARKER_LOST
        m.marker_id = 7
        m.x = 1.2 if m.locked else 0.0
        self.marker_pub.publish(m)

    def pub_odom(self):
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id, o.child_frame_id = 'odom', 'base_footprint'
        o.twist.twist.linear.x = 0.5 if (self.following and self.marker_status != MARKER_LOST) else 0.0
        self.odom_pub.publish(o)

    def on_cmd(self, msg):
        words = msg.data.split()
        cmd = words[0] if words else ''
        arg = words[1] if len(words) > 1 else None
        if cmd == 'lost' and MarkerTrack is not None:
            self.marker_status = MARKER_LOST
        elif cmd == 'found' and MarkerTrack is not None:
            self.marker_status = MARKER_TRACKING
        elif cmd == 'sos' and (arg or 'tracking_lost') in REASONS:
            self.last_sos = str(uuid.uuid4())
            self.sos_pub.publish(SosRequest(sos_id=self.last_sos, reason=REASONS[arg or 'tracking_lost'],
                                            severity=SosRequest.SEVERITY_CRITICAL, trip_id=self.trip_id))
        elif cmd == 'clear' and self.last_sos and (arg or 'tracking_recovered') in RESOLUTIONS:
            self.clear_pub.publish(SosClear(sos_id=self.last_sos,
                                            resolution=RESOLUTIONS[arg or 'tracking_recovered']))
            self.last_sos = None
        else:
            self.get_logger().warn(f'ignored demo command: {msg.data!r}')
            return
        self.get_logger().info(f'demo command: {msg.data}')


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    # demo.sh가 백그라운드에서 띄우면 SIGINT가 SIG_IGN으로 상속된다. mr_mission과 같이 처리기를 명시한다.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
