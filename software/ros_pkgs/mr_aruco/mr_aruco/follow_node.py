"""`/marker_track` → `/cmd_vel_follow`. 마커를 단 작업자를 일정 거리로 따라간다.

제어법은 `follow_control.py`에 있고 여기는 배선만 한다.

발행에 두 가지 규칙이 있다.

1. **비활성이면 아무것도 발행하지 않는다.** twist_mux는 입력이 끊기면 그 입력을
   무시하므로, 발행을 멈추는 것이 곧 "제어권을 놓는 것"이다. 0을 계속 쏘면
   우선순위가 낮은 다른 입력까지 막는다.
2. **비활성으로 전이할 때는 0을 한 번 쏜다.** MCU 워치독(200 ms)을 기다리지 않고
   즉시 서기 위해서다. 워치독은 그 명령이 유실됐을 때의 backstop으로 남긴다.

    ros2 run mr_aruco follow_node --ros-args --params-file config/follow.yaml
    ros2 service call /follow/enable std_srvs/srv/SetBool "{data: true}"
"""
from __future__ import annotations

from geometry_msgs.msg import TwistStamped
from mr_aruco.follow_control import FollowConfig, FollowController
from mr_aruco.tracking import PlanarPose
from mr_aruco_msgs.msg import MarkerTrack
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import SetBool

DIAGNOSTIC_PERIOD_S = 5.0


class FollowNode(Node):

    def __init__(self) -> None:
        super().__init__('follow')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('enabled_on_start', False)
        self.declare_parameter('cmd_frame_id', 'base_footprint')
        self.declare_parameter('target_range_m', 1.1)
        self.declare_parameter('k_range', 0.6)
        self.declare_parameter('k_bearing', 1.2)
        self.declare_parameter('k_bearing_d', 0.1)
        self.declare_parameter('derivative_tau_s', 0.15)
        self.declare_parameter('max_linear_mps', 0.25)
        self.declare_parameter('max_angular_rps', 0.7)
        self.declare_parameter('max_linear_accel', 0.4)
        self.declare_parameter('max_angular_accel', 2.0)
        self.declare_parameter('range_deadband_m', 0.12)
        self.declare_parameter('bearing_deadband_rad', 0.07)
        self.declare_parameter('align_first_rad', 0.44)
        self.declare_parameter('min_linear_mps', 0.005)
        self.declare_parameter('min_angular_rps', 0.01)
        self.declare_parameter('marker_timeout_s', 0.4)

        self.controller = FollowController(FollowConfig(
            target_range_m=self._f('target_range_m'),
            k_range=self._f('k_range'),
            k_bearing=self._f('k_bearing'),
            k_bearing_d=self._f('k_bearing_d'),
            derivative_tau_s=self._f('derivative_tau_s'),
            max_linear_mps=self._f('max_linear_mps'),
            max_angular_rps=self._f('max_angular_rps'),
            max_linear_accel=self._f('max_linear_accel'),
            max_angular_accel=self._f('max_angular_accel'),
            range_deadband_m=self._f('range_deadband_m'),
            bearing_deadband_rad=self._f('bearing_deadband_rad'),
            align_first_rad=self._f('align_first_rad'),
            min_linear_mps=self._f('min_linear_mps'),
            min_angular_rps=self._f('min_angular_rps'),
            marker_timeout_s=self._f('marker_timeout_s'),
        ))

        self.cmd_frame_id = self.get_parameter('cmd_frame_id').value
        self.enabled = bool(self.get_parameter('enabled_on_start').value)

        # twist_mux 입력은 RELIABLE이어야 한다. 그쪽이 SystemDefaultsQoS로 구독하므로
        # BEST_EFFORT로 내보내면 DDS가 아예 연결하지 않는다. 증상은 "안 움직인다"이고
        # 단서는 twist_mux 쪽의 incompatible QoS 경고다.
        # (base_contract.md "인지·자율 제어 계층" 절)
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel_follow', 10)

        self.create_subscription(MarkerTrack, 'marker_track', self.on_track, 10)
        self.create_service(SetBool, '~/enable', self.on_enable)

        self._track = None
        self._was_stale = True
        self._idle = True
        rate = self._f('control_rate_hz')
        self.create_timer(1.0 / rate, self.on_control)
        self.create_timer(DIAGNOSTIC_PERIOD_S, self.on_diagnostic)

        self.get_logger().info(
            f'follow: {rate:.0f} Hz, 목표 거리 {self.controller.config.target_range_m:.2f} m, '
            f'v_max {self.controller.config.max_linear_mps:.2f} m/s, '
            f'시작 상태 {"활성" if self.enabled else "비활성"}')

    # ---------------------------------------------------------------- 콜백

    def on_track(self, msg: MarkerTrack) -> None:
        self._track = msg

    def on_enable(self, request, response):
        if request.data == self.enabled:
            response.success = True
            response.message = f'이미 {"활성" if self.enabled else "비활성"}이다.'
            return response
        self.enabled = request.data
        if self.enabled:
            self.controller.reset()
            self._was_stale = True
            self._idle = True
        else:
            self._publish(0.0, 0.0)
        self.get_logger().info(f'추종 {"시작" if self.enabled else "정지"}')
        response.success = True
        response.message = '활성' if self.enabled else '비활성'
        return response

    def on_control(self) -> None:
        if not self.enabled:
            return

        now = self._now()
        pose, stamp = self._sample()
        cmd = self.controller.step(pose, stamp, now)

        if cmd.stale and not self._was_stale:
            self.get_logger().warn('마커 표본이 낡았다. 감속해 정지한다.')
        elif not cmd.stale and self._was_stale:
            self.get_logger().info('마커를 다시 잡았다.')
        self._was_stale = cmd.stale

        idle = cmd.stale and cmd.v == 0.0 and cmd.w == 0.0
        if idle and self._idle:
            # 이미 정지 명령을 보내고 제어권을 놓았다. 여기서 0을 계속 쏘면
            # twist_mux가 이 입력을 살아 있는 것으로 보고 우선순위가 낮은
            # 입력까지 막는다.
            return

        # idle로 막 들어온 첫 주기는 반드시 발행한다. 0을 한 번 확실히 보내
        # 즉시 서고, MCU 워치독은 그 명령이 유실됐을 때의 backstop으로 남긴다.
        # 마커 유실은 정상 동작 중에도 늘 일어나므로 워치독에 기대면 안 된다.
        self._idle = idle
        self._publish(cmd.v, cmd.w)

    def on_diagnostic(self) -> None:
        if not self.enabled or self._track is None:
            return
        pose, _ = self._sample()
        if pose is None:
            return
        self.get_logger().info(
            f'rho={pose.range_m:.2f} m alpha={pose.bearing_rad:+.2f} rad '
            f'v={self.controller.prev_v:+.3f} w={self.controller.prev_w:+.3f}')

    # ---------------------------------------------------------------- 내부

    def _sample(self):
        """최신 추적 메시지를 제어기 입력으로 바꾼다."""
        msg = self._track
        if msg is None or msg.status == MarkerTrack.STATUS_LOST:
            return None, None
        pose = PlanarPose(x=msg.x, y=msg.y, yaw=msg.yaw)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return pose, stamp

    def _publish(self, v: float, w: float) -> None:
        msg = TwistStamped()
        # stamp는 반드시 발행 직전 현재 시각이다. twist_mux는 stamp를 다시 찍지 않고
        # 그대로 전달하며, MCU는 낡은 stamp의 명령을 거절한다.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cmd_frame_id
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self.cmd_pub.publish(msg)

    def _f(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
