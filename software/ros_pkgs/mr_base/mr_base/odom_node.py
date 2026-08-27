"""`/joint_states` → `/odom` + `odom → base_footprint` TF.

이 노드는 시리얼·게인·제어 주기·틱을 하나도 모른다. 파라미터가 순수 기하학뿐인 것이
"오도메트리는 호스트, 제어는 MCU" 분리가 제대로 됐다는 증거다.

    ros2 run mr_base odom_node --ros-args --params-file config/base_params.yaml
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from mr_base.odometry import DiffDriveOdometry, RebaseReason

try:
    from mr_msgs.msg import BaseStatus
except ImportError:                                     # pragma: no cover
    # `/base/status`는 MCU reset 감지에만 쓴다. 없으면 그 기능만 꺼지고 나머지는 돈다.
    BaseStatus = None

# MCU가 발행하는 세 토픽은 전부 BEST_EFFORT, KEEP_LAST(1)이다. 구독 QoS가 RELIABLE이면
# 호환되지 않아 데이터가 아예 오지 않는다.
MCU_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

DIAGNOSTIC_PERIOD_S = 10.0


class OdomNode(Node):

    def __init__(self) -> None:
        super().__init__('mr_base')

        self.declare_parameter('wheel_radius_m', 0.04979)
        self.declare_parameter('wheel_separation_m', 0.30912)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('left_wheel_joint', 'left_wheel_joint')
        self.declare_parameter('right_wheel_joint', 'right_wheel_joint')
        self.declare_parameter('publish_tf', True)
        # wheel-only 오도메트리는 z/roll/pitch를 관측하지 못하고 횡슬립도 모른다.
        # 0으로 두어 완전한 센서처럼 보이게 하지 않는다. EKF를 붙일 때 다시 맞춘다.
        self.declare_parameter('pose_covariance_diagonal',
                               [1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2])
        self.declare_parameter('twist_covariance_diagonal',
                               [1e-3, 1e6, 1e6, 1e6, 1e6, 1e-2])

        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.odometry = DiffDriveOdometry(
            wheel_radius_m=float(self.get_parameter('wheel_radius_m').value),
            wheel_separation_m=float(self.get_parameter('wheel_separation_m').value),
            left_joint=self.get_parameter('left_wheel_joint').value,
            right_joint=self.get_parameter('right_wheel_joint').value,
        )

        self._pose_cov = _diagonal(self.get_parameter('pose_covariance_diagonal').value)
        self._twist_cov = _diagonal(self.get_parameter('twist_covariance_diagonal').value)

        # `/odom`은 MCU가 아니라 이 노드가 만든다. 소비자(Nav2, robot_localization)의
        # 기본 구독 QoS가 RELIABLE이므로 여기서 BEST_EFFORT로 내보내면 서로 보이지 않는다.
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(JointState, 'joint_states', self.on_joint_state, MCU_QOS)
        if BaseStatus is not None:
            self.create_subscription(BaseStatus, 'base/status', self.on_base_status, MCU_QOS)
        else:
            self.get_logger().warn(
                'mr_msgs를 찾을 수 없다. MCU reset 감지(/base/status.loop_count)가 꺼진다. '
                'position 점프는 wheel_jump 판정으로만 걸러진다.')

        self._last_summary = ''
        self.create_timer(DIAGNOSTIC_PERIOD_S, self.on_diagnostic)

        self.get_logger().info(
            f'mr_base odometry: r={self.odometry.wheel_radius_m:.4f} m '
            f'L={self.odometry.wheel_separation_m:.4f} m '
            f'{self.odom_frame_id} -> {self.base_frame_id} '
            f'publish_tf={self.publish_tf}')

    # ---------------------------------------------------------------- 콜백

    def on_base_status(self, msg) -> None:
        if self.odometry.note_loop_count(int(msg.loop_count)):
            self.get_logger().warn(
                f'MCU reset 감지 (loop_count={msg.loop_count}). 다음 표본은 적분하지 '
                '않고 기준점으로만 쓴다.')

    def on_joint_state(self, msg: JointState) -> None:
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        before = self.odometry.counters.rebase_total

        update = self.odometry.update_from_joint_state(stamp_s, msg.name, msg.position)

        if update is None:
            if self.odometry.counters.rebase_total > before:
                reason = self.odometry.last_rebase_reason
                # rclpy 로거는 호출 지점(파일·행)마다 severity를 기억한다. 한 줄에서
                # info와 warn을 번갈아 부르면 두 번째 호출이
                # `Logger severity cannot be changed between calls`로 죽는다.
                # 그래서 줄을 나눠 둔다.
                if reason == RebaseReason.FIRST_SAMPLE:
                    self.get_logger().info(f'재기준: {reason}')
                else:
                    self.get_logger().warn(f'재기준: {reason}')
            return

        self.publish(update, msg.header.stamp)

    def on_diagnostic(self) -> None:
        summary = self.odometry.counters.summary()
        if summary != self._last_summary:
            self.get_logger().info(summary)
            self._last_summary = summary

    # ---------------------------------------------------------------- 발행

    def publish(self, update, stamp) -> None:
        """`/odom`과 TF는 같은 입력 stamp와 같은 pose를 쓴다."""
        half_yaw = 0.5 * update.yaw
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = update.x
        odom.pose.pose.position.y = update.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = self._pose_cov
        odom.twist.twist.linear.x = update.v
        odom.twist.twist.angular.z = update.w
        odom.twist.covariance = self._twist_cov
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is None:
            return
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame_id
        tf.child_frame_id = self.base_frame_id
        tf.transform.translation.x = update.x
        tf.transform.translation.y = update.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)


def _diagonal(values) -> list:
    """길이 6 대각 성분을 6x6 row-major 공분산으로 편다."""
    if len(values) != 6:
        raise ValueError(f'covariance diagonal must have 6 entries: {list(values)}')
    cov = [0.0] * 36
    for i, value in enumerate(values):
        cov[i * 7] = float(value)
    return cov


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
