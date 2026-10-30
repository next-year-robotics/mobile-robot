"""`/imu/data_raw` → `/imu/data`. 공분산만 채워 다시 낸다.

MCU가 보내는 것은 SI 단위로 바꾸고 자이로 bias를 뺀 원시값이다. 공분산은 0(모름),
orientation은 없음(-1)이다. 이 노드는 공분산만 파라미터로 채운다. 축 회전은 하지
않는다. 그건 URDF `imu_link` 장착 자세와 tf가 한다(REP-145).

    ros2 run mr_base imu_node --ros-args --params-file config/base_params.yaml
"""
from __future__ import annotations

from mr_base.imu_covariance import covariance_from_variances
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

# MCU 토픽은 BEST_EFFORT, KEEP_LAST(1)이다. RELIABLE로 구독하면 아무것도 오지 않는다.
MCU_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

DIAGNOSTIC_PERIOD_S = 10.0

# ICM-42688-P 데이터시트(DS-000347 Rev 1.6) 노이즈 밀도 × √NBW. NBW 25.9 Hz는
# 펌웨어 설정(ODR 100 Hz, 2차 UI 필터, BW 인덱스 5)의 표값이다.
#   자이로 0.0028 °/s/√Hz → 0.01425 °/s = 2.487e-4 rad/s → 6.19e-8
#   가속 x/y 65 µg/√Hz → 3.244e-3 m/s² → 1.05e-5, z 70 µg/√Hz → 3.493e-3 → 1.22e-5
# 진동 없는 하한이다. EKF를 붙일 때 정지·주행 실측으로 다시 맞춘다.
DEFAULT_GYRO_VARIANCE = [6.19e-8, 6.19e-8, 6.19e-8]
DEFAULT_ACCEL_VARIANCE = [1.05e-5, 1.05e-5, 1.22e-5]


class ImuNode(Node):

    def __init__(self) -> None:
        super().__init__('mr_imu')

        self.declare_parameter('angular_velocity_variance', DEFAULT_GYRO_VARIANCE)
        self.declare_parameter('linear_acceleration_variance', DEFAULT_ACCEL_VARIANCE)

        # 잘못된 값이면 여기서 죽는다. 0 공분산으로 조용히 도는 것보다 낫다.
        self._gyro_cov = covariance_from_variances(
            self.get_parameter('angular_velocity_variance').value)
        self._accel_cov = covariance_from_variances(
            self.get_parameter('linear_acceleration_variance').value)

        # 소비자(robot_localization, imu_filter_madgwick)의 기본 구독 QoS가 RELIABLE이다.
        self._pub = self.create_publisher(Imu, 'imu/data', 10)
        self.create_subscription(Imu, 'imu/data_raw', self.on_imu, MCU_QOS)

        self._count = 0
        self._last_count = 0
        self.create_timer(DIAGNOSTIC_PERIOD_S, self.on_diagnostic)

        self.get_logger().info(
            f'mr_imu: gyro var {self._gyro_cov[0]:.3g}/{self._gyro_cov[4]:.3g}/'
            f'{self._gyro_cov[8]:.3g}, accel var {self._accel_cov[0]:.3g}/'
            f'{self._accel_cov[4]:.3g}/{self._accel_cov[8]:.3g}')

    def on_imu(self, msg: Imu) -> None:
        # 받은 메시지를 그대로 고쳐 낸다. stamp·frame_id·값은 MCU 것이다. stamp는 샘플
        # 시각(DRDY)이라 여기서 다시 찍으면 안 된다.
        msg.angular_velocity_covariance = self._gyro_cov
        msg.linear_acceleration_covariance = self._accel_cov
        msg.orientation_covariance[0] = -1.0
        self._pub.publish(msg)
        self._count += 1

    def on_diagnostic(self) -> None:
        rate = (self._count - self._last_count) / DIAGNOSTIC_PERIOD_S
        self._last_count = self._count
        if rate == 0.0:
            # MCU는 바퀴가 2초 멈춰 bias를 잡기 전에는 내보내지 않는다.
            self.get_logger().warn(
                '/imu/data_raw 없음. agent 연결, 펌웨어 imu_ready, '
                'imu_bias_valid(부팅 뒤 바퀴 2 s 정지)를 본다.')
        else:
            self.get_logger().debug(f'/imu/data {rate:.1f} Hz')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuNode()
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
