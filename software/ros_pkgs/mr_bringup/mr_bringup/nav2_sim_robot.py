"""Nav2 + slam_toolbox 시뮬레이션용 2D 실내 차동구동·RPLIDAR C1 시뮬레이터 노드.

실제 하드웨어 없이 PC 한 대에서 `ros2 launch mr_bringup nav_sim.launch.py`로
실제 `slam_toolbox` 2D 지도 작성 → Nav2 A* 전역 경로 생성(중앙 장애물 우회) →
Regulated Pure Pursuit 추종 → Collision Monitor 정지까지 end-to-end로 검증한다.

- 입력: `/cmd_vel` (`geometry_msgs/TwistStamped`, `twist_mux` 출력과 동일)
        `/sim/obstacle` (`std_msgs/Bool`, 동적 장애물 출현 토글)
- 출력: `/odometry/filtered`, `/odom`, `TF odom → base_footprint`, `/joint_states`,
        `/scan` (`laser_link`, `robot_params.yaml`의 RPLIDAR C1 장착 오프셋·yaw 177.5° 반영)
"""
from __future__ import annotations

import math

import numpy as np

# 실내 방과 가구 배치(m): 중앙 커피 테이블이 있어 출발점(-1.8, -1.1) → 책상(2.0, 1.2) 직선 경로를
# 가로막는다. Nav2 A* 전역 경로 계획기가 이 장애물을 피해 곡선 경로를 만들어야 도착한다.
ROOM_WALLS = [
    (-2.6, 3.6, -2.2, -2.1),   # 남쪽 벽
    (-2.6, 3.6,  2.1,  2.2),   # 북쪽 벽
    (-2.7, -2.6, -2.2, 2.2),   # 서쪽 벽
    (3.5,  3.6, -2.2,  2.2),   # 동쪽 벽
    (-0.35, -0.18,  0.65, 2.15),   # 내부 가벽(북)
    (-0.35, -0.18, -2.15, -1.20),  # 내부 가벽(남) — 폭 1.85 m 통로
    (0.85,  1.75, -0.85,  0.10),   # 중앙 커피 테이블(우회 유도)
    (2.55,  3.45,  0.70,  1.85),   # 목적지 책상
]

LIDAR_XY = (0.018, -0.004)          # robot_params.yaml 실측값
LIDAR_YAW = math.radians(177.5)
LIDAR_RAYS = 360
LIDAR_MAX_RANGE = 10.0
R_WHEEL = 0.04979
WHEEL_SEP = 0.30912


def boxes_to_segments(boxes: list[tuple[float, float, float, float]]) -> np.ndarray:
    segs = []
    for x0, x1, y0, y1 in boxes:
        segs.extend([
            (x0, y0, x1, y0),
            (x1, y0, x1, y1),
            (x1, y1, x0, y1),
            (x0, y1, x0, y0),
        ])
    return np.array(segs, dtype=np.float64)


def raycast_scan(
    origin: np.ndarray,
    angles: np.ndarray,
    segs: np.ndarray,
    circles: list[tuple[float, float, float]],
    max_range: float = LIDAR_MAX_RANGE,
) -> np.ndarray:
    dx = np.cos(angles)
    dy = np.sin(angles)
    x1, y1, x2, y2 = segs[:, 0], segs[:, 1], segs[:, 2], segs[:, 3]
    ex, ey = x2 - x1, y2 - y1
    aox, aoy = x1 - origin[0], y1 - origin[1]
    den = dx[:, None] * ey[None, :] - dy[:, None] * ex[None, :]
    valid = np.abs(den) > 1e-10
    safe = np.where(valid, den, 1.0)
    t = (aox[None, :] * ey[None, :] - aoy[None, :] * ex[None, :]) / safe
    u = (aox[None, :] * dy[:, None] - aoy[None, :] * dx[:, None]) / safe
    hit = valid & (t > 0.05) & (u >= 0.0) & (u <= 1.0)
    best = np.min(np.where(hit, t, max_range), axis=1)
    for cx, cy, r in circles:
        ocx, ocy = origin[0] - cx, origin[1] - cy
        bq = dx * ocx + dy * ocy
        disc = bq * bq - (ocx * ocx + ocy * ocy - r * r)
        tc = np.where(disc >= 0.0, -bq - np.sqrt(np.maximum(disc, 0.0)), max_range)
        best = np.minimum(best, np.where(tc > 0.05, tc, max_range))
    return best


def main(args=None) -> None:
    import rclpy
    from geometry_msgs.msg import TransformStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState, LaserScan
    from std_msgs.msg import Bool
    import tf2_ros

    class Nav2SimRobot(Node):

        def __init__(self) -> None:
            super().__init__('nav2_sim_robot')
            self.declare_parameter('initial_x', -1.80)
            self.declare_parameter('initial_y', -1.00)
            self.declare_parameter('initial_yaw', 0.25)
            self.declare_parameter('obstacle_x', 0.45)
            self.declare_parameter('obstacle_y', 0.35)
            self.declare_parameter('obstacle_radius', 0.24)

            self.x = float(self.get_parameter('initial_x').value)
            self.y = float(self.get_parameter('initial_y').value)
            self.yaw = float(self.get_parameter('initial_yaw').value)
            self.obs_xy = (
                float(self.get_parameter('obstacle_x').value),
                float(self.get_parameter('obstacle_y').value),
            )
            self.obs_r = float(self.get_parameter('obstacle_radius').value)
            self.obs_active = False

            self.v = 0.0
            self.w = 0.0
            self.cmd_v = 0.0
            self.cmd_w = 0.0
            self.cmd_at = -1e9
            self.wl = 0.0
            self.wr = 0.0
            self.segs = boxes_to_segments(ROOM_WALLS)
            self.inc = math.tau / LIDAR_RAYS
            self.scan_angles = -math.pi + self.inc * np.arange(LIDAR_RAYS)

            self._tf = tf2_ros.TransformBroadcaster(self)
            self._pub_odom = self.create_publisher(Odometry, '/odom', 10)
            self._pub_filt = self.create_publisher(Odometry, '/odometry/filtered', 10)
            self._pub_js = self.create_publisher(JointState, '/joint_states', 10)
            self._pub_scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)

            self.create_subscription(TwistStamped, '/cmd_vel', self._on_cmd, 10)
            self.create_subscription(Bool, '/sim/obstacle', self._on_obs, 10)

            self.dt = 0.02  # 50 Hz
            self.create_timer(self.dt, self._step)
            self.create_timer(0.1, self._publish_scan)  # 10 Hz RPLIDAR C1
            self.get_logger().info(
                f'nav2_sim_robot 기동: start=({self.x:.2f}, {self.y:.2f}, {math.degrees(self.yaw):.0f}°)'
            )

        def _on_cmd(self, msg: TwistStamped) -> None:
            self.cmd_v = max(-0.15, min(0.30, float(msg.twist.linear.x)))
            self.cmd_w = max(-0.80, min(0.80, float(msg.twist.angular.z)))
            self.cmd_at = self.get_clock().now().nanoseconds * 1e-9

        def _on_obs(self, msg: Bool) -> None:
            self.obs_active = bool(msg.data)
            self.get_logger().info(f'동적 장애물: {"ON" if self.obs_active else "OFF"}')

        def _step(self) -> None:
            now_msg = self.get_clock().now().to_msg()
            now_s = now_msg.sec + now_msg.nanosec * 1e-9
            tv, tw = (self.cmd_v, self.cmd_w) if now_s - self.cmd_at < 0.5 else (0.0, 0.0)

            dv = max(-0.6 * self.dt, min(0.4 * self.dt, tv - self.v))
            dw = max(-2.5 * self.dt, min(2.0 * self.dt, tw - self.w))
            self.v += dv
            self.w += dw
            self.yaw = math.atan2(math.sin(self.yaw + self.w * self.dt), math.cos(self.yaw + self.w * self.dt))
            self.x += self.v * math.cos(self.yaw) * self.dt
            self.y += self.v * math.sin(self.yaw) * self.dt

            vl = (self.v - self.w * WHEEL_SEP * 0.5) / R_WHEEL
            vr = (self.v + self.w * WHEEL_SEP * 0.5) / R_WHEEL
            self.wl += vl * self.dt
            self.wr += vr * self.dt

            qz = math.sin(0.5 * self.yaw)
            qw = math.cos(0.5 * self.yaw)

            tf = TransformStamped()
            tf.header.stamp = now_msg
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_footprint'
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._tf.sendTransform(tf)

            od = Odometry()
            od.header.stamp = now_msg
            od.header.frame_id = 'odom'
            od.child_frame_id = 'base_footprint'
            od.pose.pose.position.x = self.x
            od.pose.pose.position.y = self.y
            od.pose.pose.orientation.z = qz
            od.pose.pose.orientation.w = qw
            od.twist.twist.linear.x = self.v
            od.twist.twist.angular.z = self.w
            self._pub_odom.publish(od)
            self._pub_filt.publish(od)

            js = JointState()
            js.header.stamp = now_msg
            js.name = ['left_wheel_joint', 'right_wheel_joint']
            js.position = [self.wl, self.wr]
            js.velocity = [vl, vr]
            self._pub_js.publish(js)

        def _publish_scan(self) -> None:
            now_msg = self.get_clock().now().to_msg()
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            origin = np.array([
                self.x + c * LIDAR_XY[0] - s * LIDAR_XY[1],
                self.y + s * LIDAR_XY[0] + c * LIDAR_XY[1],
            ])
            circles = [(*self.obs_xy, self.obs_r)] if self.obs_active else []
            ranges = raycast_scan(origin, self.scan_angles + self.yaw + LIDAR_YAW, self.segs, circles)
            ranges = np.where(ranges >= LIDAR_MAX_RANGE - 0.01, float('inf'), ranges)

            scan = LaserScan()
            scan.header.stamp = now_msg
            scan.header.frame_id = 'laser_link'
            scan.angle_min = -math.pi
            scan.angle_max = math.pi - self.inc
            scan.angle_increment = self.inc
            scan.scan_time = 0.1
            scan.time_increment = 0.1 / LIDAR_RAYS
            scan.range_min = 0.05
            scan.range_max = LIDAR_MAX_RANGE
            scan.ranges = ranges.astype(np.float32).tolist()
            self._pub_scan.publish(scan)

    rclpy.init(args=args)
    node = Nav2SimRobot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
