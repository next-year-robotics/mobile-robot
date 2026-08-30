"""`/aruco_detections` → 대상 마커 하나를 골라 필터링한 `/marker_track`.

`aruco_opencv`는 보이는 마커를 전부, 매 프레임 독립적으로 낸다. 제어기가 그걸
그대로 먹으면 (1) 대상이 아닌 마커에 반응하고 (2) 한 프레임 유실에 정지하며
(3) 오검출 한 번에 튄다. 이 노드가 그 셋을 흡수한다.

시각 처리에 규칙이 하나 있다. **필터의 시계는 노드 시계이지만, 발행하는
`header.stamp`는 마지막으로 마커를 실제로 본 시각이다.** hold 중에는 stamp가
멈춰 있으므로 하류 제어기의 낡음 판정에 자연히 걸린다. 마커 좌표는
`base_footprint` 기준이라 로봇이 움직이면 낡는데, 이 구조가 그걸 자동으로 막는다.

    ros2 run mr_aruco marker_track_node --ros-args --params-file config/marker_track.yaml
"""
from __future__ import annotations

from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import PoseStamped
from mr_aruco.tracking import (
    marker_normal_yaw, MarkerTracker, PlanarPose,
    scale_point_from_origin,
    STATUS_HOLDING, STATUS_LOST, STATUS_TRACKING,
    TrackConfig,
)
from mr_aruco_msgs.msg import MarkerTrack
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

STATUS_TO_MSG = {
    STATUS_LOST: MarkerTrack.STATUS_LOST,
    STATUS_TRACKING: MarkerTrack.STATUS_TRACKING,
    STATUS_HOLDING: MarkerTrack.STATUS_HOLDING,
}

DIAGNOSTIC_PERIOD_S = 10.0


class MarkerTrackNode(Node):

    def __init__(self) -> None:
        super().__init__('marker_track')

        self.declare_parameter('target_marker_id', 0)
        self.declare_parameter('output_frame', 'base_footprint')
        self.declare_parameter('min_stable_s', 0.3)
        self.declare_parameter('max_hold_s', 1.0)
        self.declare_parameter('max_jump_pos_m', 0.10)
        self.declare_parameter('max_jump_yaw_rad', 0.35)
        self.declare_parameter('reject_streak_unlock', 8)
        # 법선의 지면 투영이 이보다 짧으면 yaw를 믿지 않는다. 0.2면 법선이 수직에서
        # 약 12도 이상 기울어야 통과한다 — 벽에 세워 단 마커는 여유 있게 넘는다.
        self.declare_parameter('min_normal_horizontal', 0.2)
        # 검출 메시지 자체가 이만큼 끊기면(카메라나 검출 노드가 죽은 경우) 미검출로
        # 간주한다. 마커가 안 보이는 것과 파이프라인이 죽은 것을 구분하기 위해
        # 별도 감시가 필요하다 — 후자는 콜백이 아예 안 온다.
        self.declare_parameter('detection_gap_timeout_s', 0.3)
        self.declare_parameter('watchdog_period_s', 0.1)
        self.declare_parameter('publish_dock_pose', True)
        # 150 mm 마커의 1.0/2.0 m 실측에서 구한 PnP 병진 보정. 물리
        # marker_size나 카메라–베이스 TF를 거짓으로 바꾸지 않고, 카메라
        # 광학 중심 기준 병진벡터만 스케일한다.
        self.declare_parameter('pnp_translation_scale', 1.0)
        self.declare_parameter('camera_origin_xyz_m', [0.250, 0.000, 0.410])

        self.target_id = int(self.get_parameter('target_marker_id').value)
        self.output_frame = self.get_parameter('output_frame').value
        self.min_normal_horizontal = float(
            self.get_parameter('min_normal_horizontal').value)
        self.gap_timeout_s = float(self.get_parameter('detection_gap_timeout_s').value)
        self.pnp_translation_scale = float(
            self.get_parameter('pnp_translation_scale').value)
        camera_origin = self.get_parameter('camera_origin_xyz_m').value
        if len(camera_origin) != 3:
            raise ValueError('camera_origin_xyz_m must contain exactly three coordinates')
        self.camera_origin = tuple(float(value) for value in camera_origin)
        # 기동 즉시 잘못된 계수를 거부한다. 실제 보정은 콜백에서 같은
        # 함수를 쓰므로 검증과 실행 경로가 갈라지지 않는다.
        scale_point_from_origin(self.camera_origin, self.camera_origin,
                                self.pnp_translation_scale)

        self.tracker = MarkerTracker(TrackConfig(
            min_stable_s=float(self.get_parameter('min_stable_s').value),
            max_hold_s=float(self.get_parameter('max_hold_s').value),
            max_jump_pos_m=float(self.get_parameter('max_jump_pos_m').value),
            max_jump_yaw_rad=float(self.get_parameter('max_jump_yaw_rad').value),
            reject_streak_unlock=int(self.get_parameter('reject_streak_unlock').value),
        ))

        self.track_pub = self.create_publisher(MarkerTrack, 'marker_track', 10)
        self.dock_pub = None
        if bool(self.get_parameter('publish_dock_pose').value):
            # opennav_docking의 SimpleChargingDock이 그대로 구독하는 이름이다.
            # 정렬을 그쪽으로 갈아끼울 때 이 토픽만 있으면 된다.
            self.dock_pub = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)

        self.create_subscription(ArucoDetection, 'aruco_detections', self.on_detection, 10)

        self._last_detection_msg_s = None
        self._observed_stamp = None          # 마지막으로 마커를 실제로 본 시각
        self._yaw_valid = False
        self._last_summary = ''
        self.create_timer(float(self.get_parameter('watchdog_period_s').value),
                          self.on_watchdog)
        self.create_timer(DIAGNOSTIC_PERIOD_S, self.on_diagnostic)

        self.get_logger().info(
            f'marker_track: id={self.target_id} frame={self.output_frame} '
            f'hold={self.tracker.config.max_hold_s:.2f}s '
            f'jump={self.tracker.config.max_jump_pos_m:.3f}m '
            f'pnp_scale={self.pnp_translation_scale:.9f} '
            f'camera_origin={self.camera_origin}')

    # ---------------------------------------------------------------- 콜백

    def on_detection(self, msg: ArucoDetection) -> None:
        now = self._now()
        self._last_detection_msg_s = now

        if msg.header.frame_id != self.output_frame and (msg.markers or msg.boards):
            # aruco_opencv의 output_frame이 안 맞으면 좌표가 카메라 기준으로 온다.
            # 그대로 제어에 쓰면 조용히 엉뚱한 방향으로 간다.
            self.get_logger().error(
                f'검출 프레임이 {msg.header.frame_id}다. {self.output_frame}이어야 한다 — '
                'aruco_opencv의 output_frame 설정을 확인하라.',
                throttle_duration_sec=5.0)
            return

        # aruco_opencv는 검출이 하나도 없으면 변환할 pose가 없어서, output_frame을
        # 설정했어도 이미지의 camera_optical_frame을 헤더에 그대로 둔다. 빈 메시지는
        # 좌표를 소비하지 않으므로 정상적인 "미검출"로 받아 tracker를 LOST 쪽으로
        # 진행시켜야 한다. 좌표가 하나라도 있을 때의 프레임 검사는 위에서 유지한다.

        pose, yaw_valid = self._select(msg)
        if pose is not None:
            if self.dock_pub is not None:
                self._publish_dock_pose(msg)

        self._step_and_publish(
            pose, now,
            observed_stamp=msg.header.stamp if pose is not None else None,
            observed_yaw_valid=yaw_valid,
        )

    def on_watchdog(self) -> None:
        """검출 메시지 자체가 끊겼을 때 상태를 진행시킨다.

        마커가 안 보이면 `aruco_opencv`는 빈 메시지를 계속 보내므로 콜백이 돈다.
        콜백이 아예 안 오는 것은 카메라나 검출 노드가 죽은 경우이고, 그때도
        추적 상태는 LOST로 내려가야 한다.
        """
        now = self._now()
        if self._last_detection_msg_s is None:
            return
        if now - self._last_detection_msg_s <= self.gap_timeout_s:
            return
        self.get_logger().warn(
            f'{now - self._last_detection_msg_s:.1f}초째 검출 메시지가 없다. '
            '카메라나 aruco_tracker를 확인하라.',
            throttle_duration_sec=5.0)
        self._step_and_publish(None, now)

    def on_diagnostic(self) -> None:
        summary = self.tracker.counters.summary()
        if summary != self._last_summary:
            self.get_logger().info(summary)
            self._last_summary = summary

    # ---------------------------------------------------------------- 선택

    def _select(self, msg: ArucoDetection):
        """대상 id의 마커를 골라 ``(pose, yaw_valid)``로 바꾼다."""
        for marker in msg.markers:
            if marker.marker_id != self.target_id:
                continue
            p = marker.pose.position
            corrected = self._correct_position(p.x, p.y, p.z)
            q = marker.pose.orientation
            yaw, horizontal = marker_normal_yaw(q.x, q.y, q.z, q.w)
            yaw_valid = horizontal >= self.min_normal_horizontal
            if not yaw_valid:
                self.get_logger().warn(
                    f'마커 {self.target_id}의 법선이 거의 수직이다(수평 성분 '
                    f'{horizontal:.3f}). yaw를 신뢰하지 않는다 — 마커가 눕혀 달렸는가?',
                    throttle_duration_sec=5.0)
            return PlanarPose(x=corrected[0], y=corrected[1],
                              yaw=float(yaw)), yaw_valid
        return None, None

    # ---------------------------------------------------------------- 발행

    def _step_and_publish(self, pose, now: float, observed_stamp=None,
                          observed_yaw_valid=None) -> None:
        out = self.tracker.update(pose, now)

        # 점프로 기각한 표본은 "새 관측"이 아니다. 유지한 예전 자세에
        # 기각 표본의 stamp를 붙이면 하류 제어기에게 오래된 좌표가 계속
        # 싱싱한 것처럼 보인다. yaw_valid도 유지 자세의 것을 지켜야 한다.
        if pose is not None and not out.rejected_jump:
            self._observed_stamp = observed_stamp
            self._yaw_valid = bool(observed_yaw_valid)

        msg = MarkerTrack()
        # stamp는 노드 시계가 아니라 "마지막으로 실제로 본 시각"이다. hold 중에는
        # 멈춰 있어야 하류가 낡음을 판정할 수 있다.
        if self._observed_stamp is not None:
            msg.header.stamp = self._observed_stamp
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame
        msg.status = STATUS_TO_MSG[out.status]
        msg.locked = out.locked
        msg.marker_id = self.target_id
        msg.hold_duration = out.hold_duration_s
        msg.rejected_jump = out.rejected_jump
        if out.pose is not None:
            msg.x = out.pose.x
            msg.y = out.pose.y
            msg.yaw = out.pose.yaw
            msg.yaw_valid = self._yaw_valid
        self.track_pub.publish(msg)

        if out.status == STATUS_LOST:
            self._observed_stamp = None
            self._yaw_valid = False

    def _publish_dock_pose(self, msg: ArucoDetection) -> None:
        """싱싱한 관측만 내보낸다.

        hold로 유지한 값은 base_footprint 기준이라 로봇이 움직이면 틀려진다.
        외부 도킹 서버는 이 토픽을 "지금 본 것"으로 취급하므로 유지값을 주면 안 된다.
        """
        for marker in msg.markers:
            if marker.marker_id != self.target_id:
                continue
            out = PoseStamped()
            out.header = msg.header
            p = marker.pose.position
            corrected = self._correct_position(p.x, p.y, p.z)
            out.pose.position.x = corrected[0]
            out.pose.position.y = corrected[1]
            out.pose.position.z = corrected[2]
            out.pose.orientation = marker.pose.orientation
            self.dock_pub.publish(out)
            return

    def _correct_position(self, x: float, y: float, z: float):
        return scale_point_from_origin(
            (float(x), float(y), float(z)),
            self.camera_origin,
            self.pnp_translation_scale,
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MarkerTrackNode()
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
