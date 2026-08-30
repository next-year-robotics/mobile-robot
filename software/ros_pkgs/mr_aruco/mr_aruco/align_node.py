"""`AlignToMarker` 액션 서버 → `/cmd_vel_align`. 고정 마커 앞 정해진 자세로 선다.

제어법은 `align_control.py`에 있고 여기는 배선과 **정책**을 맡는다. 제어기는
발진을 "표시"만 하고 중단은 여기서 정한다 — 흔들리면서도 수렴 중일 수 있어서
중단 여부는 제어법이 아니라 운용 판단이다.

한 번에 goal 하나만 받는다. 두 개가 동시에 `/cmd_vel_align`을 몰면 서로의 슬루
상태를 망가뜨린다.

    ros2 run mr_aruco align_node --ros-args --params-file config/align.yaml
    ros2 action send_goal /align_to_marker mr_aruco_msgs/action/AlignToMarker \
        "{marker_id: 10}" --feedback
"""
from __future__ import annotations

import threading

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import TwistStamped
from mr_aruco.align_control import (
    AlignConfig, AlignController, AlignGoal,
    FAILURE_MARKER_LOST, FAILURE_OSCILLATION, FAILURE_TIMEOUT,
    PHASE_DISTANCE, PHASE_FINAL, PHASE_YAW,
)
from mr_aruco.tracking import PlanarPose
from mr_aruco_msgs.action import AlignToMarker
from mr_aruco_msgs.msg import MarkerTrack
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

PHASE_TO_MSG = {
    PHASE_YAW: AlignToMarker.Feedback.PHASE_YAW,
    PHASE_DISTANCE: AlignToMarker.Feedback.PHASE_DISTANCE,
    PHASE_FINAL: AlignToMarker.Feedback.PHASE_FINAL,
}

FAILURE_TO_CODE = {
    FAILURE_MARKER_LOST: AlignToMarker.Result.MARKER_LOST,
    FAILURE_TIMEOUT: AlignToMarker.Result.TIMEOUT,
    FAILURE_OSCILLATION: AlignToMarker.Result.OSCILLATION,
}


class AlignNode(Node):

    def __init__(self) -> None:
        super().__init__('align')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('cmd_frame_id', 'base_footprint')
        self.declare_parameter('abort_on_oscillation', True)
        self.declare_parameter('standoff_m', 0.45)
        self.declare_parameter('k_bearing', 1.4)
        self.declare_parameter('k_dist', 0.8)
        self.declare_parameter('k_final_yaw', 0.6)
        self.declare_parameter('max_linear_mps', 0.15)
        self.declare_parameter('max_angular_rps', 0.7)
        self.declare_parameter('max_linear_accel', 0.5)
        self.declare_parameter('max_angular_accel', 2.5)
        self.declare_parameter('min_linear_mps', 0.004)
        self.declare_parameter('min_angular_rps', 0.008)
        self.declare_parameter('yaw_enter_tol_rad', 0.06)
        self.declare_parameter('yaw_exit_tol_rad', 0.20)
        self.declare_parameter('dist_enter_tol_m', 0.03)
        self.declare_parameter('stale_timeout_s', 0.4)
        self.declare_parameter('loss_timeout_s', 3.0)
        self.declare_parameter('oscillation_window', 30)
        self.declare_parameter('oscillation_flips', 8)
        # goal이 0을 주면 쓰는 기본 허용오차.
        self.declare_parameter('default_dist_tol_m', 0.04)
        self.declare_parameter('default_yaw_tol_rad', 0.14)
        self.declare_parameter('default_stable_time_s', 1.0)
        self.declare_parameter('default_timeout_s', 60.0)

        self.controller = AlignController(AlignConfig(
            standoff_m=self._f('standoff_m'),
            k_bearing=self._f('k_bearing'),
            k_dist=self._f('k_dist'),
            k_final_yaw=self._f('k_final_yaw'),
            max_linear_mps=self._f('max_linear_mps'),
            max_angular_rps=self._f('max_angular_rps'),
            max_linear_accel=self._f('max_linear_accel'),
            max_angular_accel=self._f('max_angular_accel'),
            min_linear_mps=self._f('min_linear_mps'),
            min_angular_rps=self._f('min_angular_rps'),
            yaw_enter_tol_rad=self._f('yaw_enter_tol_rad'),
            yaw_exit_tol_rad=self._f('yaw_exit_tol_rad'),
            dist_enter_tol_m=self._f('dist_enter_tol_m'),
            stale_timeout_s=self._f('stale_timeout_s'),
            loss_timeout_s=self._f('loss_timeout_s'),
            oscillation_window=int(self.get_parameter('oscillation_window').value),
            oscillation_flips=int(self.get_parameter('oscillation_flips').value),
        ))

        self.cmd_frame_id = self.get_parameter('cmd_frame_id').value
        self.abort_on_oscillation = bool(
            self.get_parameter('abort_on_oscillation').value)
        self.rate_hz = self._f('control_rate_hz')

        # twist_mux 입력은 RELIABLE이어야 한다 (base_contract.md 참조).
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel_align', 10)
        self._last_was_zero = False   # finally의 중복 0 발행을 막는다

        # 제어 루프가 rate.sleep()으로 자는 동안에도 구독 콜백이 돌아야 하므로
        # 재진입 그룹 + 다중 스레드 실행기를 쓴다. 기본 그룹이면 여기서 교착된다.
        self._group = ReentrantCallbackGroup()
        self.create_subscription(MarkerTrack, 'marker_track', self.on_track, 10,
                                 callback_group=self._group)
        self._server = ActionServer(
            self, AlignToMarker, 'align_to_marker',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self._group)

        self._track = None
        self._busy = threading.Lock()

        self.get_logger().info(
            f'align: {self.rate_hz:.0f} Hz, standoff {self.controller.config.standoff_m:.2f} m, '
            f'발진 시 중단 {"함" if self.abort_on_oscillation else "안 함"}')

    # ---------------------------------------------------------------- 콜백

    def on_track(self, msg: MarkerTrack) -> None:
        self._track = msg

    def on_goal(self, goal_request) -> GoalResponse:
        """받을 수 있는 goal인지 먼저 본다. 못 할 일은 시작하지 않는다."""
        track = self._track
        if track is None:
            self.get_logger().warn('goal 거절: /marker_track이 아직 안 온다.')
            return GoalResponse.REJECT
        if track.marker_id != goal_request.marker_id:
            # marker_track은 id 하나만 걸러 낸다. 다른 id는 애초에 볼 수 없다.
            self.get_logger().warn(
                f'goal 거절: marker_track은 id {track.marker_id}를 보는데 '
                f'goal은 {goal_request.marker_id}를 요구한다. '
                'marker_track의 target_marker_id를 바꿔야 한다.')
            return GoalResponse.REJECT
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn('goal 거절: 이미 정렬 중이다.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        """정렬 루프. 어떤 경로로 끝나든 마지막에 0 속도를 쏜다."""
        request = goal_handle.request
        goal = AlignGoal(
            standoff_m=request.standoff_m,
            dist_tol_m=request.goal_dist_tol_m or self._f('default_dist_tol_m'),
            yaw_tol_rad=request.goal_yaw_tol_rad or self._f('default_yaw_tol_rad'),
            stable_time_s=request.stable_time_sec or self._f('default_stable_time_s'),
            timeout_s=request.timeout_sec or self._f('default_timeout_s'),
        )
        started = self._now()
        # goal마다 초기화한다. 이전 goal의 마지막 0이 남아 있으면 이번 goal이
        # 발행 전에 취소될 때 finally가 0을 건너뛴다.
        self._last_was_zero = False
        self.controller.begin(started, goal)
        self.get_logger().info(
            f'정렬 시작: id={request.marker_id} standoff={self.controller.standoff_m:.2f} m '
            f'허용오차 {goal.dist_tol_m:.3f} m / {goal.yaw_tol_rad:.3f} rad')

        rate = self.create_rate(self.rate_hz)
        try:
            return self._loop(goal_handle, started, rate)
        finally:
            # 모든 종료 경로가 여기를 지난다. 다만 종료 tick의 지령이 이미
            # 정확히 0이었으면 다시 쏘지 않는다 — 계약은 "0 한 번"이고,
            # 중복 발행은 종료 경로가 둘이라는 오해를 남긴다.
            if not self._last_was_zero:
                self._publish(0.0, 0.0)
            rate.destroy()
            self._busy.release()

    # ---------------------------------------------------------------- 루프

    def _loop(self, goal_handle, started: float, rate):
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('정렬 취소됨')
                return self._result(False, AlignToMarker.Result.PREEMPTED,
                                    '사용자 취소', float('nan'), float('nan'))

            pose, stamp, yaw_valid = self._sample()
            if pose is not None and not yaw_valid:
                # yaw를 못 믿으면 정렬 자체가 성립하지 않는다. 위치만 맞추고
                # 자세를 못 맞추는 상태로 수렴하는 척하면 안 된다.
                goal_handle.abort()
                self.get_logger().error(
                    '중단: 마커 법선이 거의 수직이라 yaw를 신뢰할 수 없다.')
                return self._result(False, AlignToMarker.Result.YAW_UNRELIABLE,
                                    'yaw_valid=false', float('nan'), float('nan'))

            cmd = self.controller.step(pose, stamp, self._now())
            self._publish(cmd.v, cmd.w)
            self._publish_feedback(goal_handle, cmd, started)

            if cmd.oscillating and self.abort_on_oscillation:
                goal_handle.abort()
                self.get_logger().error('중단: 각속도 지령이 발진한다. 게인을 낮춰라.')
                return self._result(False, AlignToMarker.Result.OSCILLATION,
                                    '발진 감지', cmd.dist_err_m, cmd.yaw_err_rad)

            if cmd.done:
                goal_handle.succeed()
                self.get_logger().info(
                    f'정렬 완료: 거리 오차 {cmd.dist_err_m:+.3f} m '
                    f'자세 오차 {cmd.yaw_err_rad:+.3f} rad')
                return self._result(True, AlignToMarker.Result.NONE, '',
                                    cmd.dist_err_m, cmd.yaw_err_rad)

            if cmd.failure:
                goal_handle.abort()
                self.get_logger().error(f'정렬 실패: {cmd.failure}')
                return self._result(False, FAILURE_TO_CODE[cmd.failure], cmd.failure,
                                    cmd.dist_err_m, cmd.yaw_err_rad)

            rate.sleep()

        # rclpy가 내려갔다. 성공으로 처리하지 않는다.
        goal_handle.abort()
        return self._result(False, AlignToMarker.Result.UNKNOWN, 'shutdown',
                            float('nan'), float('nan'))

    # ---------------------------------------------------------------- 내부

    def _sample(self):
        msg = self._track
        if msg is None or msg.status == MarkerTrack.STATUS_LOST:
            return None, None, False
        pose = PlanarPose(x=msg.x, y=msg.y, yaw=msg.yaw)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return pose, stamp, msg.yaw_valid

    def _publish(self, v: float, w: float) -> None:
        self._last_was_zero = (v == 0.0 and w == 0.0)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cmd_frame_id
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self.cmd_pub.publish(msg)

    def _publish_feedback(self, goal_handle, cmd, started: float) -> None:
        fb = AlignToMarker.Feedback()
        fb.phase = PHASE_TO_MSG[cmd.phase]
        fb.dist_err_m = float(cmd.dist_err_m)
        fb.yaw_err_rad = float(cmd.yaw_err_rad)
        fb.stable_for_sec = float(cmd.stable_for_s)
        fb.oscillating = cmd.oscillating
        elapsed = self._now() - started
        fb.elapsed = Duration(sec=int(elapsed),
                              nanosec=int((elapsed % 1.0) * 1e9))
        goal_handle.publish_feedback(fb)

    def _result(self, success: bool, code: int, message: str,
                dist: float, yaw: float):
        result = AlignToMarker.Result()
        result.success = success
        result.error_code = code
        result.error_msg = message
        result.final_dist_m = float(dist)
        result.final_yaw_rad = float(yaw)
        return result

    def _f(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlignNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
