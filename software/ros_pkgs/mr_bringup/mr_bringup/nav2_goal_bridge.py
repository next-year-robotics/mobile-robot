"""음성·대시보드 `/nav/command` JSON 명령을 Nav2 `NavigateToPose` 액션으로 연결하는 브리지.

부스 대시보드와 음성 비서(mobile-robot-booth `tasks.py`)가 보내는 `/nav/command` 포맷을
그대로 받아 Nav2 A* 전역 경로 생성 + RPP 추종으로 실행하고, `/nav/status` JSON으로
진행 상태를 회신한다.

지원 명령:
    {"cmd": "goto_pose", "id": "t1", "x": 1.98, "y": 1.25, "yaw": 0.0}
    {"cmd": "mark_home", "slot": "home"}
    {"cmd": "goto_home", "id": "t2", "slot": "home"}
    {"cmd": "cancel"}
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

TERMINAL = ('idle', 'arrived', 'failed', 'cancelled')


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass
class BridgeState:
    """ROS 비의존 상태 관리(단위 테스트용)."""

    global_frame: str = 'map'
    pose: tuple[float, float, float] | None = None
    home: tuple[float, float, float] | None = None
    marks: dict[str, tuple[float, float, float]] = field(default_factory=dict)
    state: str = 'idle'
    detail: str = ''
    goal_id: str = ''
    goal_pose: tuple[float, float, float] | None = None
    distance_remaining: float | None = None
    path_length_m: float | None = None

    def mark_home(self, slot: str = 'home') -> bool:
        if self.pose is None:
            return False
        if slot == 'home':
            self.home = self.pose
        else:
            self.marks[slot] = self.pose
        return True

    def resolve_home(self, slot: str = 'home') -> tuple[float, float, float] | None:
        return self.home if slot == 'home' else self.marks.get(slot, self.home)

    def start_goal(self, goal_id: str, x: float, y: float, yaw: float | None) -> tuple[float, float, float]:
        resolved_yaw = float(yaw) if yaw is not None else (self.pose[2] if self.pose is not None else 0.0)
        self.goal_id = goal_id
        self.goal_pose = (float(x), float(y), resolved_yaw)
        self.state = 'driving'
        self.detail = 'nav2'
        self.distance_remaining = None
        return self.goal_pose

    def finish(self, state: str, detail: str = '') -> None:
        self.state = state
        self.detail = detail
        if state in ('arrived', 'cancelled', 'failed'):
            self.distance_remaining = 0.0 if state == 'arrived' else self.distance_remaining

    def status_dict(self) -> dict:
        out: dict = {
            'state': self.state,
            'detail': self.detail,
            'id': self.goal_id,
            'kind': 'pose' if self.goal_pose is not None else '',
            'backend': 'nav2',
            'frame_id': self.global_frame,
            'odom': self.pose is not None,
            'home': list(self.home) if self.home is not None else None,
            'blocked': False,
            'lidar': True,
            'guard': {'enabled': True, 'hold': False, 'near': None},
        }
        if self.goal_id:
            out['goal_id'] = self.goal_id
        if self.goal_pose is not None:
            out['goal'] = [round(self.goal_pose[0], 3), round(self.goal_pose[1], 3)]
            out['goal_pose'] = {
                'kind': 'pose',
                'id': self.goal_id,
                'x': round(self.goal_pose[0], 3),
                'y': round(self.goal_pose[1], 3),
                'yaw': round(self.goal_pose[2], 3),
            }
        if self.distance_remaining is not None:
            out['distance_remaining_m'] = round(self.distance_remaining, 3)
            out['dist_m'] = round(self.distance_remaining, 2)
        if self.path_length_m is not None:
            out['path_length_m'] = round(self.path_length_m, 3)
        return out


def compute_path_length(poses) -> float:
    total = 0.0
    for i in range(1, len(poses)):
        a = poses[i - 1].pose.position
        b = poses[i].pose.position
        total += math.hypot(b.x - a.x, b.y - a.y)
    return total


def main(args=None) -> None:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry, Path
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    import tf2_ros

    reliable = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
    )

    class Nav2GoalBridge(Node):

        def __init__(self) -> None:
            super().__init__('nav2_goal_bridge')
            self.declare_parameter('global_frame', 'map')
            self.declare_parameter('base_frame', 'base_footprint')
            self.declare_parameter('action_name', 'navigate_to_pose')
            self.global_frame = str(self.get_parameter('global_frame').value)
            self.base_frame = str(self.get_parameter('base_frame').value)
            action_name = str(self.get_parameter('action_name').value)

            self.bridge = BridgeState(global_frame=self.global_frame)
            self._goal_handle = None
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
            self._client = ActionClient(self, NavigateToPose, action_name)

            self._status_pub = self.create_publisher(String, '/nav/status', reliable)
            self.create_subscription(String, '/nav/command', self._on_command, reliable)
            self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)
            self.create_subscription(Path, '/plan', self._on_plan, 10)
            self.create_timer(0.2, self._publish_status)
            self.get_logger().info(
                f'nav2_goal_bridge 준비 완료 (action={action_name}, frame={self.global_frame})'
            )

        def _current_pose_in_global(self) -> tuple[float, float, float] | None:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self.global_frame, self.base_frame, rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
                t = tf.transform.translation
                q = tf.transform.rotation
                return float(t.x), float(t.y), quaternion_to_yaw(q.x, q.y, q.z, q.w)
            except Exception:
                return self.bridge.pose

        def _on_odom(self, msg: Odometry) -> None:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            tf_pose = None
            try:
                tf = self._tf_buffer.lookup_transform(
                    self.global_frame, self.base_frame, rclpy.time.Time()
                )
                t = tf.transform.translation
                r = tf.transform.rotation
                tf_pose = (float(t.x), float(t.y), quaternion_to_yaw(r.x, r.y, r.z, r.w))
            except Exception:
                tf_pose = (float(p.x), float(p.y), quaternion_to_yaw(q.x, q.y, q.z, q.w))
            self.bridge.pose = tf_pose

        def _on_plan(self, msg: Path) -> None:
            if msg.poses:
                self.bridge.path_length_m = compute_path_length(msg.poses)

        def _on_command(self, msg: String) -> None:
            try:
                data = json.loads(msg.data)
                cmd = str(data['cmd'])
                goal_id = str(data.get('id', ''))
            except (ValueError, KeyError, TypeError):
                return

            if cmd == 'cancel':
                self._cancel_active()
                return
            if cmd == 'mark_home':
                self.bridge.pose = self._current_pose_in_global()
                slot = str(data.get('slot', 'home'))
                ok = self.bridge.mark_home(slot)
                self.get_logger().info(
                    f'출발 자세 기억({slot}): {self.bridge.pose}' if ok else '출발 자세 기억 실패(pose 없음)'
                )
                self._publish_status()
                return
            if cmd == 'goto_home':
                slot = str(data.get('slot', 'home'))
                target = self.bridge.resolve_home(slot)
                if target is None:
                    self.bridge.goal_id = goal_id
                    self.bridge.finish('failed', 'no_home')
                    self._publish_status()
                    return
                self._send_nav2_goal(goal_id, target[0], target[1], target[2], data.get('frame_id'))
                return
            if cmd == 'goto_pose':
                yaw = data.get('yaw')
                self._send_nav2_goal(
                    goal_id,
                    float(data['x']),
                    float(data['y']),
                    None if yaw is None else float(yaw),
                    data.get('frame_id'),
                )
                return

        def _send_nav2_goal(
            self,
            goal_id: str,
            x: float,
            y: float,
            yaw: float | None,
            frame_id: str | None = None,
        ) -> None:
            if not self._client.wait_for_server(timeout_sec=1.0):
                self.bridge.goal_id = goal_id
                self.bridge.finish('failed', 'nav2_unavailable')
                self.get_logger().error('Nav2 navigate_to_pose 액션 서버가 응답하지 않는다.')
                self._publish_status()
                return

            if self._goal_handle is not None:
                try:
                    self._goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self._goal_handle = None

            gx, gy, gyaw = self.bridge.start_goal(goal_id, x, y, yaw)
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = str(frame_id or self.global_frame)
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = gx
            goal.pose.pose.position.y = gy
            qx, qy, qz, qw = yaw_to_quaternion(gyaw)
            goal.pose.pose.orientation.x = qx
            goal.pose.pose.orientation.y = qy
            goal.pose.pose.orientation.z = qz
            goal.pose.pose.orientation.w = qw

            self.get_logger().info(
                f'Nav2 목표 전송({goal_id}): ({gx:.2f}, {gy:.2f}, yaw={math.degrees(gyaw):.0f}°) '
                f'in {goal.pose.header.frame_id}'
            )
            fut = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
            fut.add_done_callback(lambda f: self._on_goal_response(f, goal_id))
            self._publish_status()

        def _on_goal_response(self, future, goal_id: str) -> None:
            handle = future.result()
            if handle is None or not handle.accepted:
                if self.bridge.goal_id == goal_id:
                    self.bridge.finish('failed', 'rejected')
                    self._publish_status()
                return
            self._goal_handle = handle
            res_fut = handle.get_result_async()
            res_fut.add_done_callback(lambda f: self._on_result(f, goal_id))

        def _on_feedback(self, feedback_msg) -> None:
            self.bridge.distance_remaining = float(feedback_msg.feedback.distance_remaining)

        def _on_result(self, future, goal_id: str) -> None:
            if self.bridge.goal_id != goal_id:
                return
            res = future.result()
            self._goal_handle = None
            if res is None:
                self.bridge.finish('failed', 'no_result')
            elif res.status == GoalStatus.STATUS_SUCCEEDED:
                self.bridge.finish('arrived', 'nav2_succeeded')
            elif res.status == GoalStatus.STATUS_CANCELED:
                self.bridge.finish('cancelled', 'nav2_canceled')
            else:
                self.bridge.finish('failed', f'nav2_status_{res.status}')
            self.get_logger().info(f'Nav2 완료({goal_id}): {self.bridge.state} ({self.bridge.detail})')
            self._publish_status()

        def _cancel_active(self) -> None:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None
            self.bridge.finish('cancelled', 'cancel')
            self._publish_status()

        def _publish_status(self) -> None:
            self._status_pub.publish(
                String(data=json.dumps(self.bridge.status_dict(), separators=(',', ':')))
            )

    rclpy.init(args=args)
    node = Nav2GoalBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
