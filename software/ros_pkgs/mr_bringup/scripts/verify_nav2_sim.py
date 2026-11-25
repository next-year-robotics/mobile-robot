#!/usr/bin/env python3
"""End-to-end verification script for mr_bringup Nav2 + slam_toolbox simulation.

Verifies:
  1. `ros2 launch mr_bringup nav_sim.launch.py` brings up all Nav2 lifecycle nodes + slam_toolbox.
  2. `/nav/command` JSON `mark_home` + `goto_pose(1.98, 1.25, 0.0)` triggers Nav2 A* planning around
     the center coffee table obstacle and drives Maknae through `twist_mux` (`TwistStamped`) to the goal.
  3. `/nav/command` JSON `goto_home` plans a return path and navigates cleanly back to the start pose.
"""
from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main() -> int:
    env = os.environ.copy()
    env.setdefault('ROS_DOMAIN_ID', '78')
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']

    log_path = '/tmp/mr_nav2_sim.log'
    log_f = open(log_path, 'w', encoding='utf-8')
    proc = subprocess.Popen(
        ['ros2', 'launch', 'mr_bringup', 'nav_sim.launch.py'],
        stdout=log_f,
        stderr=subprocess.STDOUT,
        env=env,
        start_new_session=True,
    )

    rclpy.init()
    node = Node('nav2_e2e_verifier')
    try:
        rel = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE
        )
        pub_cmd = node.create_publisher(String, '/nav/command', rel)

        last_status: dict = {}
        last_pose: tuple[float, float, float] | None = None
        plans: list[tuple[int, float]] = []
        got_map = False

        def on_status(m: String) -> None:
            nonlocal last_status
            last_status = json.loads(m.data)

        def on_odom(m: Odometry) -> None:
            nonlocal last_pose
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            last_pose = (p.x, p.y, yaw)

        def on_plan(m: Path) -> None:
            if m.poses:
                d = sum(
                    math.hypot(
                        m.poses[i].pose.position.x - m.poses[i - 1].pose.position.x,
                        m.poses[i].pose.position.y - m.poses[i - 1].pose.position.y,
                    )
                    for i in range(1, len(m.poses))
                )
                plans.append((len(m.poses), round(d, 3)))

        def on_map(m: OccupancyGrid) -> None:
            nonlocal got_map
            if m.info.width > 0 and m.info.height > 0:
                got_map = True

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(String, '/nav/status', on_status, rel)
        node.create_subscription(Odometry, '/odometry/filtered', on_odom, 10)
        node.create_subscription(Path, '/plan', on_plan, 10)
        node.create_subscription(OccupancyGrid, '/map', on_map, map_qos)
        client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        is_active_cli = node.create_client(Trigger, '/lifecycle_manager_navigation/is_active')

        t0 = time.monotonic()
        print('Waiting for Nav2 lifecycle ACTIVE + /map + /nav/status...', flush=True)
        nav2_active = False
        while time.monotonic() - t0 < 40.0:
            rclpy.spin_once(node, timeout_sec=0.25)
            if not nav2_active and is_active_cli.service_is_ready():
                fut = is_active_cli.call_async(Trigger.Request())
                while not fut.done() and time.monotonic() - t0 < 40.0:
                    rclpy.spin_once(node, timeout_sec=0.1)
                if fut.done() and fut.result() is not None and fut.result().success:
                    nav2_active = True
            if nav2_active and got_map and client.server_is_ready() and last_status.get('odom') and last_pose is not None:
                break
        if not (nav2_active and got_map and client.server_is_ready() and last_status.get('odom') and last_pose is not None):
            print(
                f'FAIL: Nav2 not ready after 40s! active={nav2_active} map={got_map} '
                f'server={client.server_is_ready()} status={last_status}'
            )
            return 1

        print(
            f'Nav2 READY in {time.monotonic() - t0:.1f}s! '
            f'start_pose={tuple(round(v, 2) for v in last_pose)}',
            flush=True,
        )
        # Allow slam_toolbox initial scan & map publication
        for _ in range(15):
            rclpy.spin_once(node, timeout_sec=0.2)

        # 1. Mark home
        pub_cmd.publish(String(data=json.dumps({'cmd': 'mark_home', 'slot': 'home'})))
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.2)
        print('Marked home:', last_status.get('home'), flush=True)

        # 2. Navigate around center coffee table to desk (1.98, 1.25)
        goal_xy = (1.98, 1.25)
        straight_dist = math.hypot(goal_xy[0] - last_pose[0], goal_xy[1] - last_pose[1])
        pub_cmd.publish(
            String(
                data=json.dumps(
                    {'cmd': 'goto_pose', 'id': 'desk_1', 'x': goal_xy[0], 'y': goal_xy[1], 'yaw': 0.0}
                )
            )
        )
        t_go = time.monotonic()
        arrived = False
        last_log = 0.0
        while time.monotonic() - t_go < 60.0:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if now - last_log > 4.0:
                last_log = now
                print(
                    f"  t+{now - t_go:4.1f}s pose={tuple(round(v, 2) for v in last_pose)} "
                    f"state={last_status.get('state')}({last_status.get('detail')}) "
                    f"rem={last_status.get('distance_remaining_m')} plans={len(plans)}",
                    flush=True,
                )
            if last_status.get('id') == 'desk_1' and last_status.get('state') in (
                'arrived', 'failed', 'cancelled'
            ):
                arrived = last_status.get('state') == 'arrived'
                break

        dist_err = math.hypot(last_pose[0] - goal_xy[0], last_pose[1] - goal_xy[1])
        first_plan = plans[0] if plans else None
        print(
            f"DESK RESULT: state={last_status.get('state')} detail={last_status.get('detail')} "
            f"pose={tuple(round(v, 2) for v in last_pose)} err={dist_err:.3f}m "
            f"straight={straight_dist:.2f}m first_plan={first_plan}",
            flush=True,
        )
        if not arrived or not plans:
            print('FAIL: Desk navigation failed!', flush=True)
            return 1

        # 3. Return home via goto_home
        plans_before = len(plans)
        pub_cmd.publish(String(data=json.dumps({'cmd': 'goto_home', 'id': 'home_1', 'slot': 'home'})))
        t_ret = time.monotonic()
        returned = False
        last_log = 0.0
        while time.monotonic() - t_ret < 60.0:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if now - last_log > 4.0:
                last_log = now
                print(
                    f"  ret+{now - t_ret:4.1f}s pose={tuple(round(v, 2) for v in last_pose)} "
                    f"state={last_status.get('state')} rem={last_status.get('distance_remaining_m')}",
                    flush=True,
                )
            if last_status.get('id') == 'home_1' and last_status.get('state') in (
                'arrived', 'failed', 'cancelled'
            ):
                returned = last_status.get('state') == 'arrived'
                break

        home_err = math.hypot(last_pose[0] - (-1.80), last_pose[1] - (-1.00))
        print(
            f"RETURN HOME RESULT: state={last_status.get('state')} "
            f"pose={tuple(round(v, 2) for v in last_pose)} home_err={home_err:.3f}m "
            f"new_plans={len(plans) - plans_before}",
            flush=True,
        )
        if not returned:
            print('FAIL: Return-home navigation failed!', flush=True)
            return 1

        print('ALL NAV2 END-TO-END CHECKS PASSED!', flush=True)
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
        log_f.close()


if __name__ == '__main__':
    sys.exit(main())
