"""nav2_params.yaml, twist_mux.yaml, nav2_goal_bridge가 막내 물리 상수·토픽 계약에 맞는지 본다."""
from __future__ import annotations

import ast
import math
from pathlib import Path

import pytest
import yaml

from mr_bringup.nav2_goal_bridge import BridgeState, quaternion_to_yaw, yaw_to_quaternion

ROS_PKGS = Path(__file__).resolve().parents[2]
CONFIG = ROS_PKGS / 'mr_bringup' / 'config'
ROBOT_PARAMS = ROS_PKGS / 'mr_description' / 'config' / 'robot_params.yaml'
BASE_PARAMS = ROS_PKGS / 'mr_base' / 'config' / 'base_params.yaml'


def _load(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding='utf-8'))


@pytest.fixture(scope='module')
def nav2() -> dict:
    return _load(CONFIG / 'nav2_params.yaml')


@pytest.fixture(scope='module')
def mux() -> dict:
    return _load(CONFIG / 'twist_mux.yaml')['twist_mux']['ros__parameters']


def test_nav2_frames_match_ekf_and_base(nav2: dict) -> None:
    base = _load(BASE_PARAMS)['mr_base']['ros__parameters']
    ekf = _load(CONFIG / 'ekf.yaml')['ekf_filter_node']['ros__parameters']
    bf = base['base_frame_id']
    odom = base['odom_frame_id']
    gmap = ekf['map_frame']

    assert nav2['bt_navigator']['ros__parameters']['robot_base_frame'] == bf
    assert nav2['bt_navigator']['ros__parameters']['global_frame'] == gmap
    assert nav2['bt_navigator']['ros__parameters']['odom_topic'] == '/odometry/filtered'
    assert nav2['amcl']['ros__parameters']['base_frame_id'] == bf
    assert nav2['amcl']['ros__parameters']['odom_frame_id'] == odom
    assert nav2['amcl']['ros__parameters']['global_frame_id'] == gmap
    assert nav2['local_costmap']['local_costmap']['ros__parameters']['robot_base_frame'] == bf
    assert nav2['local_costmap']['local_costmap']['ros__parameters']['global_frame'] == odom
    assert nav2['global_costmap']['global_costmap']['ros__parameters']['robot_base_frame'] == bf
    assert nav2['global_costmap']['global_costmap']['ros__parameters']['global_frame'] == gmap
    assert nav2['behavior_server']['ros__parameters']['robot_base_frame'] == bf
    assert nav2['collision_monitor']['ros__parameters']['base_frame_id'] == bf


def test_footprint_covers_measured_chassis(nav2: dict) -> None:
    rp = _load(ROBOT_PARAMS)
    lx, ly, _ = rp['chassis_frame_size_m']
    cx, cy, _ = rp['chassis_frame_centre_from_base_link_m']
    for key in ('local_costmap', 'global_costmap'):
        fp = ast.literal_eval(nav2[key][key]['ros__parameters']['footprint'])
        xs = [p[0] for p in fp]
        ys = [p[1] for p in fp]
        assert min(xs) <= cx - lx / 2 + 1e-6
        assert max(xs) >= cx + lx / 2 - 1e-6
        assert min(ys) <= cy - ly / 2 + 1e-6
        assert max(ys) >= cy + ly / 2 - 1e-6


def test_astar_planner_and_rpp_controller_limits(nav2: dict) -> None:
    planner = nav2['planner_server']['ros__parameters']['GridBased']
    assert planner['plugin'] == 'nav2_navfn_planner::NavfnPlanner'
    assert planner['use_astar'] is True

    ctrl = nav2['controller_server']['ros__parameters']['FollowPath']
    assert ctrl['plugin'] == 'nav2_rotation_shim_controller::RotationShimController'
    assert ctrl['primary_controller'] == (
        'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    )
    assert 0.15 <= ctrl['desired_linear_vel'] <= 0.25
    assert ctrl['allow_reversing'] is False


def test_stamped_cmd_vel_and_twist_mux_priority(nav2: dict, mux: dict) -> None:
    # MCU와 twist_mux는 TwistStamped만 받는다.
    assert mux['use_stamped'] is True
    for node in ('controller_server', 'behavior_server', 'velocity_smoother', 'collision_monitor'):
        assert nav2[node]['ros__parameters']['enable_stamped_cmd_vel'] is True

    # Nav2 최종 출력은 /cmd_vel_nav → twist_mux(prio 55)를 거쳐야 한다.
    cm = nav2['collision_monitor']['ros__parameters']
    assert cm['cmd_vel_out_topic'] == 'cmd_vel_nav'
    topics = mux['topics']
    assert topics['nav']['topic'] == 'cmd_vel_nav'
    assert topics['follow']['priority'] < topics['nav']['priority'] < topics['align']['priority']
    assert topics['align']['priority'] < topics['teleop']['priority'] < mux['locks']['estop']['priority']


def test_goal_bridge_state_and_home_slot() -> None:
    st = BridgeState(global_frame='map')
    assert st.mark_home() is False
    st.pose = (-1.8, -1.0, 0.25)
    assert st.mark_home('home') is True
    assert st.mark_home('trip') is True
    assert st.resolve_home('trip') == (-1.8, -1.0, 0.25)

    gx, gy, gyaw = st.start_goal('g1', 1.98, 1.25, 0.0)
    assert (gx, gy, gyaw) == (1.98, 1.25, 0.0)
    snap = st.status_dict()
    assert snap['state'] == 'driving'
    assert snap['backend'] == 'nav2'
    assert snap['goal_id'] == 'g1'

    st.finish('arrived', 'nav2_succeeded')
    assert st.status_dict()['state'] == 'arrived'

    q = yaw_to_quaternion(math.pi / 3)
    assert math.isclose(quaternion_to_yaw(*q), math.pi / 3, abs_tol=1e-6)
