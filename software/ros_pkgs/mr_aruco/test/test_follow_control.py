"""FollowController 제어법 시험. ROS 없이 돈다."""
import math

from mr_aruco.follow_control import FollowConfig, FollowController
from mr_aruco.tracking import PlanarPose
import pytest

DT = 0.05          # 20 Hz


def cfg(**kw):
    return FollowConfig(**kw)


def run(ctrl, pose, steps=60, dt=DT, t0=0.0, stamp=None):
    """정지 상태에서 여러 주기를 돌려 정상 상태 지령을 얻는다.

    슬루 제한 때문에 한 걸음만으로는 목표 지령에 도달하지 않는다.
    """
    cmd = None
    for i in range(steps):
        now = t0 + i * dt
        cmd = ctrl.step(pose, now if stamp is None else stamp, now)
    return cmd


# ------------------------------------------------------------------ 정지 조건

def test_first_step_is_zero():
    """DT = 0이라 슬루가 직전 값(0)에 묶는다. 정지에서 튀어나가지 않는다."""
    c = FollowController(cfg())
    cmd = c.step(PlanarPose(3.0, 0.0, 0.0), 0.0, 0.0)
    assert cmd.v == 0.0 and cmd.w == 0.0


def test_no_sample_stops():
    c = FollowController(cfg())
    cmd = c.step(None, None, 0.0)
    assert cmd.v == 0.0 and cmd.w == 0.0 and cmd.stale


def test_stale_sample_stops():
    c = FollowController(cfg(marker_timeout_s=0.4))
    run(c, PlanarPose(3.0, 0.0, 0.0))                 # 먼저 달리게 해 둔다
    cmd = c.step(PlanarPose(3.0, 0.0, 0.0), stamp_s=0.0, now=1.0)
    assert cmd.stale


def test_stale_ramps_down_not_cliff():
    """표본이 끊겨도 슬루를 거쳐 내려간다."""
    c = FollowController(cfg())
    run(c, PlanarPose(3.0, 0.0, 0.0))
    moving = c.prev_v
    assert moving > 0.0
    cmd = c.step(None, None, 100.0)
    assert 0.0 <= cmd.v < moving


# ------------------------------------------------------------------ 거리 제어

def test_drives_forward_when_far():
    c = FollowController(cfg(target_range_m=1.1))
    cmd = run(c, PlanarPose(3.0, 0.0, 0.0))
    assert cmd.v > 0.0 and cmd.w == 0.0
    assert cmd.range_m == pytest.approx(3.0)
    assert cmd.range_err_m == pytest.approx(1.9)


def test_never_reverses_when_too_close():
    """후진 금지 — 사람 쪽으로 다가온 뒤 물러서지 않는다."""
    c = FollowController(cfg(target_range_m=1.1))
    cmd = run(c, PlanarPose(0.3, 0.0, 0.0))
    assert cmd.v == 0.0
    assert cmd.range_err_m < 0.0                       # 실제로 너무 가까운 상황


def test_range_deadband_holds_still():
    c = FollowController(cfg(target_range_m=1.1, range_deadband_m=0.12))
    cmd = run(c, PlanarPose(1.16, 0.0, 0.0))
    assert cmd.v == 0.0


def test_linear_speed_is_capped():
    c = FollowController(cfg(max_linear_mps=0.25, k_range=0.6))
    cmd = run(c, PlanarPose(50.0, 0.0, 0.0), steps=200)
    assert cmd.v == pytest.approx(0.25)


def test_linear_slew_is_limited():
    c = FollowController(cfg(max_linear_accel=0.4))
    p = PlanarPose(50.0, 0.0, 0.0)
    c.step(p, 0.0, 0.0)
    cmd = c.step(p, DT, DT)
    assert cmd.v == pytest.approx(0.4 * DT, abs=1e-9)


# ------------------------------------------------------------------ 방향 제어

def test_turns_toward_marker_on_the_left():
    c = FollowController(cfg())
    cmd = run(c, PlanarPose(2.0, 1.0, 0.0))
    assert cmd.w > 0.0                                 # + = 반시계 = 좌회전


def test_turns_right_for_marker_on_the_right():
    c = FollowController(cfg())
    cmd = run(c, PlanarPose(2.0, -1.0, 0.0))
    assert cmd.w < 0.0


def test_bearing_deadband_stops_rotation():
    c = FollowController(cfg(bearing_deadband_rad=0.07))
    cmd = run(c, PlanarPose(3.0, 0.05, 0.0))           # α ≈ 0.017 rad
    assert cmd.w == 0.0


def test_angular_speed_is_capped():
    c = FollowController(cfg(max_angular_rps=0.7, k_bearing=1.2))
    cmd = run(c, PlanarPose(0.0, 3.0, 0.0), steps=200)  # α = +90도
    assert cmd.w == pytest.approx(0.7)


# ------------------------------------------------- 큰 각도에서는 먼저 회전한다

def test_turns_in_place_beyond_align_first():
    c = FollowController(cfg(align_first_rad=0.44))
    cmd = run(c, PlanarPose(1.0, 3.0, 0.0))            # α ≈ 1.25 rad
    assert cmd.turning_in_place
    assert cmd.v == 0.0 and cmd.w > 0.0


def test_drives_and_turns_below_align_first():
    c = FollowController(cfg(align_first_rad=0.44))
    cmd = run(c, PlanarPose(3.0, 0.6, 0.0))            # α ≈ 0.20 rad
    assert not cmd.turning_in_place
    assert cmd.v > 0.0 and cmd.w > 0.0


def test_cos_gate_slows_forward_when_off_axis():
    """같은 거리라도 옆으로 벗어나면 전진이 느려야 한다.

    속도 상한에 걸리면 게이트 효과가 가려지므로, 포화되지 않는 거리를 고른다.
    rho = 1.4에서 k_range * e_rho = 0.18 m/s로 상한 0.25 아래다.
    """
    rho, alpha = 1.4, 0.4
    straight = run(FollowController(cfg(align_first_rad=10.0)),
                   PlanarPose(rho, 0.0, 0.0))
    off = run(FollowController(cfg(align_first_rad=10.0)),
              PlanarPose(rho * math.cos(alpha), rho * math.sin(alpha), 0.0))
    assert straight.v < 0.25, '시험점이 포화되면 이 시험은 의미가 없다'
    assert off.v == pytest.approx(straight.v * math.cos(alpha), rel=1e-6)


def test_cos_gate_never_reverses_for_marker_behind():
    """마커가 뒤에 있으면 cos α < 0이지만 v가 음수로 뒤집히면 안 된다."""
    c = FollowController(cfg(align_first_rad=10.0))    # 회전 게이트를 무력화
    cmd = run(c, PlanarPose(-3.0, 0.1, 0.0))
    assert cmd.v >= 0.0


# ------------------------------------------------------------------ 출력 정리

def test_output_deadband_zeroes_tiny_commands():
    c = FollowController(cfg(target_range_m=1.1, range_deadband_m=0.0,
                             min_linear_mps=0.005, k_range=0.6))
    cmd = run(c, PlanarPose(1.104, 0.0, 0.0))          # k*e = 0.0024 m/s
    assert cmd.v == 0.0


def test_reset_clears_slew_state():
    c = FollowController(cfg())
    run(c, PlanarPose(50.0, 0.0, 0.0))
    assert c.prev_v > 0.0
    c.reset()
    assert c.prev_v == 0.0 and c.prev_w == 0.0
    assert c._prev_time is None and c._prev_bearing is None
