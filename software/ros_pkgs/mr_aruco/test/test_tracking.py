"""MarkerTracker 상태 전이 시험. ROS 없이 돈다."""
import math

from mr_aruco.tracking import (
    marker_normal_yaw, MarkerTracker, PlanarPose,
    scale_point_from_origin,
    STATUS_HOLDING, STATUS_LOST, STATUS_TRACKING, TrackConfig, wrap_angle,
)
import pytest


def cfg(**kw):
    base = {'min_stable_s': 0.3, 'max_hold_s': 1.0, 'max_jump_pos_m': 0.10,
            'max_jump_yaw_rad': 0.35, 'reject_streak_unlock': 8}
    base.update(kw)
    return TrackConfig(**base)


def lock_on(tracker, t0=0.0, pose=None, step=0.1):
    """min_stable_s를 채워 락을 건다. 마지막 출력을 돌려준다."""
    pose = pose or PlanarPose(1.0, 0.0, 0.0)
    out = None
    t = t0
    while not tracker._locked:
        out = tracker.update(pose, t)
        assert t - t0 < 2.0, '락이 걸리지 않는다'
        if not tracker._locked:
            t += step
    return out, t


# ------------------------------------------------------------------ 기하

def test_range_and_bearing():
    p = PlanarPose(3.0, 4.0, 0.0)
    assert p.range_m == pytest.approx(5.0)
    assert PlanarPose(1.0, 1.0, 0.0).bearing_rad == pytest.approx(math.pi / 4)


def test_bearing_uses_atan2_not_atan():
    """X = 0에서 터지지 않고, 뒤쪽도 앞쪽과 구분돼야 한다."""
    assert PlanarPose(0.0, 1.0, 0.0).bearing_rad == pytest.approx(math.pi / 2)
    # atan(y/x)였다면 앞뒤가 같은 값으로 접힌다.
    assert PlanarPose(-1.0, 1.0, 0.0).bearing_rad != pytest.approx(
        PlanarPose(1.0, 1.0, 0.0).bearing_rad)


def test_pnp_scale_preserves_camera_origin():
    origin = (0.250, 0.0, 0.410)
    assert scale_point_from_origin(origin, origin, 0.960729786) == pytest.approx(origin)


def test_pnp_scale_corrects_camera_relative_vector_not_base_transform():
    scale = 0.960729786
    corrected = scale_point_from_origin(
        (2.331123, 0.004297, 0.604398),
        (0.250, 0.0, 0.410),
        scale,
    )
    assert corrected == pytest.approx((2.249397, 0.004128, 0.596764), abs=1e-6)
    # base x 전체를 곱한 오류 구현은 카메라 오프셋 0.25 m까지 줄여서
    # 이 결과보다 약 9.8 mm 뒤로 간다.
    assert corrected[0] != pytest.approx(2.331123 * scale, abs=1e-3)


@pytest.mark.parametrize('scale', [0.0, -1.0, math.inf, math.nan])
def test_pnp_scale_rejects_invalid_value(scale):
    with pytest.raises(ValueError):
        scale_point_from_origin((1.0, 2.0, 3.0), (0.0, 0.0, 0.0), scale)


def test_wrap_angle():
    assert wrap_angle(0.5) == pytest.approx(0.5)
    assert wrap_angle(2 * math.pi + 0.5) == pytest.approx(0.5)
    assert wrap_angle(-2 * math.pi - 0.5) == pytest.approx(-0.5)
    # ±pi는 같은 각이라 부호는 부동소수 잔차가 정한다. 크기만 본다.
    assert abs(wrap_angle(3 * math.pi)) == pytest.approx(math.pi)


# ------------------------------------------------------------------ 락

def test_no_detection_is_lost():
    t = MarkerTracker(cfg())
    out = t.update(None, 0.0)
    assert out.status == STATUS_LOST and not out.locked and out.pose is None


def test_passes_through_before_lock():
    """락 전에도 검출값은 흘려보낸다 — 출발이 늦으면 안 된다."""
    t = MarkerTracker(cfg())
    out = t.update(PlanarPose(1.0, 0.0, 0.0), 0.0)
    assert out.status == STATUS_TRACKING and not out.locked
    assert out.pose == PlanarPose(1.0, 0.0, 0.0)


def test_locks_after_min_stable():
    t = MarkerTracker(cfg(min_stable_s=0.3))
    p = PlanarPose(1.0, 0.0, 0.0)
    assert not t.update(p, 0.0).locked
    assert not t.update(p, 0.2).locked
    assert t.update(p, 0.3).locked


def test_gap_resets_stability_timer():
    t = MarkerTracker(cfg(min_stable_s=0.3))
    p = PlanarPose(1.0, 0.0, 0.0)
    t.update(p, 0.0)
    t.update(None, 0.1)          # 끊기면 처음부터
    t.update(p, 0.2)
    assert not t.update(p, 0.4).locked
    assert t.update(p, 0.5).locked


# ------------------------------------------------------------------ hold

def test_holding_keeps_last_pose():
    t = MarkerTracker(cfg())
    _, now = lock_on(t)
    out = t.update(None, now + 0.2)
    assert out.status == STATUS_HOLDING and out.locked
    assert out.pose == PlanarPose(1.0, 0.0, 0.0)
    assert out.hold_duration_s == pytest.approx(0.2, abs=1e-6)


def test_hold_expires_to_lost():
    t = MarkerTracker(cfg(max_hold_s=1.0))
    _, now = lock_on(t)
    assert t.update(None, now + 0.9).status == STATUS_HOLDING
    out = t.update(None, now + 1.5)
    assert out.status == STATUS_LOST and not out.locked and out.pose is None
    assert t.counters.losses == 1


def test_hold_then_reacquire():
    t = MarkerTracker(cfg())
    _, now = lock_on(t)
    t.update(None, now + 0.3)
    out = t.update(PlanarPose(1.02, 0.0, 0.0), now + 0.4)
    assert out.status == STATUS_TRACKING and out.locked
    assert out.hold_duration_s == 0.0


# ------------------------------------------------------------------ 점프 기각

def test_position_jump_rejected():
    t = MarkerTracker(cfg(max_jump_pos_m=0.10))
    _, now = lock_on(t)
    out = t.update(PlanarPose(1.5, 0.0, 0.0), now)     # 0.5 m 점프
    assert out.rejected_jump
    assert out.pose == PlanarPose(1.0, 0.0, 0.0)       # 유지값이 나온다
    assert t.counters.rejected_jumps == 1


def test_yaw_jump_rejected():
    t = MarkerTracker(cfg(max_jump_yaw_rad=0.35))
    _, now = lock_on(t)
    out = t.update(PlanarPose(1.0, 0.0, 1.2), now)
    assert out.rejected_jump


def test_small_motion_accepted():
    t = MarkerTracker(cfg(max_jump_pos_m=0.10))
    _, now = lock_on(t)
    out = t.update(PlanarPose(1.05, 0.02, 0.1), now)
    assert not out.rejected_jump
    assert out.pose == PlanarPose(1.05, 0.02, 0.1)


def test_reject_streak_unlocks_and_reacquires():
    """대상이 진짜로 옮겨가면 결국 다시 잡아야 한다 — 기준점에 갇히면 안 된다."""
    t = MarkerTracker(cfg(reject_streak_unlock=3))
    _, now = lock_on(t)
    moved = PlanarPose(2.0, 0.0, 0.0)
    assert t.update(moved, now).rejected_jump
    assert t.update(moved, now + 0.1).rejected_jump
    out = t.update(moved, now + 0.2)                   # 3회째 → 락 해제 후 재취득
    assert not out.locked
    assert out.pose == moved


def test_accepted_sample_clears_streak():
    t = MarkerTracker(cfg(reject_streak_unlock=3))
    _, now = lock_on(t)
    t.update(PlanarPose(2.0, 0.0, 0.0), now)
    t.update(PlanarPose(1.01, 0.0, 0.0), now + 0.1)    # 정상 표본
    assert t.update(PlanarPose(2.0, 0.0, 0.0), now + 0.2).rejected_jump
    assert t._reject_streak == 1


def test_wrapped_yaw_is_not_a_jump():
    """+π 근처에서 부호만 뒤집힌 값을 점프로 오인하면 안 된다."""
    t = MarkerTracker(cfg(max_jump_yaw_rad=0.35))
    _, now = lock_on(t, pose=PlanarPose(1.0, 0.0, math.pi - 0.05))
    out = t.update(PlanarPose(1.0, 0.0, -math.pi + 0.05), now)
    assert not out.rejected_jump


def test_counters_survive_reset():
    t = MarkerTracker(cfg(max_hold_s=0.1))
    _, now = lock_on(t)
    t.update(None, now + 0.5)                          # LOST → 내부 reset
    assert t.counters.detections > 0
    assert 'lost=1' in t.counters.summary()


# ------------------------------------------------------------------ 법선 추출

def test_normal_yaw_for_wall_marker_facing_robot():
    """정면 벽 마커: 법선이 로봇 쪽(-x)을 향하므로 yaw = pi여야 한다."""
    h = math.sqrt(0.5)
    yaw, horiz = marker_normal_yaw(0.0, -h, 0.0, h)     # y축 -90도 회전
    assert abs(yaw) == pytest.approx(math.pi)
    assert horiz == pytest.approx(1.0)


def test_normal_yaw_for_flat_marker_is_unreliable():
    """바닥에 눕힌 마커는 법선이 수직이라 지면 투영이 0이 된다."""
    _, horiz = marker_normal_yaw(0.0, 0.0, 0.0, 1.0)    # 회전 없음
    assert horiz == pytest.approx(0.0, abs=1e-12)


def test_normal_yaw_rotated_wall_marker():
    """벽 마커를 수직축으로 조금 돌리면 그만큼 법선 방향이 따라간다."""
    turn = 0.3
    hy, hw = math.sin(-math.pi / 4), math.cos(-math.pi / 4)
    # z축 turn 회전 * y축 -90도 회전 (쿼터니언 곱)
    cz, sz = math.cos(turn / 2), math.sin(turn / 2)
    qw = cz * hw
    qx = -sz * hy
    qy = cz * hy
    qz = sz * hw
    yaw, horiz = marker_normal_yaw(qx, qy, qz, qw)
    assert horiz == pytest.approx(1.0)
    assert yaw == pytest.approx(math.pi + turn - 2 * math.pi, abs=1e-9)
