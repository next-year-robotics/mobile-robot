"""AlignController 단계 전이·기하·종료 판정 시험. ROS 없이 돈다."""
import math

from mr_aruco.align_control import (
    AlignConfig, AlignController, AlignGoal,
    FAILURE_MARKER_LOST, FAILURE_NONE, FAILURE_OSCILLATION, FAILURE_TIMEOUT,
    PHASE_DISTANCE, PHASE_FINAL, PHASE_YAW,
)
from mr_aruco.tracking import PlanarPose
import pytest

DT = 0.05
STANDOFF = 0.45


def cfg(**kw):
    return AlignConfig(**kw)


def goal(**kw):
    base = {'standoff_m': 0.0, 'dist_tol_m': 0.04, 'yaw_tol_rad': 0.14,
            'stable_time_s': 1.0, 'timeout_s': 60.0}
    base.update(kw)
    return AlignGoal(**base)


def ahead(distance, lateral=0.0, normal_offset=0.0):
    """로봇 정면 `distance`에 있고 로봇을 마주보는 마커.

    마커가 로봇을 마주보면 바깥 법선이 로봇 쪽(-x)을 향하므로 yaw = pi다.
    """
    return PlanarPose(distance, lateral, math.pi + normal_offset)


def drive(ctrl, pose, steps, dt=DT, t0=0.0):
    cmd = None
    for i in range(steps):
        now = t0 + i * dt
        cmd = ctrl.step(pose, now, now)
        if cmd.finished:
            break
    return cmd


# ------------------------------------------------------------------ 기하

def test_geometry_straight_on():
    c = AlignController(cfg(standoff_m=STANDOFF))
    c.begin(0.0, goal())
    g = c._geometry(ahead(1.0))
    assert g.bearing == pytest.approx(0.0)
    assert g.los_m == pytest.approx(1.0)
    assert g.yaw_err == pytest.approx(0.0, abs=1e-9)
    assert g.axial == pytest.approx(1.0)
    assert g.lateral == pytest.approx(0.0, abs=1e-9)
    assert g.dist_err == pytest.approx(1.0 - STANDOFF)


def test_geometry_off_normal():
    """마커가 돌아가 있으면 축방향·횡방향·자세 오차가 모두 생긴다."""
    c = AlignController(cfg(standoff_m=STANDOFF))
    c.begin(0.0, goal())
    g = c._geometry(ahead(1.0, normal_offset=-0.3))
    assert g.yaw_err == pytest.approx(-0.3)
    assert g.axial == pytest.approx(math.cos(0.3))
    assert g.lateral == pytest.approx(math.sin(0.3))
    assert g.pos_err == pytest.approx(
        math.hypot(math.cos(0.3) - STANDOFF, math.sin(0.3)))


def test_goal_standoff_overrides_config():
    c = AlignController(cfg(standoff_m=STANDOFF))
    c.begin(0.0, goal(standoff_m=0.8))
    assert c.standoff_m == 0.8
    c.begin(0.0, goal(standoff_m=0.0))          # 0이면 설정값을 쓴다
    assert c.standoff_m == STANDOFF


# ------------------------------------------------------------------ 단계 전이

def test_starts_in_yaw_phase():
    c = AlignController(cfg())
    c.begin(0.0, goal())
    assert c.phase == PHASE_YAW


def test_yaw_phase_rotates_without_driving():
    c = AlignController(cfg())
    c.begin(0.0, goal())
    cmd = drive(c, ahead(2.0, lateral=1.5), steps=40)      # 방향 오차 큼
    assert cmd.phase == PHASE_YAW
    assert cmd.v == 0.0 and cmd.w > 0.0


def test_yaw_to_distance_when_aimed():
    c = AlignController(cfg())
    c.begin(0.0, goal())
    c.step(ahead(2.0), 0.0, 0.0)
    assert c.phase == PHASE_DISTANCE


def test_distance_phase_drives_forward():
    c = AlignController(cfg(standoff_m=STANDOFF))
    c.begin(0.0, goal())
    cmd = drive(c, ahead(2.0), steps=40)
    assert cmd.phase == PHASE_DISTANCE and cmd.v > 0.0


def test_hysteresis_holds_distance_phase_through_small_wobble():
    """작은 방향 흔들림으로 단계가 왕복하면 안 된다."""
    c = AlignController(cfg(yaw_enter_tol_rad=0.06, yaw_exit_tol_rad=0.20))
    c.begin(0.0, goal())
    c.step(ahead(2.0), 0.0, 0.0)
    assert c.phase == PHASE_DISTANCE
    wobble = PlanarPose(2.0, 2.0 * math.tan(0.15), math.pi)   # bearing 0.15
    c.step(wobble, DT, DT)
    assert c.phase == PHASE_DISTANCE, '진입 문턱만 있으면 여기서 되돌아간다'


def test_hysteresis_falls_back_on_large_error():
    c = AlignController(cfg(yaw_exit_tol_rad=0.20))
    c.begin(0.0, goal())
    c.step(ahead(2.0), 0.0, 0.0)
    big = PlanarPose(2.0, 2.0 * math.tan(0.4), math.pi)
    c.step(big, DT, DT)
    assert c.phase == PHASE_YAW


def test_reaches_final_phase_at_standoff():
    c = AlignController(cfg(standoff_m=STANDOFF, dist_enter_tol_m=0.03))
    c.begin(0.0, goal())
    c.step(ahead(0.46), 0.0, 0.0)               # YAW -> DISTANCE
    c.step(ahead(0.46), DT, DT)                 # DISTANCE -> FINAL
    assert c.phase == PHASE_FINAL


# ------------------------------------------------------------------ 종료

def test_succeeds_after_stable_time():
    c = AlignController(cfg(standoff_m=STANDOFF))
    c.begin(0.0, goal(stable_time_s=1.0))
    cmd = drive(c, ahead(0.46), steps=200)
    assert cmd.done and cmd.failure == FAILURE_NONE
    assert cmd.v == 0.0 and cmd.w == 0.0
    assert cmd.stable_for_s >= 1.0


def test_does_not_succeed_before_stable_time():
    c = AlignController(cfg(standoff_m=STANDOFF))
    c.begin(0.0, goal(stable_time_s=1.0))
    cmd = drive(c, ahead(0.46), steps=10)       # 0.5 s만
    assert not cmd.done


def test_stability_timer_resets_when_leaving_tolerance():
    c = AlignController(cfg(standoff_m=STANDOFF))
    c.begin(0.0, goal(stable_time_s=1.0))
    drive(c, ahead(0.46), steps=15)
    assert c._stable_since is not None
    c.step(ahead(1.5), 20 * DT, 20 * DT)        # 허용오차 밖으로
    assert c._stable_since is None


def test_yaw_tolerance_blocks_success():
    """위치가 맞아도 자세가 틀어져 있으면 성공이 아니다."""
    c = AlignController(cfg(standoff_m=STANDOFF))
    c.begin(0.0, goal(stable_time_s=0.2, yaw_tol_rad=0.05))
    cmd = drive(c, ahead(0.46, normal_offset=0.3), steps=100)
    assert not cmd.done


# ------------------------------------------------------------------ 실패

def test_stale_stops_but_does_not_fail():
    c = AlignController(cfg(stale_timeout_s=0.4, loss_timeout_s=3.0))
    c.begin(0.0, goal())
    cmd = c.step(None, None, 1.0)
    assert cmd.failure == FAILURE_NONE and cmd.v == 0.0 and cmd.w == 0.0


def test_marker_lost_after_loss_timeout():
    c = AlignController(cfg(loss_timeout_s=3.0))
    c.begin(0.0, goal())
    c.step(None, None, 0.0)
    assert c.step(None, None, 2.9).failure == FAILURE_NONE
    assert c.step(None, None, 3.1).failure == FAILURE_MARKER_LOST


def test_fresh_sample_clears_loss_timer():
    c = AlignController(cfg(loss_timeout_s=3.0))
    c.begin(0.0, goal())
    c.step(None, None, 0.0)
    c.step(ahead(1.0), 2.0, 2.0)
    assert c._loss_since is None
    assert c.step(None, None, 4.5).failure == FAILURE_NONE


def test_goal_timeout():
    c = AlignController(cfg())
    c.begin(0.0, goal(timeout_s=1.0))
    cmd = c.step(ahead(2.0), 1.5, 1.5)
    assert cmd.failure == FAILURE_TIMEOUT and cmd.v == 0.0


def test_failure_always_zeroes_velocity():
    c = AlignController(cfg(loss_timeout_s=0.5))
    c.begin(0.0, goal())
    drive(c, ahead(2.0), steps=20)              # 먼저 달리게 해 둔다
    c.step(None, None, 10.0)
    cmd = c.step(None, None, 11.0)
    assert cmd.failure == FAILURE_MARKER_LOST
    assert cmd.v == 0.0 and cmd.w == 0.0


# ------------------------------------------------------------------ 발진 감지

def test_oscillation_detector_flags_alternating_signs():
    c = AlignController(cfg(oscillation_window=10, oscillation_flips=8))
    c.begin(0.0, goal())
    flagged = False
    for i in range(20):
        flagged = c._note_angular_sign(0.3 if i % 2 == 0 else -0.3)
    assert flagged and c.oscillation_count > 0


def test_oscillation_detector_ignores_steady_turning():
    c = AlignController(cfg(oscillation_window=10, oscillation_flips=8))
    c.begin(0.0, goal())
    flagged = any(c._note_angular_sign(0.3) for _ in range(20))
    assert not flagged


def test_oscillation_is_flagged_not_terminated():
    """발진 판정은 제어기가 표시만 하고, 중단 여부는 액션 노드가 정한다.

    `FAILURE_OSCILLATION`이 제어기 안에서는 절대 반환되지 않는다는 사실을 못박아
    둔다. 흔들리더라도 수렴 중일 수 있어서, 중단은 정책이지 제어법이 아니다.
    """
    c = AlignController(cfg(oscillation_window=4, oscillation_flips=2))
    c.begin(0.0, goal())
    for i in range(20):
        c._note_angular_sign(0.3 if i % 2 == 0 else -0.3)
    assert c.oscillation_count > 0
    cmd = c.step(ahead(2.0), 0.0, 0.0)
    assert cmd.failure != FAILURE_OSCILLATION


def test_oscillation_detector_ignores_zero_commands():
    c = AlignController(cfg(oscillation_window=4, oscillation_flips=2))
    c.begin(0.0, goal())
    assert not any(c._note_angular_sign(0.0) for _ in range(20))


def test_oscillation_needs_full_window():
    """창이 차기 전에는 판정하지 않는다 — 초기 몇 걸음으로 오판하면 안 된다."""
    c = AlignController(cfg(oscillation_window=30, oscillation_flips=2))
    c.begin(0.0, goal())
    flagged = [c._note_angular_sign(0.3 if i % 2 == 0 else -0.3) for i in range(10)]
    assert not any(flagged)


# ------------------------------------------------------------------ 재시작

def test_begin_resets_state():
    c = AlignController(cfg())
    c.begin(0.0, goal())
    drive(c, ahead(2.0, lateral=1.5), steps=20)
    assert c.shaper.prev_w != 0.0
    c.begin(100.0, goal())
    assert c.phase == PHASE_YAW
    assert c.shaper.prev_w == 0.0
    assert c._stable_since is None and c._loss_since is None


# ------------------------------------------------------------------ 폐루프 수렴

def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class _World:
    """마커 하나와 차동구동 로봇. ROS 없이 제어법만 닫아 본다.

    고정 자세를 먹이는 위 시험들은 단계 전이와 종료 판정을 잡지만 **제어법이
    오차를 실제로 지우는지**는 잡지 못한다. 진입점 조준을 넣기 전 구현은 위
    시험을 전부 통과하면서도 횡오차를 끝까지 남겼다.
    """

    def __init__(self, marker_xy, marker_yaw, robot_xy=(0.0, 0.0), robot_yaw=0.0):
        self.mx, self.my = marker_xy
        self.m_yaw = marker_yaw
        self.rx, self.ry = robot_xy
        self.rth = robot_yaw

    def sense(self):
        """로봇이 보는 마커 자세."""
        dx, dy = self.mx - self.rx, self.my - self.ry
        c, s = math.cos(self.rth), math.sin(self.rth)
        return PlanarPose(dx * c + dy * s, -dx * s + dy * c,
                          _wrap(self.m_yaw - self.rth))

    def apply(self, v, w, dt):
        self.rx += v * math.cos(self.rth) * dt
        self.ry += v * math.sin(self.rth) * dt
        self.rth = _wrap(self.rth + w * dt)

    def error_to(self, standoff):
        ex = self.mx + standoff * math.cos(self.m_yaw)
        ey = self.my + standoff * math.sin(self.m_yaw)
        return (math.hypot(self.rx - ex, self.ry - ey),
                abs(_wrap(self.rth - _wrap(self.m_yaw + math.pi))))


def close_loop(world, ctrl, steps=2000, dt=DT):
    cmd = None
    for i in range(steps):
        now = i * dt
        cmd = ctrl.step(world.sense(), now, now)
        if cmd.finished:
            break
        world.apply(cmd.v, cmd.w, dt)
    return cmd


@pytest.mark.parametrize('lateral_offset', [0.0, 0.05, 0.15, 0.30, 0.60])
def test_converges_from_lateral_offset(lateral_offset):
    """법선 축에서 옆으로 벗어나 출발해도 허용오차 안으로 들어와야 한다.

    이것이 진입점 조준을 넣은 이유다. 마커 중심을 조준하던 구현은 잔류 횡오차가
    시작 오프셋의 약 1/4로 남아 0.15 m를 넘는 순간부터 이 시험이 깨졌다.
    """
    world = _World(marker_xy=(2.0, lateral_offset), marker_yaw=math.pi)
    c = AlignController(cfg(standoff_m=STANDOFF))
    c.begin(0.0, goal(timeout_s=120.0))
    cmd = close_loop(world, c)
    pos_err, yaw_err = world.error_to(STANDOFF)
    assert cmd.done, f'수렴 실패: {cmd.failure} pos={pos_err:.4f} yaw={yaw_err:.4f}'
    assert pos_err <= 0.04
    assert yaw_err <= 0.14


def test_converges_when_marker_is_turned_away():
    """마커가 로봇을 정면으로 보지 않는 경우 — 벽에 비스듬히 붙은 타깃."""
    world = _World(marker_xy=(1.5, 0.4), marker_yaw=math.pi - 0.6)
    c = AlignController(cfg(standoff_m=STANDOFF))
    c.begin(0.0, goal(timeout_s=120.0))
    cmd = close_loop(world, c)
    pos_err, yaw_err = world.error_to(STANDOFF)
    assert cmd.done, f'수렴 실패: {cmd.failure} pos={pos_err:.4f} yaw={yaw_err:.4f}'
    assert pos_err <= 0.04 and yaw_err <= 0.14


def test_pos_err_equals_distance_to_entry_point():
    """`pos_err`와 진입점까지 거리가 같은 수라는 것이 설계의 전제다.

    접근 목표(진입점)와 성공 판정 기준(pos_err)이 한 점이어야 "진입점에 닿았다"가
    곧 "허용오차 안에 들어왔다"가 된다. 갈리면 도착하고도 성공 판정이 안 선다.
    """
    c = AlignController(cfg(standoff_m=STANDOFF))
    c.begin(0.0, goal())
    for pose in (ahead(1.0), ahead(1.0, lateral=0.4),
                 ahead(0.8, lateral=-0.3, normal_offset=0.5),
                 PlanarPose(0.3, 0.9, 2.2)):
        g = c._geometry(pose)
        entry = math.hypot(pose.x + STANDOFF * math.cos(pose.yaw),
                           pose.y + STANDOFF * math.sin(pose.yaw))
        assert g.pos_err == pytest.approx(entry, abs=1e-12)


def test_reverses_when_past_the_entry_point():
    """진입점을 지나쳤으면 후진한다. 거리 대신 전방 성분을 쓰는 이유다."""
    c = AlignController(cfg(standoff_m=STANDOFF))
    c.begin(0.0, goal())
    c.step(ahead(1.0), 0.0, 0.0)                  # YAW -> DISTANCE
    assert c.phase == PHASE_DISTANCE
    g = c._geometry(ahead(0.20))                  # standoff보다 가깝다 = 지나쳤다
    v, _ = c._law(g)
    assert g.entry_forward < 0.0 and v < 0.0


def test_entry_aiming_does_not_thrash_at_the_goal():
    """진입점 위에서는 방위가 의미를 잃는다. 그 상태로 YAW로 되돌아가면 안 된다."""
    c = AlignController(cfg(standoff_m=STANDOFF))
    c.begin(0.0, goal(dist_tol_m=0.04))
    c.step(ahead(1.0), 0.0, 0.0)
    assert c.phase == PHASE_DISTANCE
    # 진입점까지 2 cm, 그런데 그 2 cm가 거의 옆이라 진입점 방위는 90도에 가깝다.
    at_goal = PlanarPose(0.45, 0.02, math.pi)
    g = c._geometry(at_goal)
    assert abs(g.entry_bearing) > c.config.yaw_exit_tol_rad, '전제: 방위는 문턱 밖'
    c.step(at_goal, DT, DT)
    assert c.phase == PHASE_FINAL, 'YAW로 되돌아가면 제자리 회전에 갇힌다'
