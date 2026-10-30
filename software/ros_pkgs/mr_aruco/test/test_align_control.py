"""AlignController 단계 전이·기하·종료 판정 시험. ROS 없이 돈다."""
import math

from mr_aruco.align_control import (
    AlignCommand, AlignConfig, AlignController, AlignGoal,
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


def test_holds_zero_while_confirming_stability():
    """허용오차 안에서 최소 회전 지령이 로봇을 다시 밀어내면 안 된다."""
    c = AlignController(cfg(standoff_m=STANDOFF, min_turn_rps=0.20))
    c.begin(0.0, goal(stable_time_s=1.0))
    c.step(ahead(0.46), 0.0, 0.0)       # YAW -> DISTANCE
    c.step(ahead(0.46), DT, DT)          # DISTANCE -> FINAL
    cmd = c.step(ahead(0.46, normal_offset=0.05), 2 * DT, 2 * DT)
    assert c.phase == PHASE_FINAL
    assert not cmd.done
    assert cmd.v == 0.0 and cmd.w == 0.0


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


def test_oscillation_detector_ignores_unexecutable_small_corrections():
    c = AlignController(cfg(oscillation_window=4, oscillation_flips=2,
                            oscillation_min_angular_rps=0.15))
    c.begin(0.0, goal())
    flagged = [c._note_angular_sign(0.05 if i % 2 == 0 else -0.05)
               for i in range(20)]
    assert not any(flagged)
    assert c.oscillation_count == 0


def test_nonzero_turn_is_lifted_above_floor():
    c = AlignController(cfg(min_turn_rps=0.20, max_angular_accel=100.0))
    c.begin(0.0, goal())
    out = AlignCommand()
    cmd = c._emit(0.0, -0.01, DT, out)
    assert cmd.w == pytest.approx(-0.20)


def test_moving_steer_is_not_lifted_to_in_place_floor():
    """전진 중 조향까지 최소 제자리 회전으로 올리면 카메라 시야를 벗어난다."""
    c = AlignController(cfg(min_turn_rps=0.20, max_angular_accel=100.0))
    c.begin(0.0, goal())
    out = AlignCommand()
    cmd = c._emit(0.10, -0.01, DT, out)
    assert cmd.v > 0.0
    assert cmd.w == pytest.approx(-0.01)


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


# ------------------------------------------------------- 시야 유지 (2026-08-30)
#
# M12-2 2회차는 진입점 조준이 정확한데도 실패했다. 로봇이 진입점을 향해 호를
# 그리는 동안 마커가 화면 오른쪽으로 밀려나가 MARKER_LOST가 났다. 제어법에
# "마커를 계속 봐야 한다"는 조건이 없었던 것이다. 아래 시험들이 그 조건을 건다.

# c920_calibration.yaml 1280x720 실측 내부 파라미터. 주점이 중앙(640)에서
# 오른쪽으로 치우쳐 좌우 화각이 다르다.
FX_PX = 955.4046808917496
CX_PX = 680.6881205097441
IMAGE_WIDTH_PX = 1280
FOV_LEFT = math.atan(CX_PX / FX_PX)                       # 0.6191 rad = 35.47 deg
FOV_RIGHT = math.atan((IMAGE_WIDTH_PX - CX_PX) / FX_PX)   # 0.5603 rad = 32.10 deg
# mr_description TF 실측: base_footprint -> camera_optical_frame [0.250, 0, 0.410]
CAMERA_FORWARD = 0.250
MARKER_HALF_M = 0.075          # aruco_tracker.yaml marker_size 0.15


def _camera_view(world, camera_forward=CAMERA_FORWARD):
    """마커 중심의 카메라 기준 방위와 거리.

    카메라는 본체와 함께 돌면서 반경 `camera_forward`의 호를 그린다. 그래서
    제자리 회전만으로도 카메라 기준 방위는 본체 기준 방위보다 크게 움직인다.
    """
    cx = world.rx + camera_forward * math.cos(world.rth)
    cy = world.ry + camera_forward * math.sin(world.rth)
    dx, dy = world.mx - cx, world.my - cy
    c, s = math.cos(world.rth), math.sin(world.rth)
    bx, by = dx * c + dy * s, -dx * s + dy * c
    return math.atan2(by, bx), math.hypot(bx, by)


def _fov_margin(world):
    """마커의 **바깥 모서리**가 화면 경계까지 남긴 각. 음수면 잘렸다.

    중심이 아니라 모서리로 판정한다 — ArUco는 네 꼭짓점이 다 보여야 검출된다.
    기울어진 마커의 투영 폭은 실제 폭보다 좁지만, 여기서는 줄이지 않고 정면
    폭을 그대로 써서 보수적으로 본다.
    """
    bearing, rng = _camera_view(world)
    half = math.atan2(MARKER_HALF_M, max(rng, 1e-6))
    edge = FOV_LEFT if bearing >= 0.0 else FOV_RIGHT
    return edge - (abs(bearing) + half)


def close_loop_watching_fov(world, ctrl, steps=4000, dt=DT):
    """폐루프를 돌리면서 마커가 화면을 벗어난 최악 순간을 함께 낸다."""
    cmd, worst = None, float('inf')
    for i in range(steps):
        now = i * dt
        worst = min(worst, _fov_margin(world))
        cmd = ctrl.step(world.sense(), now, now)
        if cmd.finished:
            break
        world.apply(cmd.v, cmd.w, dt)
    return cmd, worst


# 시야 유지를 켤 때의 운용 설정. align.yaml과 같은 값이어야 뜻이 있다.
KEEP_STANDOFF = STANDOFF
# align.yaml과 같은 값. 마커 **중심**을 붙잡는 시선 주위 반각의 상한이고,
# 실제로 쓰는 원뿔은 거리마다 `- atan(MARKER_HALF_M / r)`만큼 좁아진다.
KEEP_RAD = 0.50


def keep_cfg(**kw):
    base = {'standoff_m': KEEP_STANDOFF, 'k_bearing': 2.8, 'max_linear_mps': 0.10,
            'max_angular_rps': 0.30, 'camera_forward_m': CAMERA_FORWARD,
            'fov_keep_rad': KEEP_RAD}
    base.update(kw)
    return AlignConfig(**base)


def test_camera_bearing_differs_from_body_bearing():
    """카메라가 앞에 있으므로 두 방위는 같지 않다. 이 차이가 M12-2를 깨뜨렸다."""
    c = AlignController(keep_cfg())
    c.begin(0.0, goal())
    g = c._geometry(ahead(0.93, lateral=-0.345))
    assert g.bearing == pytest.approx(-0.354, abs=0.01)
    assert g.cam_bearing == pytest.approx(-0.468, abs=0.01)
    assert abs(g.cam_bearing) > abs(g.bearing), '카메라 기준이 항상 더 크게 벌어진다'


def test_keep_in_view_is_off_by_default():
    """기본값은 예전 그대로다 — 조준이 진입점 방위와 같다."""
    c = AlignController(cfg(standoff_m=STANDOFF))
    c.begin(0.0, goal())
    g = c._geometry(ahead(1.2, lateral=0.8))
    assert g.aim == pytest.approx(g.entry_bearing)


def test_keep_in_view_clamps_aim_to_the_cone():
    """진입점이 원뿔 밖이면 조준을 경계로 자른다.

    진입점이 시선에서 크게 벌어지는 것은 마커가 옆을 볼 때다 — 여기서는 마커가
    정면 0.6 m에 있고 법선이 왼쪽(+y)을 향하므로 진입점은 왼쪽으로 크게 벗어난다.
    """
    c = AlignController(keep_cfg())
    c.begin(0.0, goal())
    g = c._geometry(PlanarPose(0.6, 0.0, math.pi / 2))
    assert abs(g.entry_bearing - g.cam_bearing) > g.keep_rad, '전제: 진입점이 원뿔 밖'
    assert abs(g.aim - g.cam_bearing) == pytest.approx(g.keep_rad, abs=1e-9)
    assert abs(g.aim) < abs(g.entry_bearing), '조준이 시선 쪽으로 당겨져야 한다'


def test_keep_cone_narrows_as_the_marker_gets_closer():
    """원뿔은 거리마다 다르다 — 마커 모서리가 차지하는 각을 뺀 만큼이다.

    고정 원뿔로는 두 요구를 동시에 못 맞췄다. 좁히면 큰 횡오차를 지우는 선회를
    막아 수렴이 깨졌고(진입점이 영영 원뿔 밖), 넓히면 종반에 마커가 잘렸다.
    선회는 멀리서 일어나고 잘림은 가까이서 일어나므로 거리로 가르는 게 맞다.
    """
    c = AlignController(keep_cfg())
    c.begin(0.0, goal())
    far = c._geometry(ahead(2.0)).keep_rad
    near = c._geometry(ahead(0.6)).keep_rad
    assert far > near, '멀수록 넓어야 한다'
    # 2.0 m -> 카메라 1.75 m, 마커 반각 atan(0.075/1.75) = 0.0428
    assert far == pytest.approx(KEEP_RAD - math.atan2(MARKER_HALF_M, 1.75), abs=1e-9)
    assert near == pytest.approx(KEEP_RAD - math.atan2(MARKER_HALF_M, 0.35), abs=1e-9)


def test_keep_cone_has_a_floor():
    """코앞에서는 마커 반각이 화각을 통째로 먹는다. 조향이 얼면 안 된다."""
    c = AlignController(keep_cfg())
    c.begin(0.0, goal())
    assert c._geometry(ahead(0.26)).keep_rad == pytest.approx(
        c.config.fov_keep_min_rad)


def test_keep_in_view_pulls_back_when_marker_is_past_the_edge():
    """마커가 이미 경계를 넘었으면 조준이 마커 쪽을 가리켜 되돌아본다.

    M12-2가 정확히 이 상태였다 — 진입점 조준은 ±0.035 rad로 정확했는데 마커는
    카메라 기준 -26.8도에 있었다. 제약이 없으니 그대로 밀어냈다.
    """
    c = AlignController(keep_cfg(standoff_m=0.45))
    c.begin(0.0, goal())
    # M12-2 10.6초 시점의 기하를 그대로 세운다: 진입점은 정면(방위 0)인데
    # 마커는 카메라 기준 -0.470 rad(-26.9도)에 있다. 마커가 옆을 보고 있어서다.
    g = c._geometry(PlanarPose(0.93, -0.3454, 2.2670))
    assert g.cam_bearing == pytest.approx(-0.470, abs=0.005), '전제: 마커가 원뿔 밖'
    assert abs(g.entry_bearing) < 0.01, '전제: 진입점 조준 자체는 정확하다'
    assert g.aim < 0.0, '마커 쪽(오른쪽)으로 되돌아봐야 한다'
    assert g.aim == pytest.approx(g.cam_bearing + g.keep_rad, abs=1e-9)


def test_yaw_phase_uses_clamped_aim_for_transition():
    """문턱을 진입점 방위에 걸면 원뿔 경계에서 회전이 멎고 YAW에 갇힌다."""
    c = AlignController(keep_cfg())
    c.begin(0.0, goal(timeout_s=120.0))
    world = _World(marker_xy=(1.6, 1.1), marker_yaw=math.pi)
    for i in range(400):
        now = i * DT
        cmd = c.step(world.sense(), now, now)
        world.apply(cmd.v, cmd.w, DT)
        if c.phase != PHASE_YAW:
            break
    assert c.phase != PHASE_YAW, 'YAW 단계를 못 벗어나면 timeout까지 제자리 회전한다'


@pytest.mark.parametrize('lateral_offset', [-0.30, 0.20, 0.30, 0.40])
def test_unconstrained_approach_loses_the_marker(lateral_offset):
    """제약이 없으면 마커가 화면을 벗어난다 — 고치기 전 상태를 못으로 박는다.

    수렴 판정은 전부 통과한다. 그래서 기존 시험이 이 결함을 못 잡았고, M12-2가
    바닥에서 MARKER_LOST로 끝났다.
    """
    world = _World(marker_xy=(1.4, lateral_offset), marker_yaw=math.pi)
    c = AlignController(keep_cfg(fov_keep_rad=0.0))
    c.begin(0.0, goal(timeout_s=120.0))
    cmd, worst = close_loop_watching_fov(world, c)
    assert cmd.done, '전제: 시야를 안 보면 수렴 판정은 통과한다'
    assert worst < 0.0, '전제: 그런데 마커는 화면 밖으로 나간다'


# 실측으로 확인한 운용 범위: 시작 횡오프셋 ±0.20 m, 마커 법선 어긋남 ±0.10 rad.
# 이 격자에서 수렴과 시야 유지가 모두 서고 최악 화각 여유는 +0.034 rad였다.
@pytest.mark.parametrize('lateral_offset', [-0.20, -0.10, 0.0, 0.10, 0.20])
@pytest.mark.parametrize('normal_offset', [-0.10, 0.0, 0.10])
def test_marker_stays_in_view_and_still_converges(lateral_offset, normal_offset):
    """시야 안에 붙잡아 두면서 여전히 허용오차 안으로 들어와야 한다.

    제약만 걸고 수렴을 잃으면 고친 게 아니다. 둘을 한 시험에서 같이 본다.
    """
    world = _World(marker_xy=(1.4, lateral_offset),
                   marker_yaw=math.pi - normal_offset)
    c = AlignController(keep_cfg())
    c.begin(0.0, goal(timeout_s=120.0))
    cmd, worst = close_loop_watching_fov(world, c)
    pos_err, yaw_err = world.error_to(KEEP_STANDOFF)
    assert worst >= 0.0, f'마커가 화면을 벗어났다 (여유 {worst:+.4f} rad)'
    assert cmd.done, f'수렴 실패: {cmd.failure} pos={pos_err:.4f} yaw={yaw_err:.4f}'
    assert pos_err <= 0.04 and yaw_err <= 0.14


@pytest.mark.parametrize('start_distance', [1.0, 1.2, 1.6, 2.0, 2.5])
def test_marker_stays_in_view_from_various_start_distances(start_distance):
    """시작 거리를 바꿔도 시야를 지킨다. 가까이서 출발할수록 빠듯하다."""
    world = _World(marker_xy=(start_distance, 0.20), marker_yaw=math.pi)
    c = AlignController(keep_cfg())
    c.begin(0.0, goal(timeout_s=120.0))
    cmd, worst = close_loop_watching_fov(world, c)
    assert worst >= 0.0, f'마커가 화면을 벗어났다 (여유 {worst:+.4f} rad)'
    assert cmd.done, f'수렴 실패: {cmd.failure}'


def test_strongly_turned_marker_keeps_view_but_does_not_converge():
    """크게 돌아앉은 마커는 **아직 못 한다.** 알려진 한계를 못으로 박는다.

    화각이 ±32도인데 마커 뒤쪽 진입점으로 가려면 마커에서 눈을 떼야 한다.
    시야 유지와 진입점 접근이 정면으로 충돌하는 구간이고, 조준을 자르는 것만으로는
    안 풀린다 — 멀리서 크게 선회한 뒤 접근하거나(orbit), 잠깐의 소실을 odom으로
    버티는 별도 기동이 필요하다.

    바뀐 것은 실패의 종류와 크기다. 제약이 없으면 마커를 방위 -92.9도까지
    밀어내고(여유 -1.24 rad) 그대로 성공 판정을 낸다 — 실기에서는 그 전에
    MARKER_LOST가 난다. 제약을 걸면 마커를 방위 -5.9도에 붙잡은 채 timeout으로
    끝난다. 남은 여유 부족(-0.05 rad)은 조준이 아니라 **거리** 때문이다:
    로봇이 0.135 m까지 파고들어 마커가 화면을 넘치는 것이고, 이걸 막으려면
    원뿔이 아니라 접근 자체를 멈추는 기동이 필요하다.
    """
    world = _World(marker_xy=(1.5, 0.4), marker_yaw=math.pi - 0.6)
    c = AlignController(keep_cfg())
    c.begin(0.0, goal(timeout_s=60.0))
    cmd, worst = close_loop_watching_fov(world, c)
    assert not cmd.done, '여기가 통과하기 시작하면 이 시험을 지우고 범위를 넓혀라'
    assert cmd.failure == FAILURE_TIMEOUT, '마커를 놓쳐서가 아니라 못 닿아서 끝난다'
    assert worst > -0.10, '제약 없을 때의 -1.24 rad보다는 한참 나아야 한다'

    loose = _World(marker_xy=(1.5, 0.4), marker_yaw=math.pi - 0.6)
    c2 = AlignController(keep_cfg(fov_keep_rad=0.0))
    c2.begin(0.0, goal(timeout_s=60.0))
    cmd2, worst2 = close_loop_watching_fov(loose, c2)
    assert cmd2.done and worst2 < -1.0, '전제: 제약이 없으면 크게 밀어내고도 성공한다'


@pytest.mark.parametrize('standoff', [0.45, 0.50, 0.60])
def test_standoff_choice_only_changes_the_margin(standoff):
    """standoff는 시야 유지의 전제가 아니다 — 여유만 바꾼다.

    거리 의존 원뿔을 넣기 전에는 0.45 m가 기하적으로 불가능해 보였다. 원뿔이
    종반에 스스로 좁아지면서 세 값 모두 성립하게 됐고, 달라지는 것은 최악 화각
    여유뿐이다(0.028 / 0.031 / 0.035 rad). 그래서 사양을 건드리지 않았다.
    """
    world = _World(marker_xy=(1.4, 0.20), marker_yaw=math.pi)
    c = AlignController(keep_cfg(standoff_m=standoff))
    c.begin(0.0, goal(timeout_s=120.0))
    cmd, worst = close_loop_watching_fov(world, c)
    assert cmd.done, f'수렴 실패: {cmd.failure}'
    assert worst >= 0.0, f'마커가 화면을 벗어났다 (여유 {worst:+.4f} rad)'
