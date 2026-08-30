"""ROS 비의존 고정 마커 정렬 제어기 — 마커 앞 정해진 자세로 맞춘다.

추종과 달리 여기엔 **종료 조건이 있다.** 마커 법선 위 `standoff` 지점에, 마커를
정면으로 보는 자세로 서면 끝이다. 그래서 오차가 셋이다 — 거리, 횡방향, 그리고
자세. 차동구동은 자유도가 둘뿐이라 셋을 동시에 닫을 수 없으므로 단계를 나눈다.

    YAW_ALIGN  제자리에서 진입점 방향으로 돈다.
    DISTANCE   진입점으로 접근한다. 방향은 계속 닫는다.
    FINAL      거리·방향·자세를 함께 다듬고, 허용오차 안에 머문 시간을 잰다.

**앞의 두 단계가 조준하는 것은 마커 중심이 아니라 "진입점"이다** — 마커 바깥 법선 위
standoff 지점, 즉 최종적으로 로봇이 서야 할 자리다. 마커 중심을 조준하면 오차 셋 중
횡방향이 어디에서도 닫히지 않는다. 차동구동은 옆으로 못 가므로 횡오차는 호를 그려서만
지울 수 있는데, 마커 중심 방위와 자세 오차만으로 만든 평형은 그 호를 지시하지 않는다.
진입점을 조준하면 횡오차가 방위 오차로 환산돼 기존 `k_bearing` 항이 그대로 닫는다.

> **2026-08-28 실측이 이 설계를 불렀다.** 마커 중심을 조준하던 원래 이식본은 로봇이
> 법선 축에서 옆으로 벗어나 출발하면 **시작 오프셋의 약 4분의 1이 횡오차로 남았고**
> (0.30 m → 0.073 m), 허용오차 0.04 m 안으로 들어오지 못해 timeout으로 끝났다.
> `maknae`의 원본도 같은 구조였고, 그쪽은 실기에서 이 루프를 끝까지 돌린 적이 없다.

`maknae/mm_control/align_controller.py`를 옮겨온 것이다. 바뀐 곳은 입력 규약뿐으로,
그쪽 `ArucoError(tx, ty, rz)` 대신 이 저장소의 `MarkerTrack(x, y, yaw)`를 받는다.

`YAW_ALIGN ↔ DISTANCE` 전이에 히스테리시스를 둔 이유는, 문턱 하나로 오가면 접근
도중 방향 오차가 문턱 근처에서 떨릴 때 두 단계를 계속 왕복하기 때문이다.

이 모듈에는 rclpy import가 없다. 단계 전이와 종료 판정을 액션 서버 없이 시험할 수
있어야 `align_node.py`의 배선 버그와 제어 버그를 구분할 수 있다.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
from typing import Optional

from mr_aruco.shaping import CommandShaper, ShapeLimits
from mr_aruco.tracking import PlanarPose, wrap_angle

PHASE_YAW = 'YAW_ALIGN'
PHASE_DISTANCE = 'DISTANCE'
PHASE_FINAL = 'FINAL'

FAILURE_NONE = ''
FAILURE_MARKER_LOST = 'marker_lost'
FAILURE_TIMEOUT = 'timeout'
FAILURE_OSCILLATION = 'oscillation'


@dataclass
class AlignConfig:
    # ---- 목표 자세
    standoff_m: float = 0.45

    # ---- 게인
    k_bearing: float = 1.4
    k_dist: float = 0.8
    k_final_yaw: float = 0.6

    # ---- 상한. 추종보다 느리다 — 정밀도가 목적이고 거리가 짧다.
    max_linear_mps: float = 0.15
    max_angular_rps: float = 0.7
    max_linear_accel: float = 0.5
    max_angular_accel: float = 2.5
    min_linear_mps: float = 0.004
    min_angular_rps: float = 0.008

    # ---- 단계 전이 (히스테리시스)
    yaw_enter_tol_rad: float = 0.06     # 이보다 작아지면 YAW_ALIGN을 나간다
    yaw_exit_tol_rad: float = 0.20      # 이보다 커지면 YAW_ALIGN으로 돌아간다
    # 진입점까지 남은 거리가 이만큼이면 FINAL로 넘어간다. goal의 위치 허용오차가
    # 이보다 크면 그쪽을 쓴다 — 이미 성공 판정 안에 들어왔는데 접근을 계속하면
    # 진입점 방위가 의미를 잃은 구간에서 헛돈다.
    dist_enter_tol_m: float = 0.03

    # ---- 종료 판정
    # 표본이 이보다 낡으면 정지하고 기다린다. 실패는 아니다.
    stale_timeout_s: float = 0.4
    # 그 상태가 이만큼 이어지면 실패로 끝낸다.
    loss_timeout_s: float = 3.0
    # 최근 각속도 지령 부호가 이 창 안에서 이만큼 뒤집히면 발진으로 본다.
    oscillation_window: int = 30
    oscillation_flips: int = 8

    def limits(self) -> ShapeLimits:
        return ShapeLimits(
            max_linear_mps=self.max_linear_mps,
            max_angular_rps=self.max_angular_rps,
            max_linear_accel=self.max_linear_accel,
            max_angular_accel=self.max_angular_accel,
            min_linear_mps=self.min_linear_mps,
            min_angular_rps=self.min_angular_rps,
        )


@dataclass(frozen=True)
class AlignGoal:
    """액션 goal에서 온 값. 0 이하이면 설정 기본값을 쓴다는 뜻이다."""

    standoff_m: float = 0.0
    dist_tol_m: float = 0.04
    yaw_tol_rad: float = 0.14
    stable_time_s: float = 1.0
    timeout_s: float = 60.0


@dataclass
class AlignCommand:
    """한 주기의 결과. step()이 값을 채워가며 만들기 때문에 가변이다."""

    v: float = 0.0
    w: float = 0.0
    phase: str = PHASE_YAW
    done: bool = False
    failure: str = FAILURE_NONE
    stable_for_s: float = 0.0
    # 진단·피드백용. 표본이 없으면 nan.
    dist_err_m: float = float('nan')
    lateral_err_m: float = float('nan')
    yaw_err_rad: float = float('nan')
    oscillating: bool = False

    @property
    def finished(self) -> bool:
        return self.done or self.failure != FAILURE_NONE


@dataclass
class AlignController:
    config: AlignConfig = field(default_factory=AlignConfig)

    def __post_init__(self) -> None:
        self.shaper = CommandShaper(self.config.limits())
        self.goal = AlignGoal()
        self.begin(0.0, self.goal)

    # ------------------------------------------------------------------ 수명

    def begin(self, now: float, goal: AlignGoal) -> None:
        """새 goal을 받는다. 상태를 전부 초기화한다."""
        self.goal = goal
        self.standoff_m = goal.standoff_m if goal.standoff_m > 0.0 else self.config.standoff_m
        self._final_enter_m = max(self.config.dist_enter_tol_m, goal.dist_tol_m)
        self.phase = PHASE_YAW
        self.shaper.reset()
        self._started_at = now
        self._prev_time: Optional[float] = None
        self._loss_since: Optional[float] = None
        self._stable_since: Optional[float] = None
        self._w_signs: deque = deque(maxlen=self.config.oscillation_window)
        self.oscillation_count = 0

    # ------------------------------------------------------------------ 한 걸음

    def step(self, pose: Optional[PlanarPose], stamp_s: Optional[float],
             now: float) -> AlignCommand:
        """제어 주기마다 한 번 부른다. `finished`가 참이면 그만 부른다."""
        cfg = self.config
        dt = 0.0 if self._prev_time is None else max(0.0, now - self._prev_time)
        self._prev_time = now

        if now - self._started_at > self.goal.timeout_s:
            return self._finish(AlignCommand(phase=self.phase, failure=FAILURE_TIMEOUT))

        fresh = (pose is not None and stamp_s is not None
                 and (now - stamp_s) <= cfg.stale_timeout_s)
        if not fresh:
            if self._loss_since is None:
                self._loss_since = now
            self._stable_since = None
            if now - self._loss_since > cfg.loss_timeout_s:
                return self._finish(
                    AlignCommand(phase=self.phase, failure=FAILURE_MARKER_LOST))
            # 아직 실패는 아니다. 멈추고 기다린다.
            v, w = self.shaper.shape(0.0, 0.0, dt)
            return AlignCommand(v=v, w=w, phase=self.phase)
        self._loss_since = None

        assert pose is not None
        geom = self._geometry(pose)

        self._advance_phase(geom)
        v_cmd, w_cmd = self._law(geom)

        out = AlignCommand(phase=self.phase, dist_err_m=geom.dist_err,
                           lateral_err_m=geom.lateral, yaw_err_rad=geom.yaw_err)

        # 성공 판정은 FINAL에서만 한다. 허용오차 안에 연속으로 머물러야 한다.
        inside = (geom.pos_err <= self.goal.dist_tol_m
                  and abs(geom.yaw_err) <= self.goal.yaw_tol_rad)
        if self.phase == PHASE_FINAL and inside:
            if self._stable_since is None:
                self._stable_since = now
            out.stable_for_s = now - self._stable_since
            if out.stable_for_s >= self.goal.stable_time_s:
                out.done = True
                return self._finish(out)
        else:
            self._stable_since = None

        return self._emit(v_cmd, w_cmd, dt, out)

    # -------------------------------------------------------------- 기하·제어법

    def _geometry(self, pose: PlanarPose) -> '_Geometry':
        """마커 자세에서 정렬 오차 셋과 진입점을 뽑는다.

        진입점은 "마커 바깥 법선 위 standoff 지점"이고, 로봇이 최종적으로 서야 할
        자리다. 그 점에서 얼마나 벗어났는지를 법선 방향(축방향)과 그에 수직인
        방향(횡방향)으로 나눈 것이 `dist_err`와 `lateral`이다.

        `pos_err = hypot(dist_err, lateral)`는 **진입점까지의 직선 거리와 같다.**
        두 표현이 같은 수라는 것이 `entry_*`를 조준해도 성공 판정이 그대로 성립하는
        이유다 — 접근 목표와 판정 목표가 한 점이다.
        """
        bearing = pose.bearing_rad
        los_m = pose.range_m

        # 로봇이 마커를 정면으로 보기까지 돌아야 할 각. 법선이 로봇 쪽을 향할 때 0.
        yaw_err = wrap_angle(pose.yaw + math.pi)

        nx, ny = math.cos(pose.yaw), math.sin(pose.yaw)   # 마커 바깥 법선
        vx, vy = -pose.x, -pose.y                          # 마커 → 로봇
        axial = nx * vx + ny * vy                          # 법선 위 거리
        lateral = nx * vy - ny * vx                        # 법선에서 옆으로 벗어난 양

        dist_err = axial - self.standoff_m

        # 진입점을 본체 좌표계로. 마커 위치에서 바깥 법선으로 standoff만큼 나간 점이다.
        entry_x = pose.x + self.standoff_m * nx
        entry_y = pose.y + self.standoff_m * ny
        return _Geometry(bearing=bearing, los_m=los_m, yaw_err=yaw_err,
                         axial=axial, lateral=lateral, dist_err=dist_err,
                         pos_err=math.hypot(dist_err, lateral),
                         entry_bearing=math.atan2(entry_y, entry_x),
                         entry_forward=entry_x)

    def _advance_phase(self, g: '_Geometry') -> None:
        cfg = self.config
        if self.phase == PHASE_YAW:
            if abs(g.entry_bearing) <= cfg.yaw_enter_tol_rad:
                self.phase = PHASE_DISTANCE
        elif self.phase == PHASE_DISTANCE:
            # 도착 판정이 먼저다. 진입점 위에 서면 그 점의 방위가 의미를 잃는데,
            # 그 상태에서 방위 문턱으로 YAW로 되돌아가면 제자리 회전에 갇힌다.
            if g.pos_err <= self._final_enter_m:
                self.phase = PHASE_FINAL
            elif abs(g.entry_bearing) > cfg.yaw_exit_tol_rad:
                self.phase = PHASE_YAW

    def _law(self, g: '_Geometry') -> tuple:
        """단계별 (v, ω). 앞 두 단계는 진입점을, FINAL은 마커를 본다.

        FINAL에서 마커 중심 방위로 되돌아가는 것은 실수가 아니다. 진입점 위에
        서면 `lateral`이 이미 0이라 "마커를 향한다"와 "법선에 정렬한다"가 같은
        방향이 되고, 그 근처에서는 진입점 방위가 잡음에 취약하다.

        `entry_forward`는 진입점의 **전방 성분**이라 부호가 있다. 지나쳤으면 음수가
        되어 그대로 후진 지령이 된다. 거리(`hypot`)를 쓰면 지나친 뒤에도 계속
        전진해 되돌아오지 못한다.
        """
        cfg = self.config
        if self.phase == PHASE_YAW:
            return 0.0, cfg.k_bearing * g.entry_bearing
        if self.phase == PHASE_DISTANCE:
            return cfg.k_dist * g.entry_forward, cfg.k_bearing * g.entry_bearing
        return (cfg.k_dist * g.dist_err,
                cfg.k_bearing * g.bearing + cfg.k_final_yaw * g.yaw_err)

    # ---------------------------------------------------------------- 내부 처리

    def _emit(self, v_cmd: float, w_cmd: float, dt: float,
              out: AlignCommand) -> AlignCommand:
        out.v, out.w = self.shaper.shape(v_cmd, w_cmd, dt)
        out.oscillating = self._note_angular_sign(out.w)
        return out

    def _note_angular_sign(self, w: float) -> bool:
        """각속도 부호가 자주 뒤집히면 발진으로 본다.

        게인이 높거나 마커 자세가 떨릴 때 로봇이 좌우로 진동하며 수렴하지 못하는
        경우가 있다. 시간 초과까지 기다리면 그동안 계속 흔들리므로 일찍 잡아낸다.
        """
        if w == 0.0:
            return False
        self._w_signs.append(1 if w > 0 else -1)
        if len(self._w_signs) < self._w_signs.maxlen:
            return False
        signs = list(self._w_signs)
        flips = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
        if flips >= self.config.oscillation_flips:
            self.oscillation_count += 1
            return True
        return False

    def _finish(self, out: AlignCommand) -> AlignCommand:
        """종료 지령은 항상 0이다. 슬루를 거치지 않고 즉시 세운다."""
        self.shaper.reset()
        out.v = 0.0
        out.w = 0.0
        return out


@dataclass(frozen=True)
class _Geometry:
    bearing: float          # 마커 중심 방위
    los_m: float            # 마커까지 시선 거리
    yaw_err: float          # 마커를 정면으로 보기까지 돌 각
    axial: float            # 법선 위 거리
    lateral: float          # 법선에서 옆으로 벗어난 양
    dist_err: float         # axial - standoff
    pos_err: float          # hypot(dist_err, lateral) = 진입점까지 거리
    entry_bearing: float    # 진입점 방위
    entry_forward: float    # 진입점의 전방 성분 (부호 있음)
