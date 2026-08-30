"""ROS 비의존 작업자 추종 제어기 — 마커까지의 (ρ, α)를 (v, ω)로 바꾼다.

제어법은 위치 기반 비주얼 서보다. 차동구동이라 쓸 수 있는 자유도가 둘뿐이므로
극좌표 오차 두 개를 각각 하나씩 닫는다.

    ρ = hypot(x, y)              지면상 거리
    α = atan2(y, x)              마커 방향, + = 왼쪽
    e_ρ = ρ - ρ_d                목표 거리 대비 오차

    ω = k_α·α + k_d·α̇
    v = k_ρ·e_ρ · max(0, cos α)

`cos α` 게이트가 하는 일은 "옆으로 크게 벗어난 목표를 향해 전진하지 않는" 것이다.
차동구동은 옆으로 못 가므로 먼저 돌아야 하는데, 각도가 클 때 전진하면 호를 그리며
크게 돌아간다. `max(0, ·)`은 마커가 뒤(|α| > 90°)에 있을 때 v가 음수로 뒤집히는
것을 막는다.

**후진하지 않는다.** `e_ρ < 0`(너무 가까움)이면 v = 0으로 서기만 한다. 사람을 따라가는
로봇이 사람 쪽으로 다가온 뒤 물러서면, 뒤가 안 보이는 상태로 후진하는 셈이라
위험하다. Leo Rover 예제에는 후진 구간이 있지만 여기서는 뺐다.

이 모듈에는 rclpy import가 없다. 제어법을 시계·토픽 없이 시험할 수 있어야
`follow_node.py`의 배선 버그와 제어 버그를 구분할 수 있다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Optional

from mr_aruco.shaping import CommandShaper, ShapeLimits
from mr_aruco.tracking import PlanarPose, wrap_angle


@dataclass
class FollowConfig:
    # ---- 목표
    target_range_m: float = 1.1

    # ---- 게인
    k_range: float = 0.6
    k_bearing: float = 1.2
    # 방향 오차 미분항. 사람이 방향을 틀 때 오버슈트를 줄인다. 0으로 두면 순수 P.
    k_bearing_d: float = 0.1
    # 미분값 1차 저역통과 시정수. 마커 방향은 프레임마다 튀므로 생 미분을 쓰면
    # D항이 노이즈 증폭기가 된다.
    derivative_tau_s: float = 0.15

    # ---- 상한 (limits.md의 물리 상한이 아니라 운용 상한이다)
    max_linear_mps: float = 0.25
    max_angular_rps: float = 0.7
    max_linear_accel: float = 0.4      # m/s^2
    max_angular_accel: float = 2.0     # rad/s^2

    # ---- 오차 데드밴드: 목표점 근처에서 앞뒤로 떠는 것을 막는다
    range_deadband_m: float = 0.12
    bearing_deadband_rad: float = 0.07     # 약 4도

    # 이 각도를 넘으면 전진하지 않고 제자리에서 먼저 돈다. cos 게이트만으로도
    # 부드럽게 줄지만, 명시적 문턱이 있어야 "왜 안 가지"를 로그로 설명할 수 있다.
    align_first_rad: float = 0.44          # 약 25도

    # ---- 출력 데드밴드: 모터가 못 내는 미소 지령을 0으로 만든다
    min_linear_mps: float = 0.005
    min_angular_rps: float = 0.01

    # 이보다 오래된 표본으로는 주행하지 않는다. 마커 좌표는 base_footprint 기준이라
    # 로봇이 움직이는 동안 낡는다 — 0.25 m/s에서 0.4 s면 10 cm이고, 이는 거리
    # 데드밴드 0.12 m 안이다. 이 값을 키우려면 오도메트리로 좌표를 갱신해야 한다.
    marker_timeout_s: float = 0.4

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
class FollowCommand:
    v: float = 0.0
    w: float = 0.0
    # 표본이 없거나 낡아서 정지 중인가. 노드는 이걸 보고 로그를 낸다.
    stale: bool = True
    # 진단용. 표본이 없으면 nan.
    range_m: float = float('nan')
    bearing_rad: float = float('nan')
    range_err_m: float = float('nan')
    # |α| > align_first_rad라 전진을 막고 회전만 하는 중인가.
    turning_in_place: bool = False


@dataclass
class FollowController:
    config: FollowConfig = field(default_factory=FollowConfig)

    def __post_init__(self) -> None:
        self.shaper = CommandShaper(self.config.limits())
        self.reset()

    def reset(self) -> None:
        self.shaper.reset()
        self._prev_time: Optional[float] = None
        self._prev_bearing: Optional[float] = None
        self._bearing_rate = 0.0

    @property
    def prev_v(self) -> float:
        return self.shaper.prev_v

    @property
    def prev_w(self) -> float:
        return self.shaper.prev_w

    # ------------------------------------------------------------------ 한 걸음

    def step(self, pose: Optional[PlanarPose], stamp_s: Optional[float],
             now: float) -> FollowCommand:
        """제어 주기마다 한 번 부른다.

        Args:
            pose: 최신 추적 자세. 없으면 None.
            stamp_s: 그 자세가 **실제로 관측된** 시각. hold 중이면 갱신되지 않으므로
                자연히 낡아 timeout에 걸린다. 이게 hold를 안전하게 만드는 장치다.
            now: 현재 시각 [s].
        """
        cfg = self.config
        dt = 0.0 if self._prev_time is None else max(0.0, now - self._prev_time)
        self._prev_time = now

        fresh = (pose is not None and stamp_s is not None
                 and (now - stamp_s) <= cfg.marker_timeout_s)
        if not fresh:
            self._prev_bearing = None
            self._bearing_rate = 0.0
            # 슬루를 거쳐 0으로 내린다. 갑자기 끊는 것보다 낫고, 어차피 다음
            # 주기에도 표본이 없으면 계속 0으로 수렴한다.
            v, w = self.shaper.shape(0.0, 0.0, dt)
            return FollowCommand(v=v, w=w, stale=True)

        assert pose is not None
        rho = pose.range_m
        alpha = pose.bearing_rad
        e_rho = rho - cfg.target_range_m

        rate = self._update_bearing_rate(alpha, dt)

        # ---- 각속도
        w_raw = cfg.k_bearing * alpha + cfg.k_bearing_d * rate
        if abs(alpha) < cfg.bearing_deadband_rad:
            w_raw = 0.0

        # ---- 선속도
        turning = abs(alpha) > cfg.align_first_rad
        if abs(e_rho) < cfg.range_deadband_m:
            v_raw = 0.0
        elif e_rho < 0.0:
            v_raw = 0.0                      # 후진 금지
        elif turning:
            v_raw = 0.0                      # 먼저 회전
        else:
            v_raw = cfg.k_range * e_rho * max(0.0, math.cos(alpha))

        v, w = self.shaper.shape(v_raw, w_raw, dt)
        return FollowCommand(v=v, w=w, stale=False, range_m=rho, bearing_rad=alpha,
                             range_err_m=e_rho, turning_in_place=turning)

    # -------------------------------------------------------------- 내부 처리

    def _update_bearing_rate(self, alpha: float, dt: float) -> float:
        """α̇를 1차 저역통과로 추정한다."""
        if self._prev_bearing is None or dt <= 0.0:
            self._prev_bearing = alpha
            return self._bearing_rate
        raw = wrap_angle(alpha - self._prev_bearing) / dt
        self._prev_bearing = alpha
        tau = self.config.derivative_tau_s
        beta = 1.0 if tau <= 0.0 else min(1.0, dt / tau)
        self._bearing_rate += beta * (raw - self._bearing_rate)
        return self._bearing_rate
