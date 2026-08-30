"""두 제어기가 공유하는 출력 정형 — 포화 → 슬루 → 데드밴드.

추종과 정렬은 제어법이 다르지만 마지막에 (v, ω)를 다듬는 절차는 같다. 그런데 이
절차는 **순서가 결과를 바꾸는** 종류라, 두 벌로 두면 언젠가 한쪽만 고쳐지고 갈린다.
그래서 한 곳에 둔다.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ShapeLimits:
    max_linear_mps: float
    max_angular_rps: float
    max_linear_accel: float        # m/s^2
    max_angular_accel: float       # rad/s^2
    min_linear_mps: float          # 출력 데드밴드
    min_angular_rps: float


class CommandShaper:
    """정형기. 슬루를 위해 직전 지령을 기억한다."""

    def __init__(self, limits: ShapeLimits) -> None:
        self.limits = limits
        self.reset()

    def reset(self) -> None:
        self.prev_v = 0.0
        self.prev_w = 0.0

    def shape(self, v_cmd: float, w_cmd: float, dt: float) -> tuple:
        """포화 → 슬루 → 출력 데드밴드 순으로 지령을 다듬는다.

        **슬루를 포화보다 뒤에 두는 이유**는, 포화가 먼저 걸려야 슬루가 도달 불가능한
        목표를 향해 램프를 시작하지 않기 때문이다. 순서를 바꾸면 큰 오차에서
        지령이 상한을 넘어선 채로 램프하다가 나중에 잘리면서 응답이 달라진다.

        **데드밴드를 마지막에 두는 이유**는 슬루가 만들어낸 미소 중간값까지
        걸러내기 위해서다.

        첫 걸음은 dt = 0으로 부르면 슬루 폭이 0이라 결과가 직전 값(0)에 묶인다.
        정지 상태에서 갑자기 튀어나가지 않는다.
        """
        lim = self.limits
        v = _clamp(v_cmd, -lim.max_linear_mps, lim.max_linear_mps)
        w = _clamp(w_cmd, -lim.max_angular_rps, lim.max_angular_rps)

        dv = lim.max_linear_accel * dt
        dw = lim.max_angular_accel * dt
        v = _clamp(v, self.prev_v - dv, self.prev_v + dv)
        w = _clamp(w, self.prev_w - dw, self.prev_w + dw)

        if abs(v) < lim.min_linear_mps:
            v = 0.0
        if abs(w) < lim.min_angular_rps:
            w = 0.0

        self.prev_v, self.prev_w = v, w
        return v, w


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
