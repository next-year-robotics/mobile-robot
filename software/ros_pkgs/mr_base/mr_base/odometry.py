"""ROS 비의존 차동구동 exact-arc 오도메트리 적분기.

`/joint_states.position`(누적 바퀴 각도 [rad])의 차분만 먹는다. `velocity`는 쓰지 않는다.
BEST_EFFORT 표본이 하나 빠져도 다음 절대 position 차분에 빠진 구간의 이동이 그대로
들어있기 때문이다.

이 모듈에는 rclpy import가 없다. 운동학과 이상 표본 판정을 ROS 없이 시험할 수 있어야
`odom_node.py`의 배선 버그와 적분 버그를 구분할 수 있다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

# 모터가 낼 수 있는 바퀴 각속도(약 19 rad/s)에 여유를 얹어 "물리적으로 불가능"의
# 문턱으로 쓴다. 운용 상한이 아니라 물리 상한 기준인 이유는 사람이 손으로 바퀴를
# 돌리는 경우까지 이 아래에 들어와야 하기 때문이다. 정상 표본을 이상으로 버리는 쪽이
# 훨씬 나쁘다.
MAX_WHEEL_SPEED_RAD_S = 25.0

# dt가 아주 작을 때 문턱이 양자화 아래로 내려가지 않게 하는 바닥값 [rad].
# 틱 하나가 2π/2475.5 = 2.54 mrad이므로 약 4틱이다.
MIN_JUMP_ALLOWANCE_RAD = 0.01

# 이보다 작은 |dtheta|에서는 R = ds/dtheta가 수치적으로 의미를 잃는다. 직선 극한을 쓴다.
STRAIGHT_DTHETA_RAD = 1e-9


class RebaseReason:
    """pose에 적분하지 않고 기준점만 다시 잡은 이유."""

    FIRST_SAMPLE = 'first_sample'
    MISSING_JOINT = 'missing_joint'
    NON_FINITE = 'non_finite'
    STAMP_NOT_ADVANCING = 'stamp_not_advancing'
    WHEEL_JUMP = 'wheel_jump'
    MCU_RESET = 'mcu_reset'

    ALL = (
        FIRST_SAMPLE,
        MISSING_JOINT,
        NON_FINITE,
        STAMP_NOT_ADVANCING,
        WHEEL_JUMP,
        MCU_RESET,
    )


@dataclass
class OdometryCounters:
    """진단용 계수기. 이상 표본과 재기준 횟수를 남긴다."""

    samples: int = 0
    integrated: int = 0
    rebases: dict = field(default_factory=lambda: {r: 0 for r in RebaseReason.ALL})

    @property
    def rebase_total(self) -> int:
        return sum(self.rebases.values())

    def summary(self) -> str:
        used = ' '.join(f'{k}={v}' for k, v in self.rebases.items() if v)
        return (f'samples={self.samples} integrated={self.integrated} '
                f'rebase={self.rebase_total}' + (f' [{used}]' if used else ''))


@dataclass(frozen=True)
class OdometryUpdate:
    """한 표본을 적분한 결과. pose는 누적, twist는 이 구간 평균이다."""

    stamp_s: float
    x: float
    y: float
    yaw: float
    v: float
    w: float
    dt: float
    ds: float
    dtheta: float


def normalize_angle(angle: float) -> float:
    """(-pi, pi]로 감는다."""
    wrapped = math.atan2(math.sin(angle), math.cos(angle))
    # atan2는 -pi를 그대로 돌려준다. 경계를 한쪽으로 몰아 비교를 단순하게 한다.
    return math.pi if wrapped == -math.pi else wrapped


class DiffDriveOdometry:
    """절대 바퀴 각도 차분을 exact-arc로 적분한다.

    좌표·부호 규약은 REP-103이다. `+x` 전진, `+yaw` 반시계.
    `/joint_states.position`의 부호는 펌웨어가 이미 맞춰 두었으므로 여기에 좌우 sign
    파라미터는 없다.
    """

    def __init__(
        self,
        wheel_radius_m: float,
        wheel_separation_m: float,
        left_joint: str = 'left_wheel_joint',
        right_joint: str = 'right_wheel_joint',
        max_wheel_speed_rad_s: float = MAX_WHEEL_SPEED_RAD_S,
        min_jump_allowance_rad: float = MIN_JUMP_ALLOWANCE_RAD,
    ) -> None:
        if not (wheel_radius_m > 0.0 and math.isfinite(wheel_radius_m)):
            raise ValueError(f'wheel_radius_m must be positive: {wheel_radius_m}')
        if not (wheel_separation_m > 0.0 and math.isfinite(wheel_separation_m)):
            raise ValueError(f'wheel_separation_m must be positive: {wheel_separation_m}')

        self.wheel_radius_m = wheel_radius_m
        self.wheel_separation_m = wheel_separation_m
        self.left_joint = left_joint
        self.right_joint = right_joint
        self.max_wheel_speed_rad_s = max_wheel_speed_rad_s
        self.min_jump_allowance_rad = min_jump_allowance_rad

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self._prev_stamp_s: Optional[float] = None
        self._prev_phi_l = 0.0
        self._prev_phi_r = 0.0
        self._prev_loop_count: Optional[int] = None
        self._pending_rebase: Optional[str] = None
        self.last_rebase_reason: Optional[str] = None

        self.counters = OdometryCounters()

    # ------------------------------------------------------------------ 입력

    def note_loop_count(self, loop_count: int) -> bool:
        """`/base/status.loop_count`로 MCU reset을 감지한다.

        역행이면 다음 `/joint_states` 표본을 적분하지 않고 기준점으로만 쓴다. reset 뒤
        position이 0으로 돌아온 것을 이동으로 적분하면 odom pose가 순간 점프한다.
        """
        reset = self._prev_loop_count is not None and loop_count < self._prev_loop_count
        self._prev_loop_count = loop_count
        if reset:
            self._pending_rebase = RebaseReason.MCU_RESET
        return reset

    def update_from_joint_state(
        self,
        stamp_s: float,
        names: Sequence[str],
        positions: Sequence[float],
    ) -> Optional[OdometryUpdate]:
        """이름으로 left/right를 찾는다. 배열의 물리적 순서를 믿지 않는다."""
        phi_l = _lookup(names, positions, self.left_joint)
        phi_r = _lookup(names, positions, self.right_joint)
        if phi_l is None or phi_r is None:
            self.counters.samples += 1
            self._rebase(RebaseReason.MISSING_JOINT, None, 0.0, 0.0)
            return None
        return self.update(stamp_s, phi_l, phi_r)

    def update(self, stamp_s: float, phi_l: float, phi_r: float) -> Optional[OdometryUpdate]:
        """한 표본을 처리한다. 적분했으면 결과를, 재기준했으면 None을 돌려준다."""
        self.counters.samples += 1

        if not all(math.isfinite(v) for v in (stamp_s, phi_l, phi_r)):
            # 좌표를 오염시키지 않도록 NaN/Inf는 기준점으로도 쓰지 않는다.
            self._rebase(RebaseReason.NON_FINITE, None, 0.0, 0.0)
            return None

        if self._pending_rebase is not None:
            self._rebase(self._pending_rebase, stamp_s, phi_l, phi_r)
            return None

        if self._prev_stamp_s is None:
            self._rebase(RebaseReason.FIRST_SAMPLE, stamp_s, phi_l, phi_r)
            return None

        dt = stamp_s - self._prev_stamp_s
        if dt <= 0.0:
            # 같거나 과거인 stamp. 새 시간 기준으로 갈아탄다.
            self._rebase(RebaseReason.STAMP_NOT_ADVANCING, stamp_s, phi_l, phi_r)
            return None

        dphi_l = phi_l - self._prev_phi_l
        dphi_r = phi_r - self._prev_phi_r

        # 드롭이나 긴 callback 간격만으로 기준점을 버리지 않는다. 문턱을 dt에
        # 비례시키면 빠진 구간이 길수록 허용 이동도 같이 커진다. 그래야 절대 position
        # 차분이 이동을 복구할 수 있다는 성질이 유지된다.
        allowance = max(self.max_wheel_speed_rad_s * dt, self.min_jump_allowance_rad)
        if abs(dphi_l) > allowance or abs(dphi_r) > allowance:
            self._rebase(RebaseReason.WHEEL_JUMP, stamp_s, phi_l, phi_r)
            return None

        dl = self.wheel_radius_m * dphi_l
        dr = self.wheel_radius_m * dphi_r
        ds = 0.5 * (dr + dl)
        dtheta = (dr - dl) / self.wheel_separation_m

        if abs(dtheta) < STRAIGHT_DTHETA_RAD:
            dx_body = ds
            dy_body = 0.5 * ds * dtheta      # 1차 항. 문턱에서 호 해와 연속이다.
        else:
            radius = ds / dtheta
            dx_body = radius * math.sin(dtheta)
            dy_body = radius * (1.0 - math.cos(dtheta))

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += dx_body * cos_yaw - dy_body * sin_yaw
        self.y += dx_body * sin_yaw + dy_body * cos_yaw
        self.yaw = normalize_angle(self.yaw + dtheta)

        self._prev_stamp_s = stamp_s
        self._prev_phi_l = phi_l
        self._prev_phi_r = phi_r
        self.counters.integrated += 1

        return OdometryUpdate(
            stamp_s=stamp_s,
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            v=ds / dt,
            w=dtheta / dt,
            dt=dt,
            ds=ds,
            dtheta=dtheta,
        )

    # ------------------------------------------------------------------ 내부

    def _rebase(
        self,
        reason: str,
        stamp_s: Optional[float],
        phi_l: float,
        phi_r: float,
    ) -> None:
        """pose는 그대로 두고 차분 기준점만 옮긴다. pose 점프가 생기지 않는 지점이다."""
        self.counters.rebases[reason] += 1
        self.last_rebase_reason = reason
        if stamp_s is None:
            # 쓸 수 없는 표본이다. 기준점을 이걸로 바꾸면 다음 차분이 오염된다.
            # 예약된 재기준(MCU reset)도 아직 소비하지 않은 채로 남긴다.
            return
        self._pending_rebase = None
        self._prev_stamp_s = stamp_s
        self._prev_phi_l = phi_l
        self._prev_phi_r = phi_r


def _lookup(names: Sequence[str], positions: Sequence[float], joint: str) -> Optional[float]:
    try:
        index = list(names).index(joint)
    except ValueError:
        return None
    if index >= len(positions):
        return None
    return float(positions[index])
