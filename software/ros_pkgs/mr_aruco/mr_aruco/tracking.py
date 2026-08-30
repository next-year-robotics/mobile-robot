"""ROS 비의존 마커 추적 필터 — 짧은 검출 유실을 버티고 위치 점프를 기각한다.

`aruco_opencv`의 원시 검출을 그대로 제어기에 물리면 두 가지가 문제가 된다.

1. **한 프레임만 놓쳐도 값이 끊긴다.** 사람이 걸으면 마커가 기울고 모션블러가
   끼면서 검출이 산발적으로 빠진다. 그때마다 정지하면 주행이 뚝뚝 끊긴다.
2. **오검출이 그대로 지령이 된다.** 사분면이 뒤집힌 PnP 해나 엉뚱한 반사면이
   한 프레임 잡히면 로봇이 그쪽으로 튄다.

그래서 상태기계를 하나 둔다. `maknae/mm_perception/confidence_lock.py`를 옮겨온
것이되 **신뢰도 축은 뺐다** — 그쪽은 자체 검출기가 재투영 오차로 신뢰도를 만들었지만
`aruco_opencv`는 id와 pose만 주고 품질 지표를 주지 않는다. 없는 신호를 흉내 내는
대신 "검출됐다 / 안 됐다"의 이진 입력으로 다시 짰다.

    UNLOCKED ──(min_stable_s 연속 검출)──> TRACKING
    TRACKING ──(검출 없음)──> HOLDING ──(max_hold_s 초과)──> LOST → UNLOCKED
    TRACKING ──(점프 reject_streak_unlock 회 연속)──> UNLOCKED

이 모듈에는 rclpy import가 없다. 상태 전이를 시계 없이 시험할 수 있어야
`marker_track_node.py`의 배선 버그와 필터 버그를 구분할 수 있다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Optional

STATUS_LOST = 'LOST'
STATUS_TRACKING = 'TRACKING'
STATUS_HOLDING = 'HOLDING'


@dataclass(frozen=True)
class PlanarPose:
    """본체 좌표계(`base_footprint`) 기준 평면 자세."""

    x: float
    y: float
    yaw: float

    @property
    def range_m(self) -> float:
        """지면상 거리 ρ."""
        return math.hypot(self.x, self.y)

    @property
    def bearing_rad(self) -> float:
        """마커 방향 α [rad], + = 왼쪽.

        `atan(y/x)`가 아니라 `atan2`인 이유는 두 가지다. x = 0에서 0으로 나누지
        않아야 하고, 뒤쪽(x < 0)이 앞쪽과 같은 값으로 접히면 안 된다.
        Leo Rover 예제가 `atan(y/x)`를 써서 두 문제를 다 갖고 있다.
        """
        return math.atan2(self.y, self.x)


def scale_point_from_origin(
        point: tuple[float, float, float],
        origin: tuple[float, float, float],
        scale: float,
) -> tuple[float, float, float]:
    """카메라 원점을 보존하며 PnP 병진벡터의 스케일만 보정한다.

    검출기가 이미 ``base_footprint``로 변환한 위치에 단순히 계수를
    곱하면 카메라–베이스 병진까지 축소된다. 따라서 먼저 카메라
    광학 중심을 빼고 카메라 상대 벡터만 보정한 뒤 원점을 다시 더한다.
    """
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError('PnP translation scale must be finite and positive')
    if len(point) != 3 or len(origin) != 3:
        raise ValueError('point and origin must contain exactly three coordinates')
    return tuple(o + scale * (p - o) for p, o in zip(point, origin))


@dataclass
class TrackConfig:
    # 락이 걸리기까지 필요한 연속 검출 시간. 짧으면 오검출에 락이 걸리고,
    # 길면 사람이 시야에 들어온 뒤 출발이 늦다.
    min_stable_s: float = 0.3
    # 검출이 끊긴 뒤 직전 값을 유지하는 상한. 이 값을 넘기면 LOST다.
    # 제어기의 marker_timeout(0.4 s)보다 길게 두어야 hold가 의미를 갖는다.
    max_hold_s: float = 1.0
    # 한 표본이 직전 유지값에서 이만큼 넘게 튀면 기각한다. 사람 보행 속도
    # 1.5 m/s에 30 Hz면 프레임당 5 cm이므로 10 cm는 정상 이동의 두 배다.
    max_jump_pos_m: float = 0.10
    max_jump_yaw_rad: float = 0.35
    # 연속 기각이 이만큼 쌓이면 "대상이 진짜로 옮겨간 것"으로 보고 락을 푼다.
    # 이게 없으면 한 번 잘못 잡힌 기준점에 영원히 갇힌다.
    reject_streak_unlock: int = 8


@dataclass(frozen=True)
class TrackOutput:
    pose: Optional[PlanarPose]
    status: str
    locked: bool
    hold_duration_s: float = 0.0
    rejected_jump: bool = False


@dataclass
class TrackCounters:
    """진단용 계수기."""

    samples: int = 0
    detections: int = 0
    rejected_jumps: int = 0
    holds: int = 0
    losses: int = 0

    def summary(self) -> str:
        return (f'samples={self.samples} det={self.detections} '
                f'jump_reject={self.rejected_jumps} hold={self.holds} lost={self.losses}')


@dataclass
class MarkerTracker:
    config: TrackConfig = field(default_factory=TrackConfig)

    def __post_init__(self) -> None:
        self.counters = TrackCounters()
        self.reset()

    def reset(self) -> None:
        """락과 유지값만 지운다. 계수기는 노드 수명 내내 누적한다."""
        self._locked = False
        self._stable_since: Optional[float] = None
        self._held: Optional[PlanarPose] = None
        self._last_seen: Optional[float] = None
        self._reject_streak = 0
        self._holding = False

    # ------------------------------------------------------------------ 갱신

    def update(self, pose: Optional[PlanarPose], now: float) -> TrackOutput:
        """검출 하나(또는 미검출)를 먹고 현재 추적 상태를 낸다.

        Args:
            pose: 이번 프레임에서 대상 마커가 잡혔으면 그 자세, 아니면 None.
            now: 단조 증가 시각 [s]. 노드는 검출 프레임의 stamp를 넣는다.
        """
        self.counters.samples += 1
        if pose is not None:
            self.counters.detections += 1

        if not self._locked:
            return self._update_unlocked(pose, now)

        if pose is None:
            return self._hold(now)

        assert self._held is not None
        if self._is_jump(pose, self._held):
            self.counters.rejected_jumps += 1
            self._reject_streak += 1
            if self._reject_streak >= self.config.reject_streak_unlock:
                # 대상이 실제로 옮겨갔다고 본다. 락을 풀고 이 표본부터 다시 잡는다.
                self.reset()
                return self._update_unlocked(pose, now)
            # 기각된 표본은 버리고 유지값을 그대로 낸다.
            return TrackOutput(pose=self._held, status=STATUS_TRACKING,
                               locked=True, rejected_jump=True)

        self._reject_streak = 0
        self._holding = False
        self._held = pose
        self._last_seen = now
        return TrackOutput(pose=pose, status=STATUS_TRACKING, locked=True)

    # -------------------------------------------------------------- 내부 전이

    def _update_unlocked(self, pose: Optional[PlanarPose], now: float) -> TrackOutput:
        if pose is None:
            self._stable_since = None
            return TrackOutput(pose=None, status=STATUS_LOST, locked=False)

        if self._stable_since is None:
            self._stable_since = now
        if now - self._stable_since >= self.config.min_stable_s:
            self._locked = True
            self._held = pose
            self._last_seen = now
            self._reject_streak = 0

        # 락 전에도 검출값은 그대로 흘려보낸다. 락을 기다리느라 출발이 늦는 쪽이
        # 나쁘다. 대신 이 구간에서는 점프 기각이 아직 작동하지 않는다.
        return TrackOutput(pose=pose, status=STATUS_TRACKING, locked=self._locked)

    def _hold(self, now: float) -> TrackOutput:
        held_since = self._last_seen if self._last_seen is not None else now
        hold_t = now - held_since
        if hold_t <= self.config.max_hold_s:
            if not self._holding:
                self._holding = True
                self.counters.holds += 1
            return TrackOutput(pose=self._held, status=STATUS_HOLDING,
                               locked=True, hold_duration_s=hold_t)
        self.counters.losses += 1
        self.reset()
        return TrackOutput(pose=None, status=STATUS_LOST, locked=False)

    def _is_jump(self, new: PlanarPose, held: PlanarPose) -> bool:
        cfg = self.config
        dpos = math.hypot(new.x - held.x, new.y - held.y)
        dyaw = abs(wrap_angle(new.yaw - held.yaw))
        return dpos > cfg.max_jump_pos_m or dyaw > cfg.max_jump_yaw_rad


def wrap_angle(angle: float) -> float:
    """각도를 [-π, π]로 접는다. ±π는 부동소수 잔차가 부호를 정한다."""
    return math.atan2(math.sin(angle), math.cos(angle))


def marker_normal_yaw(qx: float, qy: float, qz: float, qw: float) -> tuple:
    """마커 자세 쿼터니언에서 바깥 법선의 지면 투영 방향을 뽑는다.

    ArUco 마커 좌표계의 z축이 마커 면에서 바깥(카메라 쪽)으로 나온다. 그 축을
    본체 좌표계로 옮긴 것이 `R · [0, 0, 1]`, 즉 회전행렬의 세 번째 열이다.

    쿼터니언을 오일러로 풀어 yaw만 꺼내지 않는 이유는, 마커 좌표계가 본체
    좌표계에 대해 두 축이 90도 돌아가 있어 그 분해가 짐벌 부근에 놓이기
    때문이다. 세 번째 열만 쓰면 그 문제가 없다.

    Returns:
        (yaw, horizontal): 지면 투영 방향 [rad]과 그 투영 벡터의 크기.
        `horizontal`이 0에 가까우면 법선이 거의 수직이라는 뜻이고, 그때 yaw는
        의미가 없다. 마커를 눕혀 달았거나 자세 추정이 무너진 경우다.
    """
    nx = 2.0 * (qx * qz + qw * qy)
    ny = 2.0 * (qy * qz - qw * qx)
    return math.atan2(ny, nx), math.hypot(nx, ny)
