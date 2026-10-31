"""배터리 잔량 추정. rclpy를 import하지 않는다.

/base/status supply_voltage는 역극성 보호 다이오드(SR540) 뒤에서 재서 배터리 단자보다 낮다(10-31 실측 0.27~0.29 V).
단자 전압 = 토픽 + diode_drop_v로 4S LiFePO4 휴지 전압표에서 찾는다. 표와 상수는 부스 대시보드
(mobile-robot-booth config/booth.yaml battery, 사용자 아티팩트 "배터리 잔량표" 2026-10-30)와 같다.
30~80 %가 0.3 V 안에 몰려 있어 1 % 단위는 거짓 정밀도다. 5 % 단위로 낸다. 주행 중에는 처져서 낮게 나온다.
"""
from __future__ import annotations

import math

SOC_TABLE = ((11.20, 0), (12.00, 5), (12.40, 10), (12.85, 20), (13.00, 30), (13.05, 40), (13.10, 50),
             (13.15, 60), (13.20, 70), (13.28, 80), (13.36, 90), (13.42, 95), (13.60, 100))


def soc_percent(terminal_v: float, table=SOC_TABLE) -> float:
    if terminal_v <= table[0][0]:
        return float(table[0][1])
    for (v0, s0), (v1, s1) in zip(table, table[1:]):
        if terminal_v <= v1:
            return s0 + (s1 - s0) * (terminal_v - v0) / (v1 - v0)
    return float(table[-1][1])


class BatteryEstimator:
    """1차 저역 통과(tau_s) 뒤 표에서 잔량을 찾는다. stale_s 동안 전압이 안 오면 모른다(-1)."""

    def __init__(self, *, diode_drop_v: float = 0.30, tau_s: float = 20.0, stale_s: float = 5.0):
        self.diode_drop_v = diode_drop_v
        self.tau_s = tau_s
        self.stale_s = stale_s
        self._v: float | None = None
        self._at: float | None = None

    def update(self, now: float, topic_v: float) -> None:
        if not math.isfinite(topic_v) or topic_v <= 1.0:      # 0 V = MCU가 아직 못 쟀다
            return
        if self._v is None or self._at is None or now - self._at > self.stale_s:
            self._v = topic_v
        else:
            a = 1.0 - math.exp(-max(now - self._at, 0.0) / self.tau_s) if self.tau_s > 0 else 1.0
            self._v += a * (topic_v - self._v)
        self._at = now

    def percent(self, now: float) -> int:
        """0..100(5 % 단위), 모르면 -1(RobotAppStatus.battery_percent 규칙)."""
        if self._v is None or self._at is None or now - self._at > self.stale_s:
            return -1
        return int(5 * round(soc_percent(self._v + self.diode_drop_v) / 5))
