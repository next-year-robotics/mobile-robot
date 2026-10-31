"""배터리 잔량 추정(rclpy 없이). 표·상수는 battery.py 머리말."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mr_mission.battery import BatteryEstimator, soc_percent  # noqa: E402


def test_table_ends_and_interpolation():
    assert soc_percent(10.0) == 0 and soc_percent(14.0) == 100
    assert soc_percent(13.10) == 50
    assert abs(soc_percent(13.125) - 55) < 1e-9


def test_topic_voltage_gets_diode_drop_and_5_percent_steps():
    b = BatteryEstimator()
    assert b.percent(0.0) == -1
    b.update(0.0, 12.80)                                    # 단자 13.10 V = 50 %
    assert b.percent(0.0) == 50
    b.update(1.0, 0.0)                                      # MCU가 아직 못 잰 0 V는 버린다
    assert b.percent(1.0) == 50


def test_filter_and_stale():
    b = BatteryEstimator(tau_s=20.0, stale_s=5.0)
    b.update(0.0, 12.82)
    b.update(1.0, 12.0)                                     # 주행 중 순간 처짐은 20 s 필터로 거른다
    assert b.percent(1.0) >= 40
    assert b.percent(7.0) == -1                             # 5 s 넘게 안 오면 모른다
    b.update(8.0, 12.0)                                     # 끊긴 뒤 첫 값은 그대로 쓴다
    assert b.percent(8.0) == 10
