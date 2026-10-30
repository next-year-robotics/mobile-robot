"""mission 상태 기계(rclpy 없이)."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mr_mission.machine import (MSG_ENABLE_FAILED, MSG_STOP_UNCONFIRMED, Mission,  # noqa: E402
                                MissionConfig)

ON, OFF = [('enable', True)], [('enable', False)]


class Run:
    """시각을 직접 넘기는 도우미. states에 거쳐 간 상태를 모은다."""

    def __init__(self, **cfg):
        self.m = Mission(MissionConfig(**cfg))
        self.t = 100.0
        self.states = ['idle']

    def _seen(self):
        if self.states[-1] != self.m.state:
            self.states.append(self.m.state)

    def do(self, name, *args):
        out = getattr(self.m, name)(self.t, *args)
        self._seen()
        return out

    def wait(self, dt, step=0.05, odom=None):
        actions = []
        for _ in range(int(round(dt / step))):
            self.t += step
            if odom is not None:
                self.m.odom(self.t, *odom)
            actions += self.m.tick(self.t)
            self._seen()
        return actions


def started():
    r = Run()
    assert r.do('start', 'trip-1', 'place_home', 'place_market') == ON
    assert r.m.state == 'starting'
    assert r.do('enable_result', True, True) == []
    assert r.m.state == 'active'
    return r


def test_happy_path_with_odom():
    r = started()
    assert r.do('cancel', 'trip-1') == OFF
    assert r.m.state == 'cancelling'
    r.do('enable_result', False, True)
    r.wait(0.2, odom=(0.3, 0.0))                   # 아직 움직인다
    assert r.m.state == 'cancelling'
    r.wait(0.4, odom=(0.0, 0.01))                  # 0.3 s 정지 유지
    assert r.m.state == 'cancelled' and r.m.trip_id == 'trip-1'
    r.wait(3.2, odom=(0.0, 0.0))
    assert r.m.state == 'idle' and r.m.trip_id is None
    assert r.states == ['idle', 'starting', 'active', 'cancelling', 'cancelled', 'idle']
    assert 'arrived' not in r.states


def test_stop_without_odom_uses_time_fallback():
    r = started()
    r.do('cancel', 'trip-1')
    r.do('enable_result', False, True)
    r.wait(0.9)
    assert r.m.state == 'cancelling'
    r.wait(0.2)
    assert r.m.state == 'cancelled'
    assert any('no /odom' in line for line in r.m.log)


def test_stop_not_confirmed_is_error():
    r = started()
    r.do('cancel', 'trip-1')
    r.do('enable_result', False, True)
    r.wait(5.2, odom=(0.2, 0.0))                  # 계속 움직인다
    assert r.m.state == 'error' and r.m.last_error == MSG_STOP_UNCONFIRMED
    # 다시 멈춤을 누르면 복구 경로
    assert r.do('cancel', 'trip-1') == OFF and r.m.state == 'cancelling'
    r.do('enable_result', False, True)
    r.wait(0.5, odom=(0.0, 0.0))
    assert r.m.state == 'cancelled'


def test_disable_failure_is_retried():
    r = started()
    r.do('cancel', 'trip-1')
    assert r.do('enable_result', False, False, 'timeout') == []
    assert r.wait(0.45) == []
    assert r.wait(0.1) == OFF                       # 0.5 s 뒤 재시도
    r.do('enable_result', False, True)
    r.wait(0.5, odom=(0.0, 0.0))
    assert r.m.state == 'cancelled'


def test_enable_failure_and_timeout():
    r = Run()
    r.do('start', 't', 'a', 'b')
    assert r.do('enable_result', True, False, 'no service') == OFF
    assert r.m.state == 'error' and r.m.last_error == MSG_ENABLE_FAILED
    r = Run(enable_timeout_s=3.0)
    r.do('start', 't', 'a', 'b')
    assert r.wait(2.9) == []
    assert r.wait(0.2) == OFF and r.m.state == 'error'


def test_late_enable_is_undone():
    r = Run()
    r.do('start', 't', 'a', 'b')
    assert r.do('cancel', 't') == OFF               # starting 중 취소
    assert r.do('enable_result', True, True) == OFF  # 늦게 켜짐 → 다시 끈다
    assert r.m.state == 'cancelling'


def test_cancel_is_never_refused():
    r = Run()
    assert r.do('cancel', 'whatever') == OFF and r.m.state == 'idle'
    r = started()
    assert r.do('cancel', 'other-trip') == OFF and r.m.state == 'cancelling'
    assert r.do('cancel', 'trip-1') == []           # 이미 멈추는 중


def test_start_ignored_during_trip_and_allowed_after():
    r = started()
    assert r.do('start', 'trip-2', 'a', 'b') == [] and r.m.trip_id == 'trip-1'
    r.do('cancel', 'trip-1')
    r.do('enable_result', False, True)
    r.wait(1.2)
    assert r.m.state == 'cancelled'
    assert r.do('start', 'trip-2', 'a', 'b') == ON and r.m.trip_id == 'trip-2'


@pytest.mark.parametrize('state_setup,marker,expected', [
    ('idle', None, 'unavailable'),
    ('idle', 1, 'idle'),                       # 2026-09-27: 주행 밖에서는 마커가 보여도 following이 아니다
    ('idle', 0, 'idle'),
    ('starting', 0, 'searching'),
    ('starting', 1, 'following'),
    ('active_never', 0, 'searching'),
    ('active_acquired', 0, 'lost'),
    ('active_acquired', 1, 'following'),
    ('active_acquired', 2, 'following'),
    ('cancelling', 1, 'idle'),
    ('error', 1, 'idle'),
])
def test_tracking_state(state_setup, marker, expected):
    r = Run()
    if state_setup != 'idle':
        r.do('start', 't', 'a', 'b')
    if state_setup.startswith('active') or state_setup == 'cancelling':
        r.do('enable_result', True, True)
    if state_setup == 'cancelling':
        r.do('cancel', 't')
    if state_setup == 'error':
        r.do('enable_result', True, False, 'no follow')
    if state_setup == 'active_acquired':
        r.m.marker(r.t, 1)
    if marker is not None:
        r.m.marker(r.t, marker)
    assert r.m.tracking_state(r.t) == expected
    if marker is not None:
        assert r.m.tracking_state(r.t + 1.5) == 'unavailable'


# ------------------------------------------------------------ 안전 정지(M3 D4)

def started(**cfg):
    r = Run(**cfg)
    r.do('start', 'trip-1', 'place_home', 'place_market')
    r.do('enable_result', True, True)
    assert r.m.state == 'active'
    return r


def test_abort_stops_like_cancel_and_ends_in_error():
    r = started()
    assert r.do('abort', '어르신 폰이 응답하지 않아 멈췄어요') == OFF
    assert r.m.state == 'cancelling'
    r.do('enable_result', False, True)
    r.wait(0.5, odom=(0.0, 0.0))
    assert r.m.state == 'error' and r.m.last_error == '어르신 폰이 응답하지 않아 멈췄어요'
    assert r.states == ['idle', 'starting', 'active', 'cancelling', 'error']
    r.do('marker', 1)                                     # 마커가 다시 보여도
    assert r.wait(5.0, odom=(0.0, 0.0)) == [] and r.m.state == 'error'      # 다시 켜지 않는다
    assert r.do('start', 'trip-2', 'place_home', 'place_market') == ON        # 새 출발은 된다


def test_abort_outside_trip_does_nothing():
    r = Run()
    assert r.do('abort', 'x') == [] and r.m.state == 'idle'
    r = started()
    r.do('cancel', 'trip-1')
    assert r.do('abort', 'x') == [] and r.m.state == 'cancelling'


def test_user_cancel_during_abort_ends_cancelled():
    r = started()
    r.do('abort', 'x')
    assert r.do('cancel', 'trip-1') == []                 # disable은 이미 나갔다(중복 요청 안 함)
    r.do('enable_result', False, True)
    r.wait(0.5, odom=(0.0, 0.0))
    assert r.m.state == 'cancelled' and r.m.last_error is None


def test_abort_stop_unconfirmed_reports_that():
    r = started()
    r.do('abort', 'x')
    r.do('enable_result', False, True)
    r.wait(5.5, odom=(0.3, 0.0))                          # 계속 움직인다
    assert r.m.state == 'error' and r.m.last_error == MSG_STOP_UNCONFIRMED
