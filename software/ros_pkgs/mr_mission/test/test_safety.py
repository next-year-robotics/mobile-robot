"""안전 판단 상태 기계(rclpy 없이). 규칙: docs/senior-care/m3-safety-design.md D1~D5(SOS), D6(앱 연결 끊김)."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mr_mission.safety import (MARKER_HOLDING, MARKER_LOST, MARKER_TRACKING,  # noqa: E402
                               MSG_STOPPED_LINK_LOST, MSG_STOPPED_NO_ACK, LinkConfig, LinkWatch, SafetyConfig,
                               SafetyMonitor)

TRIP = 'bbef019a-dd3a-437e-8f5c-388913ae9eca'


class Run:
    """10 Hz marker_track을 흉내 낸다. marker=None이면 marker_track이 끊긴 것."""

    def __init__(self, **cfg):
        ids = iter(f'sos-{i}' for i in range(1, 100))
        self.s = SafetyMonitor(SafetyConfig(**cfg), new_id=lambda: next(ids))
        self.t = 100.0
        self.out: list = []

    def do(self, name, *args):
        r = getattr(self.s, name)(self.t, *args)
        self.out += r
        return r

    def run(self, dt, marker=MARKER_TRACKING, step=0.1):
        got = []
        for _ in range(int(round(dt / step))):
            self.t += step
            got += self.s.marker(self.t, marker) if marker is not None else self.s.tick(self.t)
        self.out += got
        return got

    def active(self, trip=TRIP):
        self.do('mission', 'starting', trip)
        self.do('mission', 'active', trip)


def kinds(actions):
    return [a[0] for a in actions]


def test_sos_after_lost_for_10s_while_active():
    r = Run()
    r.active()
    r.run(2.0)                                            # 잡았다
    assert r.run(9.9, MARKER_LOST) == []
    got = r.run(0.2, MARKER_LOST)
    assert got == [('sos_request', 'sos-1', 'tracking_lost', 'critical', TRIP)]
    assert r.run(30.0, MARKER_LOST) == []                 # 한 번만


def test_holding_counts_as_seen():
    r = Run()
    r.active()
    r.run(1.0)
    r.run(1.0, MARKER_HOLDING)                            # marker_track이 버티는 중
    assert r.run(9.5, MARKER_LOST) == []
    assert kinds(r.run(0.6, MARKER_LOST)) == ['sos_request']


def test_marker_track_silence_counts_as_lost():
    r = Run()
    r.active()
    r.run(1.0)
    assert kinds(r.run(10.2, marker=None)) == ['sos_request']


@pytest.mark.parametrize('state', ['idle', 'cancelling', 'cancelled', 'error', 'arrived'])
def test_no_sos_outside_active_trip(state):
    r = Run()
    r.active()
    r.run(1.0)
    r.do('mission', state, TRIP if state != 'idle' else None)
    assert r.run(30.0, MARKER_LOST) == []


def test_no_sos_while_searching_before_first_acquisition():
    r = Run()
    r.run(5.0)                                            # idle에서 본 것은 이번 trip의 획득이 아니다
    r.active()
    assert r.run(60.0, MARKER_LOST) == []
    r.run(0.1)                                            # 처음 잡음
    assert kinds(r.run(10.1, MARKER_LOST)) == ['sos_request']


def test_new_trip_resets_acquisition():
    r = Run()
    r.active()
    r.run(1.0)
    r.do('mission', 'cancelled', TRIP)
    r.do('mission', 'idle', None)
    r.active('11111111-2222-4333-8444-555555555555')
    assert r.run(30.0, MARKER_LOST) == []


def test_recover_5s_clears_then_rearms_immediately():
    r = Run()
    r.active()
    r.run(1.0)
    r.run(10.1, MARKER_LOST)
    assert r.s.sos_id == 'sos-1'
    assert r.run(4.8) == []                               # 아직 5 s 안 됨(첫 표본부터 4.7 s)
    assert r.run(0.4) == [('sos_clear', 'sos-1', 'tracking_recovered')]
    assert r.s.sos_id is None
    assert r.run(9.8, MARKER_LOST) == []                  # D2: 다시 10 s 필요
    assert r.run(0.3, MARKER_LOST) == [('sos_request', 'sos-2', 'tracking_lost', 'critical', TRIP)]


def test_flicker_does_not_clear():
    r = Run()
    r.active()
    r.run(1.0)
    r.run(10.1, MARKER_LOST)
    for _ in range(5):                                    # 3 s 보이고 1 s 놓침을 반복
        assert r.run(3.0) == []
        assert r.run(1.0, MARKER_LOST) == []
    assert r.s.sos_id == 'sos-1'
    r.run(3.0, marker=None)                               # 회복 중 marker_track 끊김도 연속이 깨진다
    assert r.run(4.8) == [] and kinds(r.run(0.4)) == ['sos_clear']


def test_recovery_clears_even_after_trip_ended():
    r = Run()
    r.active()
    r.run(1.0)
    r.run(10.1, MARKER_LOST)
    r.do('mission', 'error', TRIP)
    assert r.run(5.3)[-1] == ('sos_clear', 'sos-1', 'tracking_recovered')


def test_app_cancel_or_start_clears():
    r = Run()
    r.active()
    r.run(1.0)
    r.run(10.1, MARKER_LOST)
    assert r.do('trip_command', 'cancel') == [('sos_clear', 'sos-1', 'cancelled')]
    assert r.run(20.0, MARKER_LOST) == []                 # mission이 아직 active여도 곧바로 다시 내지 않는다
    r.run(0.1)
    r.run(10.1, MARKER_LOST)
    assert r.do('trip_command', 'start') == [('sos_clear', 'sos-2', 'handled')]
    assert r.do('trip_command', 'start') == []            # SOS가 없으면 아무것도 안 한다


def test_no_ack_does_nothing_until_decided():
    """OPEN-R3 결정 전 기본값: 아무것도 안 한다(현재 동작)."""
    r = Run()
    r.active()
    r.run(1.0)
    r.run(10.1, MARKER_LOST)
    assert r.do('sos_ack', 'sos-1', False) == []
    assert 'did not acknowledge' in r.s.log[-1]


def test_no_ack_stops_once_when_enabled():
    r = Run(stop_on_no_ack=True)
    r.active()
    r.run(1.0)
    r.run(10.1, MARKER_LOST)
    assert r.do('sos_ack', 'sos-1', True) == []           # 받았으면 안 세운다
    assert r.do('sos_ack', 'other', False) == []          # 다른 SOS
    assert r.do('sos_ack', 'sos-1', False) == [('stop', MSG_STOPPED_NO_ACK)]
    assert r.do('sos_ack', 'sos-1', False) == []          # 한 번만
    r.do('mission', 'error', TRIP)
    r.run(5.3)                                            # 회복해서 해제돼도 다시 움직이게 하지 않는다(= 출력 없음)
    assert kinds(r.out) == ['sos_request', 'stop', 'sos_clear']


def test_no_ack_outside_trip_does_not_stop():
    r = Run(stop_on_no_ack=True)
    r.active()
    r.run(1.0)
    r.run(10.1, MARKER_LOST)
    r.do('mission', 'cancelled', TRIP)
    assert r.do('sos_ack', 'sos-1', False) == []


def test_lost_for():
    r = Run()
    assert r.s.lost_for(r.t) == 0.0
    r.active()
    r.run(1.0)
    r.run(3.0, MARKER_LOST)
    assert r.s.lost_for(r.t) == pytest.approx(3.0, abs=0.11)


# ============================================================ D6: 앱 연결 끊김


class Link:
    """브리지 흉내: 연결 중에는 5 s마다 connected=true를 재발행한다. mission tick은 0.1 s."""

    def __init__(self, t=100.0, **cfg):
        self.t = t
        self.w = LinkWatch(LinkConfig(**cfg), now=t)
        self.up = False
        self.bridge_alive = True
        self._last_pub = -1e9
        self.out: list = []

    def set(self, connected):
        self.up = connected
        self._last_pub = self.t
        self.out += self.w.link(self.t, connected)

    def trip(self, state, trip=TRIP):
        self.out += self.w.mission(self.t, state, trip)

    def run(self, dt, step=0.1):
        got = []
        for _ in range(int(round(dt / step))):
            self.t += step
            if self.up and self.bridge_alive and self.t - self._last_pub >= 5.0:
                self._last_pub = self.t
                got += self.w.link(self.t, True)
            got += self.w.tick(self.t)
        self.out += got
        return got


def start_connected(**cfg):
    k = Link(**cfg)
    k.set(True)
    k.run(1.0)
    k.trip('starting')
    k.trip('active')
    return k


def test_link_lost_30s_during_trip_stops_once():
    k = start_connected()
    k.run(10.0)
    k.set(False)
    assert k.run(29.9) == []
    assert 'stopping in 30 s' in k.w.log[0]
    assert k.run(0.2) == [('stop', MSG_STOPPED_LINK_LOST)]
    k.trip('cancelling')
    k.trip('error')
    assert k.run(60.0) == [] and kinds(k.out) == ['stop']
    k.set(True)                                            # 다시 붙어도 아무것도 안 한다(자동 재개는 mission 몫이 아니고, 없다)
    assert k.run(5.0) == []


def test_link_back_within_grace_keeps_going_and_restarts_timer():
    k = start_connected()
    k.set(False)
    k.run(20.0)
    k.set(True)
    assert k.run(1.0) == []
    assert 'restored' in k.w.log[-1]
    k.set(False)                                           # 다시 끊김: 처음부터 30 s
    assert k.run(29.9) == []
    assert k.run(0.2) == [('stop', MSG_STOPPED_LINK_LOST)]


def test_link_stale_counts_as_lost():
    """브리지 프로세스가 죽으면 latched connected=true가 남는다. 12 s 넘게 재발행이 없으면 끊김."""
    k = start_connected()
    k.bridge_alive = False
    last = k._last_pub
    k.run(12.0 + 30.0 - (k.t - last) - 0.2)
    assert kinds(k.out) == []
    k.run(0.4)
    assert kinds(k.out) == ['stop']
    assert k.w.down_since(k.t) == pytest.approx(last + 12.0)


def test_link_never_seen_counts_from_trip_start():
    k = Link(t=100.0)
    k.run(300.0)                                           # 오래 전부터 브리지 없음(CLI 출발 같은 경우)
    k.trip('starting')
    k.trip('active')
    assert k.run(29.9) == []
    assert k.run(0.2) == [('stop', MSG_STOPPED_LINK_LOST)]


def test_link_loss_before_trip_counts_from_trip_start():
    k = Link()
    k.set(True)
    k.run(1.0)
    k.set(False)
    k.run(100.0)
    k.trip('starting')
    k.trip('active')
    assert k.w.down_for(k.t) == 0.0
    assert k.run(29.9) == []
    assert k.run(0.2) == [('stop', MSG_STOPPED_LINK_LOST)]


@pytest.mark.parametrize('state', ['idle', 'cancelling', 'cancelled', 'error', 'arrived'])
def test_link_loss_outside_moving_trip_does_nothing(state):
    k = Link()
    k.set(True)
    k.trip(state)
    k.set(False)
    assert k.run(60.0) == [] and k.w.log == []


def test_link_loss_during_starting_counts():
    k = start_connected()
    k.trip('starting', 'trip-2')                          # 새 trip(예: 방금 출발)
    k.set(False)
    assert k.run(29.9) == []
    assert k.run(0.2) == [('stop', MSG_STOPPED_LINK_LOST)]


def test_new_trip_after_stop_gets_its_own_timer_even_with_same_id():
    k = start_connected()
    k.set(False)
    k.run(30.1)
    k.trip('error')
    k.trip('starting')                                     # 같은 trip_id로 다시 출발(CLI 시험)
    k.trip('active')
    assert k.run(29.9) == []
    assert k.run(0.2) == [('stop', MSG_STOPPED_LINK_LOST)]


def test_link_watch_disabled_with_zero():
    k = Link(stop_after_s=0.0)
    k.trip('starting')
    k.trip('active')
    assert k.run(100.0) == []


def test_link_down_for():
    k = start_connected()
    assert k.w.down_for(k.t) == 0.0
    k.set(False)
    k.run(7.0)
    assert k.w.down_for(k.t) == pytest.approx(7.0, abs=0.11)


# ============================================================ D7: 앱 SOS 버튼

MANUAL = '0b9a4c1e-7d2f-4e55-9a31-6c8e2f4d7b90'


def test_manual_sos_opens_outside_trip_and_is_not_cleared_by_marker():
    r = Run()
    assert r.do('manual', MANUAL) == [('sos_request', MANUAL, 'manual', 'critical', None)]
    assert r.run(10.0) == []                               # 마커를 계속 봐도 tracking_recovered로 닫지 않는다
    assert r.do('manual', 'another') == []                 # 열린 SOS가 있으면 새로 열지 않는다
    assert r.do('cancel', 'other-id') == []
    assert r.do('cancel', MANUAL) == [('sos_clear', MANUAL, 'cancelled')]
    assert r.s.sos_id is None and r.s.sos_reason is None


def test_manual_sos_during_trip_keeps_trip_and_app_trip_command_clears():
    r = Run()
    r.active()
    assert kinds(r.do('manual', MANUAL)) == ['sos_request']
    assert r.out[-1][4] == TRIP
    assert r.do('trip_command', 'cancel') == [('sos_clear', MANUAL, 'cancelled')]


def test_cancel_closes_tracking_lost_sos_and_does_not_reopen_at_once():
    r = Run()
    r.active()
    r.run(1.0)
    r.run(10.1, MARKER_LOST)
    assert r.do('cancel', 'sos-1') == [('sos_clear', 'sos-1', 'cancelled')]
    assert r.run(20.0, MARKER_LOST) == []
