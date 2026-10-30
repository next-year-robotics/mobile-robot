"""텔레메트리 에이전트 코어. rclpy를 import하지 않는다(ROS 노드와 --sim이 같은 입력 메서드를 부른다).

입력(`on_*`)은 아무 스레드에서나 부른다. RTDB 쓰기는 전송 스레드 하나가 한다. 규칙(care-telemetry README 5장):
- status: 입력이 바뀌면 곧바로(최소 0.5 s 간격), 아니면 5 s마다 덮어쓴다. 실패하면 버린다(최신값만 의미).
- events·sos: SQLite 큐에 넣고 순서대로 보낸다. 실패하면 1, 2, 4 … 60 s 백오프. `ts_ms`와 push id는 만들 때 정한다.
- meta: 온라인이 되면 한 번. `first_seen_ms`는 이미 있으면 유지.
- events 보존: 시작 시와 하루 1회, 로봇당 최대 1000개만 남긴다.
- diag(publish_diag): 5 s마다 덮어쓴다. 토픽 이름은 RTDB 경로로 펼친다(`/base/status` → `topics/base/status`).
- RTDB가 400(키 규칙 위반 등)으로 거절한 쓰기는 로그 ERROR로 드러낸다(대상마다 처음 한 번, 그 뒤 5분마다 묶어서).
  diag.recent_errors에만 넣으면 diag 자체가 거절될 때 아무도 모른다.
- 이벤트에는 좌표를 넣지 않는다. 폰 위치는 SOS 중에만 `sos/{id}`에 쓰고 sos.clear에서 지운다.
"""
from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable

from . import __version__
from . import model as M
from .outbox import Outbox
from .rtdb import RtdbClient, RtdbError

log = logging.getLogger(__name__)

SOURCE_RANK = {'none': 0, 'cache': 1, 'fresh': 2}
SOS_REASONS = ('tracking_lost', 'stuck', 'low_battery', 'manual', 'unknown')
SOS_RESOLUTIONS = ('tracking_recovered', 'handled', 'cancelled', 'unknown')
MAX_LOCATION_UPDATES = 3
DIAG_TOPICS = ('/base/status', '/joint_states', '/marker_track', '/robot/app_status')


@dataclass
class AgentConfig:
    robot_id: str = 'mr-001'
    display_name: str = '돌봄 로봇 1호'
    version: str = __version__
    status_period_s: float = 5.0
    status_min_interval_s: float = 0.5
    app_status_stale_s: float = 6.0
    marker_stale_s: float = 1.0
    publish_sos_location: bool = True
    publish_diag: bool = False
    diag_period_s: float = 5.0
    max_events: int = 1000
    prune_period_s: float = 86400.0
    ack_debounce_s: float = 1.0
    backoff_max_s: float = 60.0
    error_log_repeat_s: float = 300.0
    loop_s: float = 0.2


def now_ms() -> int:
    return time.time_ns() // 1_000_000


class TelemetryAgent:

    def __init__(self, cfg: AgentConfig, client: RtdbClient, outbox: Outbox, *,
                 clock: Callable[[], float] = time.monotonic, wall_ms: Callable[[], int] = now_ms):
        self.cfg = cfg
        self.client = client
        self.outbox = outbox
        self.clock = clock
        self.wall_ms = wall_ms
        self.root = f'robots/{M.safe_key(cfg.robot_id)}'
        self.pushids = M.PushIds()
        self._lock = threading.RLock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.started_at = clock()
        # 입력 상태
        self.app: dict | None = None
        self.app_at = 0.0
        self.link: dict | None = None
        self.base: dict | None = None
        self.bridge_alive: bool | None = None
        self.marker: int | None = None
        self.marker_at = 0.0
        self.labels: dict[str, str] = {}
        self.trips: dict[str, dict] = {}
        self.cancel_reasons: dict[str, str] = {}
        self.sos: dict[str, dict] = {}
        self.sos_closed: deque[str] = deque(maxlen=64)
        self.topics: dict[str, dict] = {}
        self.errors: deque[dict] = deque(maxlen=10)
        # 전송 상태
        self.online: bool | None = None
        self._status_dirty = True
        self._last_status_at = -1e9
        self._last_diag_at = -1e9
        self._backoff_s = 0.0
        self._next_send_at = 0.0
        self._meta_done = False
        self._next_prune_at = 0.0
        self._rejected: dict[str, dict] = {}     # 영구 오류가 난 대상 → 로그 묶음 상태

    # ================================================================ 수명

    def start(self) -> None:
        self._emit('agent.online', 'info', M.MSG_AGENT_ONLINE, data={'version': self.cfg.version})
        self._thread = threading.Thread(target=self._run, name='care-telemetry-sender', daemon=True)
        self._thread.start()
        log.info('telemetry → %s robot=%s (queue %d)', self.client.target, self.cfg.robot_id, len(self.outbox))

    def stop(self, flush_timeout_s: float = 3.0) -> None:
        """agent.offline을 남기고 큐를 최대한 보낸 뒤 멈춘다. 못 보낸 것은 다음 기동 때 보낸다."""
        self._emit('agent.offline', 'info', M.MSG_AGENT_OFFLINE, data={'version': self.cfg.version})
        self._flush_deadline = self.clock() + flush_timeout_s
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(flush_timeout_s + 5.0)
        self.outbox.close()

    # ================================================================ 입력

    def on_app_status(self, *, mission_state: str, tracking_state: str, trip_id: str | None,
                      from_place_id: str | None, to_place_id: str | None, battery_percent: float | None,
                      last_error: str | None, age_s: float = 0.0) -> None:
        new = {'mission_state': mission_state if mission_state in M.MISSION_STATES else 'unknown',
               'tracking_state': tracking_state if tracking_state in M.TRACKING_STATES else 'unknown',
               'trip_id': trip_id or None, 'from_place_id': from_place_id or None,
               'to_place_id': to_place_id or None, 'battery_percent': M.finite_or_none(battery_percent),
               'last_error': last_error or None}
        with self._lock:
            prev, self.app = self.app, new
            self.app_at = self.clock() - max(0.0, age_s)
            self.topic_seen('/robot/app_status')
            if prev != new:
                self._status_dirty = True
            self._trip_events(prev, new)
        self._wake.set()

    def _trip_events(self, prev: dict | None, new: dict) -> None:
        tid, ms = new['trip_id'], new['mission_state']
        if not tid:
            return
        t = self.trips.get(tid)
        if t is None:
            t = self.trips[tid] = {'started': False, 'ended': False, 'error': False}
            if prev is None:            # 기동 직후 받은 첫 상태: 이미 진행 중이던 trip은 이벤트를 다시 내지 않는다
                t.update(started=True, ended=ms in ('cancelled', 'arrived'), error=ms == 'error')
            while len(self.trips) > 50:
                self.trips.pop(next(iter(self.trips)))
        if ms in ('starting', 'active') and not t['started']:
            t['started'] = True
            frm, to = new['from_place_id'] or '', new['to_place_id'] or ''
            self._emit('trip.start', 'info',
                       M.trip_start_message(self.labels.get(frm, frm or '출발지'), self.labels.get(to, to or '목적지')),
                       trip_id=tid, data={'from_place_id': frm, 'to_place_id': to})
        elif ms == 'cancelled' and not t['ended']:
            t['ended'] = True
            self._emit('trip.cancelled', 'info', M.MSG_TRIP_CANCELLED, trip_id=tid,
                       data={'reason': self.cancel_reasons.get(tid, 'unknown')})
        elif ms == 'arrived' and not t['ended']:
            t['ended'] = True
            self._emit('trip.arrived', 'info', M.MSG_TRIP_ARRIVED, trip_id=tid, data={'reason': 'arrived'})
        elif ms == 'error' and not t['error']:
            t['error'] = True
            self._emit('trip.error', 'warning', new['last_error'] or M.MSG_TRIP_ERROR, trip_id=tid,
                       data={'error': new['last_error'] or 'unknown'})

    def on_link_status(self, *, connected: bool, app_instance_id: str, session_id: str, last_seen_ms: int) -> None:
        new = {'connected': bool(connected), 'app_instance_id': app_instance_id, 'session_id': session_id,
               'last_seen_ms': max(0, int(last_seen_ms))}
        with self._lock:
            prev, self.link = self.link, new
            if prev is None or prev['connected'] != new['connected']:
                self._status_dirty = True
            if prev is not None and prev['connected'] != new['connected']:
                app_id = new['app_instance_id'] or prev['app_instance_id']
                if new['connected']:
                    self._emit('link.connected', 'info', M.MSG_LINK_CONNECTED, data={'app_instance_id': app_id})
                else:
                    self._emit('link.disconnected', 'warning', M.MSG_LINK_DISCONNECTED,
                               data={'app_instance_id': app_id})
        self._wake.set()

    def on_base_status(self, *, agent_state: str, cmd_watchdog_ok: bool) -> None:
        new = {'agent_state': agent_state if agent_state in M.BASE_AGENT_STATES else 'unknown',
               'cmd_watchdog_ok': bool(cmd_watchdog_ok), 'last_seen_ms': self.wall_ms()}
        with self._lock:
            prev, self.base = self.base, new
            self.topic_seen('/base/status')
            if prev is None or (prev['agent_state'], prev['cmd_watchdog_ok']) != \
                    (new['agent_state'], new['cmd_watchdog_ok']):
                self._status_dirty = True

    def on_bridge_alive(self, alive: bool) -> None:
        with self._lock:
            if self.bridge_alive != alive:
                self.bridge_alive = alive
                self._status_dirty = True

    def on_marker(self, status: int) -> None:
        with self._lock:
            self.marker, self.marker_at = int(status), self.clock()
            self.topic_seen('/marker_track')

    def on_place_catalog(self, labels: dict[str, str]) -> None:
        with self._lock:
            self.labels = dict(labels)

    def on_trip_command(self, *, command: str, trip_id: str, reason: str = '') -> None:
        if command == 'cancel' and trip_id:
            with self._lock:
                self.cancel_reasons[trip_id] = reason or 'unknown'
                while len(self.cancel_reasons) > 50:
                    self.cancel_reasons.pop(next(iter(self.cancel_reasons)))

    def on_sos_request(self, *, sos_id: str, reason: str, severity: str, trip_id: str | None) -> None:
        wire = M.sos_wire_id(sos_id)
        reason = reason if reason in SOS_REASONS else 'unknown'
        severity = severity if severity in ('critical', 'warning') else 'critical'
        with self._lock:
            if wire in self.sos or wire in self.sos_closed:
                return
            ts = self.wall_ms()
            e = self.sos[wire] = {'started_ms': ts, 'reason': reason, 'severity': severity, 'trip_id': trip_id or None,
                                  'app_received': False, 'retries': 0, 'ack_ms': None, 'ack_due': None,
                                  'ack_emitted': False, 'fail_emitted': False, 'best_rank': 0, 'best_loc': None,
                                  'loc_updates': 0}
            self._emit('sos.request', severity, M.SOS_REQUEST_MESSAGES[reason], ts_ms=ts, trip_id=e['trip_id'],
                       sos_id=wire, data={'reason': reason})
            if self.cfg.publish_sos_location:
                self.outbox.put('PUT', self._sos_path(wire), {
                    'started_at_ms': ts, 'updated_at_ms': ts, 'reason': reason, 'severity': severity,
                    'trip_id': e['trip_id'], 'app_received': False, 'location': None})
        self._wake.set()

    def on_sos_ack(self, *, sos_id: str, app_received: bool, retries: int, has_location: bool = False,
                   best_source: str = 'none') -> None:
        wire = M.sos_wire_id(sos_id)
        with self._lock:
            if wire in self.sos_closed:
                return
            e = self.sos.get(wire)
            if e is None:                # 에이전트가 SOS 도중에 켜졌다: 이벤트만 남긴다
                e = self.sos[wire] = {'started_ms': self.wall_ms(), 'reason': 'unknown', 'severity': 'critical',
                                      'trip_id': None, 'app_received': False, 'retries': 0, 'ack_ms': None,
                                      'ack_due': None, 'ack_emitted': False, 'fail_emitted': False,
                                      'best_rank': 0, 'best_loc': None, 'loc_updates': MAX_LOCATION_UPDATES,
                                      'orphan': True}
            ts = self.wall_ms()
            e['retries'] = int(retries)
            changed = e['app_received'] != bool(app_received)
            e['app_received'] = bool(app_received)
            if changed and self.cfg.publish_sos_location and not e.get('orphan'):
                self.outbox.put('PATCH', self._sos_path(wire), {'app_received': e['app_received'], 'updated_at_ms': ts})
            if not app_received:
                if not e['fail_emitted']:
                    e['fail_emitted'] = True
                    e['ack_ms'] = ts
                    self._emit_sos_ack(wire, e)
            elif not e['ack_emitted'] and e['ack_due'] is None:
                e['ack_ms'] = ts
                e['ack_due'] = self.clock() + self.cfg.ack_debounce_s      # 캐시 위치가 곧 온다(≤500 ms)
        self._wake.set()

    def on_location(self, *, reason: str, sos_id: str | None, has_fix: bool, lat, lon, accuracy_m,
                    captured_at_ms: int, source: str, low_confidence: bool) -> None:
        if reason != 'sos' or not sos_id or not has_fix:
            return                      # periodic 위치는 RTDB에 쓰지 않는다
        lat, lon = M.finite_or_none(lat), M.finite_or_none(lon)
        if lat is None or lon is None or not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return
        rank = SOURCE_RANK.get(source, 0)
        if rank == 0:
            return
        with self._lock:
            e = self.sos.get(sos_id)
            if e is None or rank < e['best_rank']:
                return
            acc = M.finite_or_none(accuracy_m)
            loc = {'lat': lat, 'lon': lon, 'accuracy_m': acc if acc is None or acc >= 0 else None,
                   'captured_at_ms': max(0, int(captured_at_ms or 0)), 'source': source,
                   'low_confidence': bool(low_confidence)}
            e['best_rank'], e['best_loc'] = rank, loc
            if self.cfg.publish_sos_location and e['loc_updates'] < MAX_LOCATION_UPDATES:
                e['loc_updates'] += 1
                self.outbox.put('PATCH', self._sos_path(sos_id), {'location': loc, 'updated_at_ms': self.wall_ms()})
        self._wake.set()

    def on_sos_clear(self, *, sos_id: str, resolution: str) -> None:
        wire = M.sos_wire_id(sos_id)
        resolution = resolution if resolution in SOS_RESOLUTIONS else 'unknown'
        with self._lock:
            if wire in self.sos_closed:
                return
            e = self.sos.pop(wire, None)
            if e is not None and e['ack_due'] is not None and not e['ack_emitted']:
                self._emit_sos_ack(wire, e)
            self.sos_closed.append(wire)
            self._emit('sos.clear', 'info', M.SOS_CLEAR_MESSAGES[resolution], trip_id=e['trip_id'] if e else None,
                       sos_id=wire, data={'resolution': resolution})
            if self.cfg.publish_sos_location:
                self.outbox.put('DELETE', self._sos_path(wire))
        self._wake.set()

    def topic_seen(self, name: str) -> None:
        with self._lock:
            t = self.topics.setdefault(name, {'last_seen_ms': 0, 'count': 0})
            t['last_seen_ms'] = self.wall_ms()
            t['count'] += 1

    # ================================================================ 문서

    def status_doc(self) -> dict:
        now = self.clock()
        with self._lock:
            app_fresh = self.app is not None and now - self.app_at <= self.cfg.app_status_stale_s
            marker_fresh = self.marker is not None and now - self.marker_at <= self.cfg.marker_stale_s
            if app_fresh:
                a = self.app
                mission, tracking = a['mission_state'], a['tracking_state']
                trip, frm, to, battery, err = (a['trip_id'], a['from_place_id'], a['to_place_id'],
                                               a['battery_percent'], a['last_error'])
            else:
                mission = 'unknown'
                tracking = ({0: 'lost', 1: 'following', 2: 'following'}.get(self.marker, 'unknown')
                            if marker_fresh else 'unavailable')
                trip = frm = to = battery = err = None
            if not self.bridge_alive:
                bridge = 'unknown'
            elif app_fresh or marker_fresh:
                bridge = 'ready'
            else:
                bridge = 'degraded'
            link = self.link if self.bridge_alive else None
            doc = {
                'updated_at_ms': self.wall_ms(),
                'agent': {'version': self.cfg.version, 'uptime_s': int(now - self.started_at),
                          'queue_len': len(self.outbox)},
                'bridge_state': bridge,
                'mission_state': mission,
                'tracking_state': tracking,
                'trip_id': trip,
                'from_place_id': frm,
                'to_place_id': to,
                'battery_percent': battery if battery is None or 0 <= battery <= 100 else None,
                'last_error': err,
                'phone_link': {'connected': bool(link and link['connected']),
                               'last_seen_ms': (self.link or {}).get('last_seen_ms', 0)},
            }
            if self.base is not None:
                doc['base'] = dict(self.base)
            return doc

    def diag_doc(self, elapsed_s: float) -> dict:
        with self._lock:
            topics = {}
            for name in DIAG_TOPICS:
                t = self.topics.get(name, {'last_seen_ms': 0, 'count': 0})
                topics[name] = {'last_seen_ms': t['last_seen_ms'],
                                'hz': round(t['count'] / elapsed_s, 1) if elapsed_s > 0 else 0}
                if name in self.topics:
                    self.topics[name]['count'] = 0
            doc = {'updated_at_ms': self.wall_ms(), 'topics': M.topic_tree(topics)}
            if self.errors:                  # RTDB는 빈 배열을 저장하지 않는다(README 3.5: 없으면 필드 없음)
                doc['recent_errors'] = list(self.errors)
            return doc

    # ================================================================ 내부

    def _sos_path(self, wire: str) -> str:
        return f'{self.root}/sos/{M.safe_key(wire)}'

    def _emit(self, kind: str, severity: str, message: str, *, ts_ms: int | None = None, trip_id=None,
              sos_id=None, data: dict | None = None) -> None:
        ts = self.wall_ms() if ts_ms is None else ts_ms
        doc = M.event_doc(ts, kind, severity, message, trip_id=trip_id, sos_id=sos_id, data=data)
        self.outbox.put('PUT', f'{self.root}/events/{self.pushids(ts)}', doc)
        log.info('event %s: %s', kind, message)
        self._wake.set()

    def _emit_sos_ack(self, wire: str, e: dict) -> None:
        loc = e['best_loc']
        source = loc['source'] if loc else 'none'
        bucket = M.accuracy_bucket(loc is not None, loc['accuracy_m'] if loc else None,
                                   loc['low_confidence'] if loc else False)
        self._emit('sos.ack', 'info', M.MSG_SOS_ACK_OK if e['app_received'] else M.MSG_SOS_ACK_FAIL,
                   ts_ms=e['ack_ms'], trip_id=e['trip_id'], sos_id=wire,
                   data={'app_received': e['app_received'], 'retries': e['retries'], 'location': source,
                         'accuracy_bucket': bucket})
        if e['app_received']:
            e['ack_emitted'] = True
        e['ack_due'] = None

    def _record_error(self, err: Exception | str) -> None:
        with self._lock:
            self.errors.append({'ts_ms': self.wall_ms(), 'source': 'telemetry', 'error': str(err)[:200]})

    def _rejected_write(self, what: str, e: RtdbError) -> None:
        """RTDB가 영구 오류로 거절한 쓰기. 다시 보내도 같으므로 로그 ERROR로 드러낸다.
        같은 대상은 처음 한 번, 그 뒤로는 error_log_repeat_s마다 그동안 몇 번 더 났는지와 함께 남긴다."""
        self._record_error(e)
        now = self.clock()
        r = self._rejected.get(what)
        if r is None or now - r['logged_at'] >= self.cfg.error_log_repeat_s:
            more = f' ({r["suppressed"]} more since last report)' if r and r['suppressed'] else ''
            log.error('RTDB rejected %s: %s%s', what, e, more)
            self._rejected[what] = {'logged_at': now, 'suppressed': 0}
        else:
            r['suppressed'] += 1

    def _write_accepted(self, what: str) -> None:
        if self._rejected.pop(what, None) is not None:
            log.info('RTDB accepts %s again', what)

    def _set_online(self, online: bool, err: Exception | None = None) -> None:
        if online != self.online:
            if online:
                log.info('RTDB online (%s)', self.client.target)
            else:
                log.warning('RTDB offline: %s', err)
            self.online = online

    # ================================================================ 전송 스레드

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._step()
            except Exception as e:           # 전송 스레드는 죽지 않는다
                log.exception('telemetry step failed')
                self._record_error(e)
            self._wake.wait(self.cfg.loop_s)
            self._wake.clear()
        self._final_flush()

    def _step(self) -> None:
        now = self.clock()
        with self._lock:
            for wire, e in list(self.sos.items()):
                if e['ack_due'] is not None and now >= e['ack_due'] and not e['ack_emitted']:
                    self._emit_sos_ack(wire, e)
        dirty = self._status_dirty and now - self._last_status_at >= self.cfg.status_min_interval_s
        if dirty or now - self._last_status_at >= self.cfg.status_period_s:
            self._put_status(now)
        if self.cfg.publish_diag and now - self._last_diag_at >= self.cfg.diag_period_s:
            elapsed = now - self._last_diag_at if self._last_diag_at > 0 else self.cfg.diag_period_s
            self._last_diag_at = now
            self._put_diag(elapsed)
        if now < self._next_send_at:
            return
        if not self._meta_done:
            self._sync_meta()
            if not self._meta_done:
                return
        self._drain(now)
        if self._backoff_s == 0 and now >= self._next_prune_at:
            self._prune()
            self._next_prune_at = now + self.cfg.prune_period_s

    def _put_status(self, now: float) -> None:
        self._status_dirty = False
        self._last_status_at = now
        try:
            self.client.put(f'{self.root}/status', self.status_doc())
        except RtdbError as e:
            if not e.transient:          # RTDB는 닿았다. 문서가 잘못됐다
                self._rejected_write('status', e)
                return
            self._set_online(False, e)
            self._record_error(e)
            return
        self._write_accepted('status')
        self._set_online(True)
        if self._backoff_s > 0:          # 다시 온라인: 큐를 바로 보낸다
            self._next_send_at = 0.0

    def _put_diag(self, elapsed_s: float) -> None:
        """최신값만 의미 있다: 실패하면 버린다. 온라인 판정은 status가 한다."""
        try:
            self.client.put(f'{self.root}/diag', self.diag_doc(elapsed_s))
        except RtdbError as e:
            if e.transient:
                self._record_error(e)
            else:
                self._rejected_write('diag', e)
            return
        self._write_accepted('diag')

    def _sync_meta(self) -> None:
        try:
            cur = self.client.get(f'{self.root}/meta')
            first = cur.get('first_seen_ms') if isinstance(cur, dict) else None
            self.client.put(f'{self.root}/meta', {
                'robot_id': self.cfg.robot_id, 'display_name': self.cfg.display_name[:40],
                'agent_version': self.cfg.version,
                'first_seen_ms': first if isinstance(first, int) and first > 0 else self.wall_ms()})
            self._meta_done = True
        except RtdbError as e:
            if e.transient:
                self._fail(e)
            else:                        # 다시 보내도 같다. meta 때문에 큐를 막지 않는다
                self._rejected_write('meta', e)
                self._meta_done = True

    def _drain(self, now: float) -> None:
        while not self._stop.is_set() or self._flushing:
            ops = self.outbox.peek(20)
            if not ops:
                return
            for op in ops:
                try:
                    self.client.request(op.method, op.path, op.body)
                except RtdbError as e:
                    if e.transient:
                        self.outbox.failed(op.id)
                        self._fail(e)
                        return
                    log.error('RTDB rejected %s %s, dropping it from the queue: %s', op.method, op.path, e)
                    self._record_error(e)
                self.outbox.done(op.id)
                self._backoff_s = 0.0
                self._set_online(True)

    _flushing = False

    def _fail(self, e: RtdbError) -> None:
        self._backoff_s = 1.0 if self._backoff_s == 0 else min(self.cfg.backoff_max_s, self._backoff_s * 2)
        self._next_send_at = self.clock() + self._backoff_s
        self._set_online(False, e)
        self._record_error(e)

    def _prune(self) -> None:
        path = f'{self.root}/events'
        try:
            keys = self.client.get(path, shallow='true') or {}
            excess = sorted(keys)[:max(0, len(keys) - self.cfg.max_events)]
            for i in range(0, len(excess), 200):
                self.client.patch(path, {k: None for k in excess[i:i + 200]})
            if excess:
                log.info('pruned %d old event(s)', len(excess))
        except RtdbError as e:
            if e.transient:
                self._fail(e)
            else:
                self._rejected_write('events prune', e)

    def _final_flush(self) -> None:
        """정지 직전: 백오프를 무시하고 한 번 더 보낸다. 실패하면 큐에 남긴다."""
        deadline = getattr(self, '_flush_deadline', self.clock())
        self._flushing = True
        try:
            while len(self.outbox) and self.clock() < deadline:
                before = len(self.outbox)
                self._backoff_s = 0.0
                self._drain(self.clock())
                if len(self.outbox) == before:
                    break
        finally:
            self._flushing = False
