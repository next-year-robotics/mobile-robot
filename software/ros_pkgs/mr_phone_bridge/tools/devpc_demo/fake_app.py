#!/usr/bin/env python3
"""개발 PC 데모용 가짜 앱: phone-bridge/1 클라이언트. 폰 없이 앱 흐름(출발·멈춤·SOS 응답·연결 끊김)을 돌린다.

    fake_app.py serve --url ws://127.0.0.1:18865/bridge/v1 --state DIR   # demo.sh start --fake-app이 띄운다
    fake_app.py ctl --state DIR <명령…>                                   # demo.sh ctl trip …·ctl app …이 부른다

브리지는 앱 세션을 하나만 둔다(새 hello가 이전 세션을 4000으로 닫는다). **포트 8765(브리지 기본값)는 거부한다**:
붙는 순간 진짜 폰의 세션을 밀어낸다. 데모는 MR_DEMO_PORT를 바꿔서 쓴다.

동작(PROTOCOL.md):
- 연결 → hello → hello.ack·place.catalog·robot.status. heartbeat는 hello.ack의 주기(2 s). 끊기면 1·2·4·8·10 s 뒤 다시 연결(8.4, jitter 없음).
- sos.request → 정책대로 sos.ack(received=true)와 location.update(reason=sos). 같은 sos_id가 다시 오면 ACK와 지금까지
  보낸 최선 위치를 다시 보낸다(10.5).
- 받은 메시지는 모두 계약 스키마로 검사한다(위반은 로그 `CONTRACT VIOLATION`, status의 violations). 보내는 메시지도
  브리지와 같은 검사기(parse_frame)로 검사한다.
- 모든 송수신은 DIR/fake_app.jsonl에 남는다(실행마다 새로 쓴다).

제어 명령(ctl, 응답은 JSON 한 줄):
  status | mark | log [n] | quit
  trip start <출발> <도착>          장소는 id(place_home)·짧은 id(home)·이름(집)
  trip cancel [trip_id]             기본: 현재 trip
  ack yes|no|late <s>               sos.request에 ACK(기본 yes). late: 첫 요청 s초 뒤에 ACK
  location <모드>                   SOS 위치(기본 cache+fresh). 아래 LOCATION_PLANS
  accuracy <cache_m> [fresh_m]      기본 18 4 → 대시보드 정확도 버킷 fair → good
  drop close|abort|silent [s]       연결 끊기. close = 정상 닫기, abort = close frame 없이 TCP 끊기,
                                    silent = 소켓은 둔 채 아무것도 안 보내고 받은 것도 무시(브리지 heartbeat 타임아웃 10 s).
                                    s를 주면 끊은 때부터 s초 뒤 다시 연결, 없으면 connect까지 끊긴 채
  connect                           다시 연결 허용
  wait <조건> [timeout_s]           key=value | key!=value | key~부분문자열 (robot.status 필드, connected),
                                    또는 메시지 type(sos.request, sos.clear, error …: mark 뒤에 온 것만)
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import shlex
import socket
import sys
import time
import uuid
from pathlib import Path
from urllib.parse import urlparse

DEFAULT_BRIDGE_PORT = 8765
REPLY_TIMEOUT_S = 3.0
BACKOFF_S = (1, 2, 4, 8, 10)
HISTORY_MAX = 5000
# SOS 위치: (보내는 시점, 종류). 0 = ACK 직후(≤ 500 ms 규칙), 'later' = fresh_delay 뒤(실제 앱은 최대 30 s)
LOCATION_PLANS = {
    'cache+fresh': ((0, 'cache'), ('later', 'fresh')),
    'cache': ((0, 'cache'), ('later', 'none:timeout')),
    'fresh': ((0, 'none:unavailable'), ('later', 'fresh')),
    'none': ((0, 'none:provider_disabled'),),
    'off': (),                                        # 계약 위반(위치를 안 보냄). 시험용
}
SOURCE_RANK = {'none': 0, 'cache': 1, 'fresh': 2}
# 계약 예시 좌표(PROTOCOL 10.2). 실제 사람의 위치가 아니다.
FAKE_LAT, FAKE_LON = 36.1234, 128.5678
STATUS_FIELDS = ('bridge_state', 'mission_state', 'tracking_state', 'trip_id', 'from_place_id', 'to_place_id',
                 'battery_percent', 'last_error')

log = logging.getLogger('fake_app')


def now_ms() -> int:
    return time.time_ns() // 1_000_000


def refuse_default_port(url: str) -> None:
    port = urlparse(url).port
    if port is None or port == DEFAULT_BRIDGE_PORT:
        sys.exit(f'fake_app: {url}: 포트 {DEFAULT_BRIDGE_PORT}(기본)은 쓰지 않는다. 브리지는 앱 세션을 하나만 두므로 '
                 '진짜 폰의 세션을 밀어낸다. MR_DEMO_PORT를 바꿔 띄운 데모 브리지에만 붙는다')


class CmdError(Exception):
    pass


class FakeApp:

    def __init__(self, url: str, state: Path, *, ack: str = 'yes', ack_late_s: float = 0.0,
                 location: str = 'cache+fresh', fresh_delay_s: float = 2.0, cache_acc_m: float = 18.0,
                 fresh_acc_m: float = 4.0):
        from mr_phone_bridge.protocol.envelope import check_outgoing, parse_frame   # 계약 검사기(브리지와 같은 것)
        self._check_in, self._check_out = check_outgoing, parse_frame
        self.url = url
        self.state = state
        self.ack, self.ack_late_s = ack, ack_late_s
        self.location = location
        self.fresh_delay_s = fresh_delay_s
        self.cache_acc_m, self.fresh_acc_m = cache_acc_m, fresh_acc_m
        self.app_instance_id = self._instance_id()
        self.ws = None
        self.session_id: str | None = None
        self.connected = False                 # handshake까지 끝났나
        self.silent = False
        self.want = True                       # 끊기면 다시 연결할까
        self.status: dict | None = None
        self.catalog: dict | None = None
        self.last_trip_id: str | None = None
        self.sos: dict[str, dict] = {}         # 활성 SOS
        self.history: list[dict] = []
        self.seq = 0
        self.mark = 0
        self.violations = 0
        self.last_close: dict | None = None
        self.stopping = False
        self._replies: dict[str, asyncio.Future] = {}
        self._waiters: set[asyncio.Future] = set()
        self._want_event = asyncio.Event()
        self._want_event.set()
        self._tasks: set[asyncio.Task] = set()
        self._jsonl = open(state / 'fake_app.jsonl', 'w', encoding='utf-8')   # 실행마다 새로(seq가 1부터)

    def _instance_id(self) -> str:
        path = self.state / 'fake_app.id'
        try:
            value = path.read_text().strip()
            uuid.UUID(value)
            return value
        except (OSError, ValueError):
            value = str(uuid.uuid4())
            path.write_text(value + '\n')
            return value

    # ------------------------------------------------------------ 연결

    async def run(self) -> None:
        fails = 0
        while not self.stopping:
            if not self.want:
                self._want_event.clear()
                await self._want_event.wait()
                continue
            opened = await self._session()
            fails = 0 if opened else fails + 1
            if self.stopping or not self.want:
                continue
            delay = BACKOFF_S[min(fails, len(BACKOFF_S) - 1)] if not opened else BACKOFF_S[0]
            log.info('reconnect in %d s', delay)
            self._want_event.clear()
            try:
                await asyncio.wait_for(self._want_event.wait(), delay)   # connect 명령이면 바로
            except asyncio.TimeoutError:
                pass

    async def _session(self) -> bool:
        import websockets
        from websockets.exceptions import ConnectionClosed
        try:
            ws = await websockets.connect(self.url, open_timeout=3, close_timeout=2, ping_interval=None)
        except Exception as e:
            log.info('connect %s failed: %s', self.url, e)
            return False
        self.ws, self.silent = ws, False
        reader = asyncio.ensure_future(self._reader(ws, ConnectionClosed))
        opened = False
        hb = None
        try:
            _, reply = await self._request('hello', {
                'app_instance_id': self.app_instance_id, 'app_version': '0.0.0-fake', 'platform': 'android',
                'capabilities': ['location.cache', 'location.fresh']}, reader)
            if reply is None or reply['type'] != 'hello.ack':
                log.error('handshake failed: %s', reply and reply.get('payload'))
                await ws.close(1000, 'handshake failed')
            else:
                self.session_id = reply['payload']['session_id']
                self.connected = opened = True
                log.info('session %s open (%s)', self.session_id, self.url)
                self._kick()
                hb = asyncio.ensure_future(self._heartbeat(ws, reply['payload']['heartbeat_interval_ms'] / 1000))
            await reader
        finally:
            if hb is not None:
                hb.cancel()
            reader.cancel()
            self.connected, self.session_id, self.ws, self.silent = False, None, None, False
            self.status = None                 # 끊긴 동안의 상태는 모른다. 다음 handshake의 robot.status로 다시 맞춘다(8.4)
            self.last_close = {'code': ws.close_code, 'reason': ws.close_reason, 'at_ms': now_ms()}
            self._record('close', {'type': 'close', 'payload': self.last_close})
            log.info('connection closed: %s %s', ws.close_code, ws.close_reason)
            for f in self._replies.values():
                if not f.done():
                    f.set_result(None)
            self._kick()
        return opened

    async def _reader(self, ws, closed_exc) -> None:
        try:
            async for raw in ws:
                self._on_frame(raw)
        except closed_exc:
            pass

    async def _heartbeat(self, ws, interval_s: float) -> None:
        while True:
            await asyncio.sleep(interval_s)
            if self.silent or ws is not self.ws:
                continue
            try:
                await self._send('heartbeat', {'session_id': self.session_id})
            except Exception as e:
                log.debug('heartbeat send failed: %s', e)

    async def drop(self, mode: str, reconnect_after_s: float | None) -> dict:
        ws = self.ws
        if ws is None:
            raise CmdError('not connected')
        self.want = False
        at = time.monotonic()
        log.warning('drop %s (reconnect %s)', mode, f'after {reconnect_after_s:g} s' if reconnect_after_s else 'on connect')
        if mode == 'close':
            await ws.close(1000, 'fake app drop')
        elif mode == 'abort':
            ws.transport.abort()
        elif mode == 'silent':
            self.silent = True
            self.connected = False            # 앱 입장에서는 이미 끊겼다(보내지도 받지도 않는다)
            self._kick()
        else:
            raise CmdError(f'unknown drop mode {mode!r}')
        if reconnect_after_s is not None:
            self._spawn(self._reconnect_later(at + reconnect_after_s, ws))
        return {'dropped': mode, 'at_ms': now_ms()}

    async def _reconnect_later(self, at: float, old_ws) -> None:
        await asyncio.sleep(max(0.0, at - time.monotonic()))
        if old_ws is self.ws:                 # silent: 서버가 아직 안 닫았으면 우리가 버린다
            old_ws.transport.abort()
            for _ in range(50):
                if self.ws is None:
                    break
                await asyncio.sleep(0.05)
        self.connect()

    def connect(self) -> None:
        self.want = True
        self._want_event.set()

    # ------------------------------------------------------------ 수신

    def _on_frame(self, raw) -> None:
        if isinstance(raw, bytes):
            self._violation('binary frame', raw[:80])
            return
        try:
            msg = json.loads(raw)
        except ValueError:
            self._violation('malformed JSON', raw[:200])
            return
        err = self._check_in(msg) if isinstance(msg, dict) else 'not an object'
        if err:
            self._violation(err, msg)
            if not isinstance(msg, dict) or not isinstance(msg.get('payload'), dict):
                return
        self._record('in', msg, ignored=self.silent)
        if self.silent:
            return
        rt = msg.get('reply_to')
        if rt in self._replies and not self._replies[rt].done():
            self._replies[rt].set_result(msg)
        mtype, p = msg.get('type'), msg['payload']
        if mtype == 'robot.status':
            old = self.status
            self.status = p
            if old is None or any(old.get(k) != p.get(k) for k in STATUS_FIELDS if k != 'battery_percent'):
                log.info('status %s/%s/%s trip=%s error=%s', p['bridge_state'], p['mission_state'],
                         p['tracking_state'], (p['trip_id'] or '-')[:8], p['last_error'])
        elif mtype == 'place.catalog':
            self.catalog = p
        elif mtype == 'sos.request':
            self._on_sos_request(msg)
        elif mtype == 'sos.clear':
            s = self.sos.pop(p['sos_id'], None)
            for t in (s or {}).get('tasks', ()):
                t.cancel()
            log.warning('SOS %s cleared: %s', p['sos_id'][:8], p['resolution'])
        elif mtype == 'error' and rt not in self._replies:
            log.warning('error from bridge: %s', p)
        self._kick()

    def _on_sos_request(self, msg: dict) -> None:
        p = msg['payload']
        sid = p['sos_id']
        s = self.sos.get(sid)
        first = s is None
        if first:
            s = self.sos[sid] = {'reason': p['reason'], 'severity': p['severity'], 'trip_id': p['trip_id'],
                                 'requests': 0, 'acked': False, 'ack_scheduled': False, 'best': None,
                                 'tasks': set(), 'received_ms': now_ms()}
        s['requests'] += 1
        log.warning('SOS %s %s/%s request #%d (ack=%s, location=%s)', sid[:8], p['reason'], p['severity'],
                    s['requests'], self.ack if self.ack != 'late' else f'late {self.ack_late_s:g}s', self.location)
        if self.ack == 'no':
            return
        if self.ack == 'late' and not s['acked']:
            if not s['ack_scheduled']:
                s['ack_scheduled'] = True
                self._sos_task(s, self._respond_sos(sid, msg['id'], self.ack_late_s))
            return
        self._sos_task(s, self._respond_sos(sid, msg['id'], 0.0))

    def _sos_task(self, s: dict, coro) -> None:
        t = self._spawn(coro)
        s['tasks'].add(t)
        t.add_done_callback(s['tasks'].discard)

    async def _respond_sos(self, sid: str, request_id: str, delay_s: float) -> None:
        if delay_s:
            await asyncio.sleep(delay_s)
        s = self.sos.get(sid)
        if s is None or self.silent or not self.connected:
            return
        await self._send('sos.ack', {'sos_id': sid, 'received': True}, reply_to=request_id)
        if s['acked']:                         # 같은 SOS 재요청: 최선 위치만 다시(10.5)
            best = dict(s['best']) if s['best'] is not None else None
            if best is not None:
                if best['has_fix']:
                    best['age_ms'] = now_ms() - best['captured_at_ms']
                await self._send('location.update', best)
            return
        s['acked'] = True
        for when, kind in LOCATION_PLANS[self.location]:
            if when == 0:
                await self._send_sos_location(sid, kind)
            else:
                self._sos_task(s, self._later_location(sid, kind))

    async def _later_location(self, sid: str, kind: str) -> None:
        await asyncio.sleep(self.fresh_delay_s)
        await self._send_sos_location(sid, kind)

    async def _send_sos_location(self, sid: str, kind: str) -> None:
        s = self.sos.get(sid)
        if s is None or self.silent or not self.connected:
            return
        payload = self._location_payload(sid, kind)
        await self._send('location.update', payload)
        best = s['best']
        if best is None or SOURCE_RANK[payload['source']] >= SOURCE_RANK[best['source']]:
            s['best'] = payload

    def _location_payload(self, sid: str | None, kind: str, reason: str = 'sos') -> dict:
        if kind in ('cache', 'fresh'):
            age = 800 if kind == 'cache' else 150
            return {'reason': reason, 'sos_id': sid, 'has_fix': True, 'lat': FAKE_LAT, 'lon': FAKE_LON,
                    'horizontal_accuracy_m': self.cache_acc_m if kind == 'cache' else self.fresh_acc_m,
                    'captured_at_ms': now_ms() - age, 'age_ms': age, 'source': kind, 'failure_reason': None}
        failure = kind.split(':', 1)[1]
        return {'reason': reason, 'sos_id': sid, 'has_fix': False, 'lat': None, 'lon': None,
                'horizontal_accuracy_m': None, 'captured_at_ms': None, 'age_ms': None, 'source': 'none',
                'failure_reason': failure}

    # ------------------------------------------------------------ 송신

    async def _send(self, mtype: str, payload: dict, *, reply_to: str | None = None, msg_id: str | None = None) -> str:
        ws = self.ws
        if ws is None or self.silent:
            raise CmdError('not connected')
        msg = {'version': 1, 'type': mtype, 'id': msg_id or str(uuid.uuid4()), 'sent_at_ms': now_ms(),
               'reply_to': reply_to, 'payload': payload}
        text = json.dumps(msg, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
        self._check_out(text)               # 계약 위반이면 ProtocolError: 가짜 앱의 버그다
        await ws.send(text)
        self._record('out', msg)
        return msg['id']

    async def _request(self, mtype: str, payload: dict, reader: asyncio.Future | None = None) -> tuple[str, dict | None]:
        """보내고 reply_to가 이 id인 응답을 3 s 기다린다. 응답 future는 보내기 전에 건다(localhost는 응답이 빠르다)."""
        mid = str(uuid.uuid4())
        fut = asyncio.get_running_loop().create_future()
        self._replies[mid] = fut
        try:
            await self._send(mtype, payload, msg_id=mid)
            waits = [fut] + ([reader] if reader is not None else [])
            done, _ = await asyncio.wait(waits, timeout=REPLY_TIMEOUT_S, return_when=asyncio.FIRST_COMPLETED)
            return mid, (fut.result() if fut in done else None)
        finally:
            self._replies.pop(mid, None)

    async def command(self, mtype: str, payload: dict) -> dict:
        if not self.connected:
            raise CmdError('not connected')
        mid, reply = await self._request(mtype, payload)
        if reply is None:
            return {'ok': False, 'error': 'no reply within 3 s (결과 불명)', 'id': mid}
        return {'ok': reply['type'] == 'ack', 'reply': reply['type'], 'payload': reply['payload'], 'id': mid}

    # ------------------------------------------------------------ 기록·대기

    def _record(self, direction: str, msg: dict, **extra) -> None:
        self.seq += 1
        entry = {'seq': self.seq, 't': time.monotonic(), 'wall_ms': now_ms(), 'dir': direction,
                 'type': msg.get('type'), 'id': msg.get('id'), 'reply_to': msg.get('reply_to'),
                 'payload': msg.get('payload'), **extra}
        self.history.append(entry)
        del self.history[:-HISTORY_MAX]
        self._jsonl.write(json.dumps(entry, ensure_ascii=False) + '\n')
        self._jsonl.flush()

    def _violation(self, err, msg) -> None:
        self.violations += 1
        log.error('CONTRACT VIOLATION: %s: %s', err, str(msg)[:300])

    def _kick(self) -> None:
        for f in self._waiters:
            if not f.done():
                f.set_result(None)
        self._waiters.clear()

    async def wait(self, cond: str, timeout: float) -> dict:
        check = self._condition(cond)
        start = time.monotonic()
        while True:
            hit = check()
            if hit is not None:
                return {'ok': True, 'elapsed_s': round(time.monotonic() - start, 2), 'match': hit,
                        'status': self.status}
            left = timeout - (time.monotonic() - start)
            if left <= 0:
                return {'ok': False, 'error': f'timeout after {timeout:g} s waiting for {cond}', 'status': self.status,
                        'connected': self.connected}
            fut = asyncio.get_running_loop().create_future()
            self._waiters.add(fut)
            try:
                await asyncio.wait_for(fut, min(left, 0.5))
            except asyncio.TimeoutError:
                pass

    def _condition(self, cond: str):
        for op in ('!=', '=', '~'):
            if op in cond:
                key, value = cond.split(op, 1)
                break
        else:
            mtype = cond

            def seen():
                for e in self.history:
                    if e['seq'] > self.mark and e['dir'] == 'in' and e['type'] == mtype:
                        return {k: e[k] for k in ('seq', 'type', 'id', 'payload', 'wall_ms')}
                return None
            return seen
        if key != 'connected' and key not in STATUS_FIELDS:
            raise CmdError(f'unknown key {key!r}: connected, {", ".join(STATUS_FIELDS)}')
        want = {'null': None, 'true': True, 'false': False}.get(value, value)

        def current():
            if key == 'connected':
                got = self.connected
            elif self.status is None:
                return None
            else:
                got = self.status.get(key)
            if op == '=':
                ok = got == want or (isinstance(got, (int, float)) and str(got) == value)
            elif op == '!=':
                ok = got != want
            else:
                ok = isinstance(got, str) and value in got
            return {key: got} if ok else None
        return current

    def snapshot(self) -> dict:
        return {'ok': True, 'url': self.url, 'connected': self.connected, 'session_id': self.session_id,
                'want_connected': self.want, 'silent': self.silent, 'app_instance_id': self.app_instance_id,
                'ack': self.ack if self.ack != 'late' else f'late {self.ack_late_s:g}', 'location': self.location,
                'accuracy_m': [self.cache_acc_m, self.fresh_acc_m], 'robot_status': self.status,
                'active_sos': {k: {x: v[x] for x in ('reason', 'severity', 'trip_id', 'requests', 'acked')}
                               for k, v in self.sos.items()},
                'last_trip_id': self.last_trip_id, 'seq': self.seq, 'mark': self.mark,
                'violations': self.violations, 'last_close': self.last_close}

    # ------------------------------------------------------------ 제어 명령

    def _place(self, name: str) -> str:
        places = (self.catalog or {}).get('places') or []
        for cand in (name, f'place_{name}'):
            if any(p['id'] == cand for p in places):
                return cand
        for p in places:
            if p['label'] == name:
                return p['id']
        raise CmdError(f'unknown place {name!r}; known: ' + ', '.join(f'{p["id"]}({p["label"]})' for p in places))

    async def handle(self, words: list[str]) -> dict:
        if not words:
            raise CmdError('empty command')
        cmd, args = words[0], words[1:]
        if cmd == 'status':
            return self.snapshot()
        if cmd == 'mark':
            self.mark = self.seq
            return {'ok': True, 'mark': self.mark}
        if cmd == 'log':
            n = int(args[0]) if args else 20
            return {'ok': True, 'log': [{k: e[k] for k in ('seq', 'dir', 'type', 'payload')} for e in self.history[-n:]]}
        if cmd == 'trip' and args[:1] == ['start'] and len(args) == 3:
            frm, to = self._place(args[1]), self._place(args[2])
            res = await self.command('trip.start', {'from_place_id': frm, 'to_place_id': to,
                                                    'catalog_revision': self.catalog['revision']})
            if res['ok']:
                self.last_trip_id = res['payload']['trip_id']
            return res
        if cmd == 'trip' and args[:1] == ['cancel'] and len(args) <= 2:
            trip_id = args[1] if len(args) == 2 else ((self.status or {}).get('trip_id') or self.last_trip_id)
            if not trip_id:
                raise CmdError('no current trip')
            return await self.command('trip.cancel', {'trip_id': trip_id, 'reason': 'user_request'})
        if cmd == 'ack' and args[:1] in (['yes'], ['no']) and len(args) == 1:
            self.ack = args[0]
            return self.snapshot()
        if cmd == 'ack' and args[:1] == ['late'] and len(args) == 2:
            self.ack, self.ack_late_s = 'late', float(args[1])
            return self.snapshot()
        if cmd == 'location' and len(args) == 1 and args[0] in LOCATION_PLANS:
            self.location = args[0]
            return self.snapshot()
        if cmd == 'accuracy' and 1 <= len(args) <= 2:
            self.cache_acc_m = float(args[0])
            self.fresh_acc_m = float(args[1]) if len(args) == 2 else self.fresh_acc_m
            return self.snapshot()
        if cmd == 'drop' and 1 <= len(args) <= 2:
            return {'ok': True, **await self.drop(args[0], float(args[1]) if len(args) == 2 else None)}
        if cmd == 'connect' and not args:
            self.connect()
            return {'ok': True}
        if cmd == 'wait' and 1 <= len(args) <= 2:
            return await self.wait(args[0], float(args[1]) if len(args) == 2 else 30.0)
        if cmd == 'quit':
            self.stopping = True
            self.want = False
            self._want_event.set()
            if self.ws is not None:
                await self.ws.close(1001, 'fake app quit')
            return {'ok': True}
        raise CmdError(f'unknown command: {" ".join(words)} (see fake_app.py header)')

    async def serve_control(self, port_file: Path) -> asyncio.AbstractServer:
        async def on_client(reader, writer):
            try:
                line = (await reader.readline()).decode('utf-8').strip()
                try:
                    res = await self.handle(shlex.split(line))
                except (CmdError, ValueError) as e:
                    res = {'ok': False, 'error': str(e)}
                writer.write((json.dumps(res, ensure_ascii=False) + '\n').encode('utf-8'))
                await writer.drain()
            except Exception:
                log.exception('control command failed')
            finally:
                writer.close()

        server = await asyncio.start_server(on_client, '127.0.0.1', 0)
        port = server.sockets[0].getsockname()[1]
        tmp = port_file.with_suffix('.tmp')
        tmp.write_text(f'{port}\n')
        os.replace(tmp, port_file)
        return server

    def _spawn(self, coro) -> asyncio.Task:
        t = asyncio.ensure_future(coro)
        self._tasks.add(t)
        t.add_done_callback(self._task_done)
        return t

    def _task_done(self, t: asyncio.Task) -> None:
        self._tasks.discard(t)
        if not t.cancelled() and t.exception() is not None and not isinstance(t.exception(), CmdError):
            log.error('task failed: %r', t.exception())


# ============================================================ 진입점

def control(state: Path, line: str, timeout: float = 60.0) -> dict:
    """제어 명령 하나를 보내고 응답(dict)을 돌려준다. e2e.py도 이것을 쓴다."""
    try:
        port = int((state / 'fake_app.port').read_text())
    except (OSError, ValueError):
        return {'ok': False, 'error': f'fake app is not running ({state}/fake_app.port)'}
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=timeout) as s:
            s.sendall((line + '\n').encode('utf-8'))
            buf = b''
            while not buf.endswith(b'\n'):
                chunk = s.recv(65536)
                if not chunk:
                    break
                buf += chunk
    except OSError as e:
        return {'ok': False, 'error': f'control connection failed: {e}'}
    return json.loads(buf) if buf else {'ok': False, 'error': 'empty response'}


def ctl_timeout(words: list[str]) -> float:
    if words[:1] == ['wait']:
        return (float(words[2]) if len(words) > 2 else 30.0) + 5.0
    return 30.0


async def serve(args) -> None:
    import signal
    state = Path(args.state)
    state.mkdir(parents=True, exist_ok=True)
    app = FakeApp(args.url, state, ack=args.ack, location=args.location, fresh_delay_s=args.fresh_delay)
    server = await app.serve_control(state / 'fake_app.port')
    loop = asyncio.get_running_loop()
    main = asyncio.ensure_future(app.run())

    def stop():
        app.stopping, app.want = True, False
        app._want_event.set()
        if app.ws is not None:
            asyncio.ensure_future(app.ws.close(1001, 'fake app exit'))
        loop.call_later(3.0, main.cancel)
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop)
    log.info('fake app ready: %s (control 127.0.0.1:%s, ack=%s, location=%s)', args.url,
             server.sockets[0].getsockname()[1], app.ack, app.location)
    try:
        await main
    except asyncio.CancelledError:
        pass
    finally:
        server.close()
        (state / 'fake_app.port').unlink(missing_ok=True)
        log.info('fake app stopped')


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='mode', required=True)
    s = sub.add_parser('serve')
    s.add_argument('--url', required=True)
    s.add_argument('--state', required=True)
    s.add_argument('--ack', choices=('yes', 'no'), default='yes')
    s.add_argument('--location', choices=tuple(LOCATION_PLANS), default='cache+fresh')
    s.add_argument('--fresh-delay', type=float, default=2.0)
    c = sub.add_parser('ctl')
    c.add_argument('--state', required=True)
    c.add_argument('words', nargs=argparse.REMAINDER)
    args = ap.parse_args()
    if args.mode == 'serve':
        refuse_default_port(args.url)
        logging.basicConfig(level=logging.INFO, format='%(asctime)s [fake_app] %(levelname)s %(message)s',
                            stream=sys.stdout)
        asyncio.run(serve(args))
        return
    words = args.words
    res = control(Path(args.state), shlex.join(words), ctl_timeout(words))
    print(json.dumps(res, ensure_ascii=False, indent=1))
    sys.exit(0 if res.get('ok') else 1)


if __name__ == '__main__':
    main()
