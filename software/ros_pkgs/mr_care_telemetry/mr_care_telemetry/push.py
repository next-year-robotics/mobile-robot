"""보호자 웹 푸시(FCM HTTP v1). 대시보드를 띄워 두지 않아도 패드·폰에 알림이 뜨게 한다(2026-11-01).

- 받는 사람: `/access/{robot_id}`에 있는 uid마다 `/users/{uid}/push/{key}` = {token, …}. 대시보드가 "알림 켜기"에서 쓴다.
- 보내는 것: 외출 시작(trip.start)과 SOS(sos.request) 둘뿐이다(안부 알림. 종료·오류는 보내지 않는다, 사용자 결정).
- Spark 요금제라 Cloud Functions가 없다. 서비스 계정을 가진 이 에이전트가 직접 보낸다. FCM HTTP v1은 무료다.
- 실패해도 텔레메트리 기록(RTDB)은 막지 않는다. 별도 스레드에서 보내고, 오래된 알림(stale_s)은 버린다.
- 토큰이 만료·해지됐다고 FCM이 답하면(404 UNREGISTERED) 그 토큰을 지운다.
"""
from __future__ import annotations

import json
import logging
import queue
import socket
import threading
import time
import urllib.error
import urllib.request
from typing import Callable

from .rtdb import RtdbClient, RtdbError, ServiceAccountToken

log = logging.getLogger(__name__)

FCM_SCOPE = 'https://www.googleapis.com/auth/firebase.messaging'
MAX_TOKENS = 20


def josa_ro(word: str) -> str:
    """'(으)로'. 받침이 없거나 ㄹ이면 '로'."""
    if not word:
        return '로'
    c = ord(word[-1])
    if not 0xAC00 <= c <= 0xD7A3:
        return '로'
    jong = (c - 0xAC00) % 28
    return '로' if jong in (0, 8) else '으로'


def trip_start_text(senior: str, to_label: str) -> tuple[str, str]:
    return '외출 시작', f'{senior}께서 {to_label}{josa_ro(to_label)} 외출을 시작하셨어요.'


def sos_text(senior: str, reason: str) -> tuple[str, str]:
    if reason == 'manual':
        body = f'{senior}께서 도움을 요청하셨어요. 눌러서 위치를 확인하세요.'
    elif reason == 'tracking_lost':
        body = f'로봇이 {senior}을 놓쳤어요. 눌러서 위치를 확인하세요.'
    else:
        body = f'{senior} 곁의 로봇에 SOS가 발생했어요. 눌러서 위치를 확인하세요.'
    return 'SOS 긴급 알림', body


class FcmSender:
    """FCM HTTP v1 한 건 보내기. 블로킹이다."""

    def __init__(self, token: ServiceAccountToken, project_id: str | None = None, *, timeout_s: float = 10.0):
        self._token = token
        self._project_id = project_id
        self.timeout_s = timeout_s

    @property
    def project_id(self) -> str:
        if not self._project_id:
            self._token._load()
            self._project_id = self._token.project_id
        if not self._project_id:
            raise RtdbError('service account has no project_id')
        return self._project_id

    def send(self, message: dict, *, validate_only: bool = False) -> tuple[int, str]:
        """(HTTP 상태, FCM 오류 코드 또는 ''). 네트워크 오류는 RtdbError."""
        url = f'https://fcm.googleapis.com/v1/projects/{self.project_id}/messages:send'
        body = json.dumps({'message': message, 'validate_only': validate_only}, ensure_ascii=False).encode('utf-8')
        req = urllib.request.Request(url, data=body, method='POST', headers={
            'Content-Type': 'application/json; charset=utf-8', 'Authorization': f'Bearer {self._token.token()}'})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                return resp.status, ''
        except urllib.error.HTTPError as e:
            return e.code, _fcm_error(e.read()[:4000])
        except (urllib.error.URLError, socket.timeout, OSError) as e:
            raise RtdbError(f'FCM unreachable: {e}') from None


def _fcm_error(body: bytes) -> str:
    try:
        err = json.loads(body.decode('utf-8'))['error']
    except (ValueError, KeyError, TypeError):
        return body.decode('utf-8', 'replace')[:200]
    for d in err.get('details') or []:
        if isinstance(d, dict) and d.get('errorCode'):
            return str(d['errorCode'])
    return str(err.get('status') or err.get('message') or '')[:200]


def token_is_dead(status: int, code: str) -> bool:
    # INVALID_ARGUMENT는 메시지 형식 오류일 수도 있어 지우지 않는다(대시보드가 열릴 때마다 같은 키로 다시 쓴다)
    return status == 404 or code == 'UNREGISTERED'


class PushNotifier:
    """notify()는 어느 스레드에서든 부른다(큐에 넣기만 한다). 전송은 자기 스레드에서 한다."""

    def __init__(self, client: RtdbClient, sender: FcmSender, *, robot_id: str, link: str,
                 stale_s: float = 120.0, clock: Callable[[], float] = time.monotonic):
        self.client = client
        self.sender = sender
        self.robot_id = robot_id
        self.link = link
        self.stale_s = stale_s
        self.clock = clock
        self._q: queue.SimpleQueue = queue.SimpleQueue()
        self._thread: threading.Thread | None = None
        self.sent = 0

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name='care-telemetry-push', daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._q.put(None)

    def notify(self, kind: str, title: str, body: str, *, tag: str, urgent: bool = False,
               data: dict | None = None) -> None:
        self._q.put((self.clock(), kind, title, body, tag, urgent, dict(data or {})))

    def _run(self) -> None:
        while True:
            job = self._q.get()
            if job is None:
                return
            try:
                self.deliver(*job)
            except Exception:                # 전송 스레드는 죽지 않는다
                log.exception('push failed')

    def tokens(self) -> list[tuple[str, str, str]]:
        """[(uid, key, token)]. RTDB 관리자 읽기(규칙 우회)."""
        out: list[tuple[str, str, str]] = []
        access = self.client.get(f'access/{self.robot_id}') or {}
        for uid in access if isinstance(access, dict) else ():
            entries = self.client.get(f'users/{uid}/push') or {}
            for key, v in (entries.items() if isinstance(entries, dict) else ()):
                tok = v.get('token') if isinstance(v, dict) else None
                if isinstance(tok, str) and tok:
                    out.append((uid, key, tok))
        return out[:MAX_TOKENS]

    def message(self, token: str, kind: str, title: str, body: str, tag: str, urgent: bool, data: dict) -> dict:
        """data 전용 웹 푸시. 대시보드 서비스 워커(public/sw.js)가 title·body·url·tag·urgent로 알림을 그린다.
        notification 필드를 쓰지 않는 것은 문구·진동·누르면 열 주소(?sos=id)를 서비스 워커 한 곳에서 정하려는 것이다."""
        url = self.link + (f'?sos={data["sos_id"]}' if data.get('sos_id') else '')
        payload = {**{k: str(v) for k, v in data.items() if v is not None}, 'kind': kind, 'robot_id': self.robot_id,
                   'title': title, 'body': body, 'url': url, 'tag': tag, 'urgent': '1' if urgent else '0'}
        return {'token': token, 'data': payload,
                'webpush': {'headers': {'Urgency': 'high' if urgent else 'normal', 'TTL': str(int(self.stale_s))}}}

    def deliver(self, queued_at: float, kind: str, title: str, body: str, tag: str, urgent: bool,
                data: dict) -> int:
        if self.clock() - queued_at > self.stale_s:
            log.warning('push %s dropped: %.0f s old', kind, self.clock() - queued_at)
            return 0
        try:
            targets = self.tokens()
        except RtdbError as e:
            log.warning('push %s: cannot read tokens: %s', kind, e)
            return 0
        if not targets:
            log.info('push %s: no subscribed guardians', kind)
            return 0
        ok = 0
        for uid, key, tok in targets:
            try:
                status, code = self.sender.send(self.message(tok, kind, title, body, tag, urgent, data))
            except RtdbError as e:
                log.warning('push %s: %s', kind, e)
                return ok
            if 200 <= status < 300:
                ok += 1
            elif token_is_dead(status, code):
                log.info('push token of %s expired (%s %s): removing', uid[:6], status, code)
                try:
                    self.client.delete(f'users/{uid}/push/{key}')
                except RtdbError as e:
                    log.warning('push token removal failed: %s', e)
            else:
                log.warning('push %s to %s failed: %s %s', kind, uid[:6], status, code)
        self.sent += ok
        log.info('push %s: %d/%d delivered to FCM', kind, ok, len(targets))
        return ok
