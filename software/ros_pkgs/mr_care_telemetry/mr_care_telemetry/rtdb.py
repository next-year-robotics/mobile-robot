"""Firebase RTDB REST 클라이언트(표준 라이브러리 + python3-cryptography).

- 실제 DB: 서비스 계정 JSON으로 OAuth2 JWT bearer 토큰을 만들어 `Authorization: Bearer`로 붙인다
  (care-telemetry README 5장 "REST로 직접 쓸 경우 서비스 계정 OAuth2 토큰"). firebase-admin은 쓰지 않는다
  (RPi5에서 pip이 필요해서). 서비스 계정 파일은 RPi5 `~/.config/mr_care_telemetry/`에만 둔다.
- 에뮬레이터: `FIREBASE_DATABASE_EMULATOR_HOST=host:port`면 `http://host:port/<path>.json?ns=<namespace>`에
  `Bearer owner`(관리자, 규칙 우회)로 쓴다.
블로킹 호출이다. 에이전트의 전송 스레드에서만 부른다.
"""
from __future__ import annotations

import base64
import json
import os
import socket
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any

SCOPES = 'https://www.googleapis.com/auth/firebase.database https://www.googleapis.com/auth/userinfo.email'
# 요청 내용 자체가 잘못된 경우(키 규칙 위반·크기 초과)만 영구 오류다. 다시 보내도 같으므로 큐에서 버린다.
# 그 밖의 4xx(404 = DB URL 오류, 401/403 = 토큰, 402/423 = 요금제·DB 정지 등)는 설정·서버 상태 문제라
# 고쳐지면 다시 보낼 수 있게 일시 오류로 두고 큐에 남긴다.
PERMANENT_STATUS = {400, 413}


class RtdbError(Exception):

    def __init__(self, message: str, *, status: int | None = None, transient: bool = True):
        super().__init__(message)
        self.status = status
        self.transient = transient


def _b64url(data: bytes) -> str:
    return base64.urlsafe_b64encode(data).rstrip(b'=').decode('ascii')


class ServiceAccountToken:
    """서비스 계정 → OAuth2 access token. 만료 60 s 전에 새로 받는다."""

    def __init__(self, path: str, *, timeout_s: float = 10.0):
        self.path = os.path.expanduser(path)
        self._timeout_s = timeout_s
        self._lock = threading.Lock()
        self._token: str | None = None
        self._expires_at = 0.0
        self.client_email = self.token_uri = None
        self._key_pem = b''

    def _load(self) -> None:
        """파일은 처음 토큰이 필요할 때 읽는다. 없으면 일시 오류로 올려, 에이전트가 큐에 쌓으며 기다리게 한다
        (파일을 나중에 넣어도 재시작 없이 붙는다)."""
        if self._key_pem:
            return
        try:
            with open(self.path, encoding='utf-8') as f:
                info = json.load(f)
            self.client_email = info['client_email']
            self.token_uri = info.get('token_uri', 'https://oauth2.googleapis.com/token')
            self._key_pem = info['private_key'].encode('utf-8')
        except FileNotFoundError:
            raise RtdbError(f'service account not found: {self.path}') from None
        except (OSError, ValueError, KeyError) as e:
            raise RtdbError(f'service account unreadable ({self.path}): {e!r}') from None

    def assertion(self, now: float | None = None) -> str:
        from cryptography.hazmat.primitives import hashes, serialization
        from cryptography.hazmat.primitives.asymmetric import padding
        self._load()
        now = int(now if now is not None else time.time())
        header = {'alg': 'RS256', 'typ': 'JWT'}
        claims = {'iss': self.client_email, 'scope': SCOPES, 'aud': self.token_uri, 'iat': now, 'exp': now + 3600}
        signing_input = (_b64url(json.dumps(header, separators=(',', ':')).encode()) + '.'
                         + _b64url(json.dumps(claims, separators=(',', ':')).encode()))
        try:
            key = serialization.load_pem_private_key(self._key_pem, password=None)
        except (ValueError, TypeError) as e:
            raise RtdbError(f'service account private_key is invalid: {e}') from None
        sig = key.sign(signing_input.encode('ascii'), padding.PKCS1v15(), hashes.SHA256())
        return signing_input + '.' + _b64url(sig)

    def token(self) -> str:
        with self._lock:
            if self._token and time.monotonic() < self._expires_at - 60:
                return self._token
            self._load()
            body = urllib.parse.urlencode({
                'grant_type': 'urn:ietf:params:oauth:grant-type:jwt-bearer',
                'assertion': self.assertion(),
            }).encode('ascii')
            req = urllib.request.Request(self.token_uri, data=body, method='POST',
                                         headers={'Content-Type': 'application/x-www-form-urlencoded'})
            try:
                with urllib.request.urlopen(req, timeout=self._timeout_s) as resp:
                    data = json.loads(resp.read().decode('utf-8'))
            except urllib.error.HTTPError as e:
                # 토큰 실패(키 오류·시계 어긋남 등)는 쓰려던 작업의 잘못이 아니다: 항상 일시 오류로 두어 큐를 지킨다
                raise RtdbError(f'token endpoint {e.code} {_token_error_detail(e.read()[:1000])}',
                                status=e.code) from None
            except (urllib.error.URLError, socket.timeout, OSError) as e:
                raise RtdbError(f'token endpoint unreachable: {e}') from None
            self._token = data['access_token']
            self._expires_at = time.monotonic() + float(data.get('expires_in', 3600))
            return self._token

    def invalidate(self) -> None:
        with self._lock:
            self._token = None


def _error_detail(body: bytes) -> str:
    """RTDB 오류 본문 `{"error": "..."}`에서 문구만 한 줄로 꺼낸다(로그·recent_errors용)."""
    text = body.decode('utf-8', 'replace')
    try:
        msg = json.loads(text).get('error')
        if isinstance(msg, str):
            text = msg
    except (ValueError, AttributeError):
        pass
    return ' '.join(text.split())[:200]


def _token_error_detail(body: bytes) -> str:
    """OAuth 오류 본문 `{"error": "invalid_grant", "error_description": "..."}` → 한 줄."""
    try:
        d = json.loads(body.decode('utf-8', 'replace'))
        return ' '.join(f"{d.get('error', '')}: {d.get('error_description', '')}".split())[:200]
    except (ValueError, AttributeError):
        return _error_detail(body)


def namespace_of(database_url: str) -> str:
    return urllib.parse.urlparse(database_url).hostname.split('.')[0]


class RtdbClient:

    def __init__(self, database_url: str, *, emulator_host: str | None = None, namespace: str | None = None,
                 token: ServiceAccountToken | None = None, timeout_s: float = 5.0):
        self.database_url = database_url.rstrip('/')
        self.emulator_host = emulator_host
        self.namespace = namespace or namespace_of(database_url)
        self._token = token
        self.timeout_s = timeout_s
        if not emulator_host and token is None:
            raise ValueError('real database needs a service account (or set FIREBASE_DATABASE_EMULATOR_HOST)')

    @classmethod
    def from_settings(cls, database_url: str, service_account_path: str | None, *,
                      emulator_host: str | None = None, timeout_s: float = 5.0) -> 'RtdbClient':
        emulator_host = emulator_host or os.environ.get('FIREBASE_DATABASE_EMULATOR_HOST') or None
        token = None
        if not emulator_host:
            token = ServiceAccountToken(service_account_path)
        return cls(database_url, emulator_host=emulator_host, token=token, timeout_s=timeout_s)

    @property
    def target(self) -> str:
        return f'emulator http://{self.emulator_host} ns={self.namespace}' if self.emulator_host \
            else self.database_url

    def url(self, path: str, params: dict | None = None) -> str:
        params = dict(params or {})
        path = urllib.parse.quote(path.strip('/'), safe='/-_~')
        if self.emulator_host:
            params['ns'] = self.namespace
            base = f'http://{self.emulator_host}'
        else:
            base = self.database_url
        query = ('?' + urllib.parse.urlencode(params)) if params else ''
        return f'{base}/{path}.json{query}'

    def request(self, method: str, path: str, value: Any = None, params: dict | None = None) -> Any:
        params = dict(params or {})
        if method in ('PUT', 'PATCH', 'DELETE', 'POST'):
            params.setdefault('print', 'silent')
        data = None
        if method in ('PUT', 'PATCH', 'POST'):
            try:
                data = json.dumps(value, ensure_ascii=False, allow_nan=False).encode('utf-8')
            except (TypeError, ValueError) as e:
                raise RtdbError(f'cannot encode {path}: {e}', transient=False) from None
        headers = {'Content-Type': 'application/json'}
        if self.emulator_host:
            headers['Authorization'] = 'Bearer owner'
        else:
            headers['Authorization'] = f'Bearer {self._token.token()}'
        req = urllib.request.Request(self.url(path, params), data=data, method=method, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                body = resp.read()
        except urllib.error.HTTPError as e:
            detail = _error_detail(e.read()[:1000])
            if e.code == 401 and self._token is not None:
                self._token.invalidate()
            raise RtdbError(f'{method} {path} -> {e.code} {detail}', status=e.code,
                            transient=e.code not in PERMANENT_STATUS) from None
        except (urllib.error.URLError, socket.timeout, ConnectionError, OSError) as e:
            raise RtdbError(f'{method} {path}: {getattr(e, "reason", e)}') from None
        if not body:
            return None
        return json.loads(body.decode('utf-8'))

    def put(self, path: str, value: Any) -> None:
        self.request('PUT', path, value)

    def patch(self, path: str, value: dict) -> None:
        self.request('PATCH', path, value)

    def delete(self, path: str) -> None:
        self.request('DELETE', path)

    def get(self, path: str, **params) -> Any:
        return self.request('GET', path, params=params)
