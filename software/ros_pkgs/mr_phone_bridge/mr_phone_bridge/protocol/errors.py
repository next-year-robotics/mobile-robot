"""phone-bridge/1 오류 코드(PROTOCOL.md 8.3). 11개로 고정."""
from __future__ import annotations

INVALID_MESSAGE = 'INVALID_MESSAGE'
UNSUPPORTED_TYPE = 'UNSUPPORTED_TYPE'
UNSUPPORTED_VERSION = 'UNSUPPORTED_VERSION'
NOT_READY = 'NOT_READY'
UNKNOWN_PLACE = 'UNKNOWN_PLACE'
INVALID_STATE = 'INVALID_STATE'
CATALOG_CONFLICT = 'CATALOG_CONFLICT'
PLACE_LIMIT = 'PLACE_LIMIT'
PLACE_IN_USE = 'PLACE_IN_USE'
PERMISSION_DENIED = 'PERMISSION_DENIED'
INTERNAL_ERROR = 'INTERNAL_ERROR'

# 코드별 기본 retryable. 앱 동작표(PROTOCOL.md 8.3)에서 "재시도 허용"인 것만 True.
DEFAULT_RETRYABLE = {
    INVALID_MESSAGE: False,
    UNSUPPORTED_TYPE: False,
    UNSUPPORTED_VERSION: False,
    NOT_READY: True,
    UNKNOWN_PLACE: False,
    INVALID_STATE: False,
    CATALOG_CONFLICT: True,
    PLACE_LIMIT: False,
    PLACE_IN_USE: False,
    PERMISSION_DENIED: False,
    INTERNAL_ERROR: True,
}
ALL_CODES = frozenset(DEFAULT_RETRYABLE)

MAX_MESSAGE_CHARS = 200


class ProtocolError(Exception):
    """앱에 `error`로 돌려줄 거절. `close=True`면 응답 후 연결을 닫는다.

    message는 앱 디버그 로그에 남는다. stack trace·파일 경로·토큰을 넣지 않는다.
    """

    def __init__(self, code: str, message: str, *, retryable: bool | None = None,
                 reply_to: str | None = None, close: bool = False):
        if code not in ALL_CODES:
            raise ValueError(f'unknown error code {code}')
        super().__init__(f'{code}: {message}')
        self.code = code
        self.message = message[:MAX_MESSAGE_CHARS]
        self.retryable = DEFAULT_RETRYABLE[code] if retryable is None else retryable
        self.reply_to = reply_to
        self.close = close

    def payload(self) -> dict:
        return {'code': self.code, 'message': self.message, 'retryable': self.retryable}
