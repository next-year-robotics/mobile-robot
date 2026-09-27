"""websockets 전송 계층. JSON text frame만, frame 상한 16 KiB.

RPi5(Ubuntu 24.04 apt `python3-websockets` 10.4)와 개발 PC(uv 최신 websockets ≥ 13)를 모두 지원한다.
≥ 13은 `websockets.asyncio.server`, 그 이전은 legacy `websockets.server`를 쓴다.
"""
from __future__ import annotations

import asyncio
import logging
from typing import AsyncIterator

import websockets
from websockets.exceptions import ConnectionClosed

try:  # websockets >= 13
    from websockets.asyncio.server import serve as _serve
    NEW_API = True
except ImportError:  # pragma: no cover - websockets 10.x (RPi5 apt): legacy 구현
    from websockets import serve as _serve
    NEW_API = False

from .bridge import BridgeCore

log = logging.getLogger(__name__)
WEBSOCKETS_VERSION = getattr(websockets, '__version__', '?')


class WsConnection:
    """websockets 연결을 BridgeCore.Connection으로 감싼다."""

    def __init__(self, ws):
        self._ws = ws
        if NEW_API:
            self.path = ws.request.path
        else:
            self.path = ws.path
        addr = ws.remote_address
        self.remote = f'{addr[0]}:{addr[1]}' if addr else '?'

    async def send(self, text) -> None:
        await self._ws.send(text)

    async def close(self, code: int, reason: str) -> None:
        await self._ws.close(code=code, reason=reason)

    def abort(self) -> None:
        transport = getattr(self._ws, 'transport', None)
        if transport is not None:
            transport.abort()

    async def __aiter__(self) -> AsyncIterator[str | bytes]:
        try:
            async for message in self._ws:
                yield message
        except ConnectionClosed:
            return


class BridgeServer:
    """`async with BridgeServer(core, host, port) as srv:` 또는 start()/stop()."""

    def __init__(self, core: BridgeCore, host: str = '0.0.0.0', port: int = 8765):
        self.core = core
        self.host = host
        self.port = port
        self._server = None

    async def _handler(self, ws, path=None) -> None:
        await self.core.handle(WsConnection(ws))

    async def start(self) -> None:
        await self.core.start()
        self._server = await _serve(
            self._handler, self.host, self.port,
            max_size=self.core.cfg.max_frame_bytes,
            close_timeout=self.core.cfg.close_timeout_s,
        )
        socks = getattr(self._server, 'sockets', None) or []
        if socks:
            self.port = socks[0].getsockname()[1]
        log.info('phone-bridge listening on ws://%s:%d%s (websockets %s)', self.host, self.port,
                 self.core.cfg.path, WEBSOCKETS_VERSION)

    async def stop(self) -> None:
        await self.core.stop()
        if self._server is not None:
            self._server.close()
            try:
                await asyncio.wait_for(self._server.wait_closed(), self.core.cfg.close_timeout_s + 2.0)
            except asyncio.TimeoutError:
                log.warning('server did not close in time')
            self._server = None

    async def __aenter__(self) -> 'BridgeServer':
        await self.start()
        return self

    async def __aexit__(self, *exc) -> None:
        await self.stop()
