"""Minimal single-peer WebSocket signaling server (answerer side).

Same JSON-over-WebSocket wire format as the camera streamer's signaling
(hello/welcome/offer/answer/candidate/bye), so the Quest reuses one signaling
client shape for both PeerConnections -- it just connects to a second port for
the arm. Hand-rolled RFC 6455 (glib-free): Upgrade handshake, unmasked text out,
masked in, ping->pong. Ported from streamer/src/webrtc_signaling.cpp.

Serial accept: one peer at a time; a new connection is served after the previous
one closes. Runs on its own accept thread; callbacks fire on that thread.
"""

from __future__ import annotations

import base64
import hashlib
import json
import select
import socket
import threading
from typing import Callable

_WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def _accept_key(client_key: str) -> str:
    digest = hashlib.sha1((client_key + _WS_GUID).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def _header_value(request: str, name_lower: str) -> str:
    for line in request.split("\r\n"):
        if ":" in line and line.split(":", 1)[0].strip().lower() == name_lower:
            return line.split(":", 1)[1].strip()
    return ""


def _encode_frame(payload: bytes, opcode: int) -> bytes:
    out = bytes([0x80 | opcode])
    n = len(payload)
    if n < 126:
        out += bytes([n])
    elif n <= 0xFFFF:
        out += bytes([126, (n >> 8) & 0xFF, n & 0xFF])
    else:
        out += bytes([127]) + n.to_bytes(8, "big")
    return out + payload


def _parse_frame(buf: bytes):
    """-> (consumed, opcode, payload) or (0, None, None) if incomplete."""
    if len(buf) < 2:
        return 0, None, None
    b1 = buf[1]
    masked = b1 & 0x80
    length = b1 & 0x7F
    pos = 2
    if length == 126:
        if len(buf) < 4:
            return 0, None, None
        length = (buf[2] << 8) | buf[3]
        pos = 4
    elif length == 127:
        if len(buf) < 10:
            return 0, None, None
        length = int.from_bytes(buf[2:10], "big")
        pos = 10
    mask = b"\x00\x00\x00\x00"
    if masked:
        if len(buf) < pos + 4:
            return 0, None, None
        mask = buf[pos:pos + 4]
        pos += 4
    if len(buf) < pos + length:
        return 0, None, None
    payload = buf[pos:pos + length]
    if masked:
        payload = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
    return pos + length, buf[0] & 0x0F, payload


class SignalingServer:
    def __init__(self, port: int, allowed_source_ip: str | None = None) -> None:
        self._port = port
        self._allowed = allowed_source_ip or ""
        self._on_message: Callable[[dict], None] | None = None
        self._on_disconnect: Callable[[], None] | None = None
        self._on_peer: Callable[[str], None] | None = None
        self._listen: socket.socket | None = None
        self._peer: socket.socket | None = None
        self._peer_ip = ""
        self._ws_ready = False
        self._running = False
        self._send_lock = threading.Lock()
        self._thread: threading.Thread | None = None

    def set_handlers(self, on_message, on_disconnect, on_peer=None) -> None:
        self._on_message = on_message
        self._on_disconnect = on_disconnect
        self._on_peer = on_peer

    @property
    def peer_ip(self) -> str:
        return self._peer_ip

    def start(self) -> None:
        self._listen = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listen.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listen.bind(("0.0.0.0", self._port))
        self._listen.listen(1)
        self._running = True
        self._thread = threading.Thread(target=self._accept_loop, name="arm-signaling", daemon=True)
        self._thread.start()

    def _accept_loop(self) -> None:
        while self._running:
            r, _, _ = select.select([self._listen], [], [], 0.25)
            if not r:
                continue
            try:
                conn, addr = self._listen.accept()
            except OSError:
                continue
            if self._allowed and addr[0] != self._allowed:
                conn.close()
                continue
            self._peer = conn
            self._peer_ip = addr[0]
            if self._on_peer:
                self._on_peer(self._peer_ip)
            try:
                self._serve_peer(conn)
            finally:
                self._ws_ready = False
                try:
                    conn.close()
                except OSError:
                    pass
                self._peer = None
                if self._on_disconnect:
                    self._on_disconnect()

    def _serve_peer(self, conn: socket.socket) -> None:
        buffer = b""
        conn.settimeout(0.25)
        # Phase 1: HTTP Upgrade.
        while self._running and b"\r\n\r\n" not in buffer:
            if len(buffer) > 16384:
                return
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if not chunk:
                return
            buffer += chunk
        head, _, buffer = buffer.partition(b"\r\n\r\n")
        request = head.decode("latin-1", errors="replace")
        key = _header_value(request, "sec-websocket-key")
        if not key:
            return
        resp = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {_accept_key(key)}\r\n\r\n"
        )
        with self._send_lock:
            try:
                conn.sendall(resp.encode("ascii"))
            except OSError:
                return
        self._ws_ready = True

        # Phase 2: WebSocket frames.
        while self._running:
            consumed, opcode, payload = _parse_frame(buffer)
            while consumed:
                buffer = buffer[consumed:]
                if opcode == 0x8:                       # close
                    return
                if opcode == 0x9:                       # ping -> pong
                    with self._send_lock:
                        try:
                            conn.sendall(_encode_frame(payload, 0xA))
                        except OSError:
                            return
                elif opcode == 0x1:                     # text
                    self._dispatch(payload)
                consumed, opcode, payload = _parse_frame(buffer)
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if not chunk:
                return
            buffer += chunk

    def _dispatch(self, payload: bytes) -> None:
        try:
            message = json.loads(payload.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return  # drop malformed line
        if isinstance(message, dict) and message.get("type") and self._on_message:
            self._on_message(message)

    def send(self, message: dict) -> None:
        conn = self._peer
        if conn is None or not self._ws_ready:
            return
        message.setdefault("v", 1)
        frame = _encode_frame(json.dumps(message).encode("utf-8"), 0x1)
        with self._send_lock:
            try:
                conn.sendall(frame)
            except OSError:
                pass

    def stop(self) -> None:
        self._running = False
        if self._peer is not None:
            try:
                self._peer.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._listen is not None:
            try:
                self._listen.close()
            except OSError:
                pass
