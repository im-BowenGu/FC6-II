"""Unix-socket streaming of macroscopic velocity slices."""

from __future__ import annotations

import json
import socket
import struct
from pathlib import Path
from typing import Any

import numpy as np

MAGIC = b"CFD1"
_MAX_HEADER = 1 << 20


def _json_default(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"cannot serialise {type(value).__name__} in CFD frame header")


class _Client:
    __slots__ = ("sock", "pending")

    def __init__(self, sock: socket.socket) -> None:
        sock.setblocking(False)
        self.sock = sock
        self.pending = bytearray()


class FieldStreamServer:
    """Non-blocking broadcaster; slow clients are disconnected, never awaited."""

    def __init__(self, socket_path: str | Path, max_pending_bytes: int = 16 << 20) -> None:
        self.path = Path(socket_path).expanduser()
        self.max_pending_bytes = max_pending_bytes
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.exists():
            self.path.unlink()
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._sock.bind(str(self.path))
        self._sock.listen(16)
        self._sock.setblocking(False)
        self._clients: list[_Client] = []
        self._closed = False

    def _accept(self) -> None:
        while True:
            try:
                conn, _ = self._sock.accept()
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                return
            self._clients.append(_Client(conn))

    def _drop(self, client: _Client) -> None:
        try:
            self._clients.remove(client)
        except ValueError:
            pass
        try:
            client.sock.close()
        except OSError:
            pass

    def _flush(self, client: _Client) -> None:
        while client.pending:
            try:
                sent = client.sock.send(client.pending)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                self._drop(client)
                return
            if sent == 0:
                self._drop(client)
                return
            del client.pending[:sent]

    def publish(self, header: dict[str, Any], array: np.ndarray) -> None:
        self._accept()
        if not self._clients:
            return
        nx = int(header["nx"])
        ny = int(header["ny"])
        values = np.asarray(array, dtype="<f4")
        if values.shape != (nx, ny, 3):
            raise ValueError(f"velocity slice shape {values.shape} != {(nx, ny, 3)}")
        raw_header = json.dumps(
            header, separators=(",", ":"), default=_json_default
        ).encode("utf-8")
        if len(raw_header) > _MAX_HEADER:
            raise ValueError("CFD frame header too large")
        frame = MAGIC + struct.pack("<I", len(raw_header)) + raw_header + \
            values.tobytes(order="C")
        for client in list(self._clients):
            client.pending.extend(frame)
            if len(client.pending) > self.max_pending_bytes:
                self._drop(client)
                continue
            self._flush(client)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for client in list(self._clients):
            self._drop(client)
        try:
            self._sock.close()
        except OSError:
            pass
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass

    def __enter__(self) -> FieldStreamServer:
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.close()


class FieldStreamClient:
    def __init__(self, socket_path: str | Path) -> None:
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._sock.connect(str(Path(socket_path).expanduser()))

    def _recv_exact(self, count: int) -> bytes:
        buf = bytearray()
        while len(buf) < count:
            chunk = self._sock.recv(count - len(buf))
            if not chunk:
                raise ConnectionError("CFD stream closed")
            buf.extend(chunk)
        return bytes(buf)

    def recv(self) -> tuple[dict[str, Any], np.ndarray]:
        if self._recv_exact(4) != MAGIC:
            raise ValueError("bad CFD frame magic")
        header_len = struct.unpack("<I", self._recv_exact(4))[0]
        if header_len == 0 or header_len > _MAX_HEADER:
            raise ValueError("bad CFD frame header length")
        header = json.loads(self._recv_exact(header_len).decode("utf-8"))
        nx = int(header["nx"])
        ny = int(header["ny"])
        count = nx * ny * 3
        values = np.frombuffer(
            self._recv_exact(count * 4), dtype="<f4"
        ).reshape(nx, ny, 3).copy()
        return header, values

    def close(self) -> None:
        self._sock.close()

    def __enter__(self) -> FieldStreamClient:
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.close()
