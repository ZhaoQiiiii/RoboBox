"""Length-prefixed JSON-header frames between the ROS tee and the Rerun sink."""

import json
import socket
import struct

MAGIC = b"RBK1"
MAX_HEADER_BYTES = 16384
_PREFIX = struct.Struct("!4sII")


def send_frame(sock: socket.socket, header: dict, payload: bytes = b"") -> None:
    """Write one frame: magic, header length, payload length, JSON header, payload."""
    raw = json.dumps(header, separators=(",", ":")).encode("utf-8")
    if len(raw) > MAX_HEADER_BYTES:
        raise ValueError(f"frame header too large: {len(raw)} bytes")
    sock.sendall(_PREFIX.pack(MAGIC, len(raw), len(payload)) + raw + payload)


def recv_frame(sock: socket.socket) -> tuple:
    """Read one frame; raise ValueError on bad magic, oversized header or short body."""
    prefix = _read_exact(sock, _PREFIX.size)
    magic, header_len, payload_len = _PREFIX.unpack(prefix)
    if magic != MAGIC:
        raise ValueError(f"bad frame magic: {magic!r}")
    if header_len > MAX_HEADER_BYTES:
        raise ValueError(f"frame header too large: {header_len} bytes")
    header = json.loads(_read_exact(sock, header_len).decode("utf-8"))
    payload = _read_exact(sock, payload_len)
    return header, payload


def _read_exact(sock: socket.socket, size: int) -> bytes:
    chunks = []
    remaining = size
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ValueError("connection closed before frame completed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)
