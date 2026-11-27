"""LidarRender TCP 消息协议。

消息格式：``!4sII`` 前缀（magic ``LDSM``、JSON 字节数、二进制载荷字节数），
后接 UTF-8 JSON 头与原始二进制载荷。协议不依赖 ROS，可在普通 Python 环境运行。
"""

import json
import math
import socket
import struct
import time
from typing import Any, Dict, List, Optional, Tuple

MAGIC = b"LDSM"
PROTOCOL_VERSION = 1
MESSAGE_PREFIX = struct.Struct("!4sII")
MAX_HEADER_BYTES = 16 * 1024
MAX_PAYLOAD_BYTES = 16 * 1024 * 1024
MESSAGE_DEADLINE_SECONDS = 2.0
FRAME_ID = "world"
POINT_BYTES = 16

Message = Tuple[Dict[str, Any], bytes]


def encode_message(header: Dict[str, Any], payload: bytes = b"") -> bytes:
    """把消息头与载荷编码为可直接写入 socket 的字节串。"""
    header_bytes = json.dumps(header, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if len(header_bytes) > MAX_HEADER_BYTES:
        raise ValueError("消息头超过 %d 字节" % MAX_HEADER_BYTES)
    if len(payload) > MAX_PAYLOAD_BYTES:
        raise ValueError("消息载荷超过 %d 字节" % MAX_PAYLOAD_BYTES)
    return MESSAGE_PREFIX.pack(MAGIC, len(header_bytes), len(payload)) + header_bytes + payload


def send_message(sock: socket.socket, header: Dict[str, Any], payload: bytes = b"",
                 timeout: Optional[float] = None) -> None:
    """发送一条完整消息；timeout 限制整条消息的发送时间。"""
    data = encode_message(header, payload)
    original_timeout = sock.gettimeout()
    try:
        if timeout is not None:
            sock.settimeout(timeout)
        sock.sendall(data)
    except socket.timeout as error:
        raise TimeoutError("发送消息超时") from error
    finally:
        sock.settimeout(original_timeout)


def recv_exact(sock: socket.socket, size: int, deadline: Optional[float] = None) -> bytes:
    """读取恰好 size 字节。deadline 为 time.monotonic() 时间点，None 表示不限时。"""
    if size == 0:
        return b""
    chunks = []  # type: List[bytes]
    remaining = size
    original_timeout = sock.gettimeout()
    try:
        while remaining > 0:
            if deadline is not None:
                remaining_time = deadline - time.monotonic()
                if remaining_time <= 0:
                    raise TimeoutError("读取消息超时")
                sock.settimeout(remaining_time)
            try:
                chunk = sock.recv(min(remaining, 65536))
            except socket.timeout as error:
                raise TimeoutError("读取消息超时") from error
            if not chunk:
                raise EOFError("连接在读取 %d/%d 字节后关闭" % (size - remaining, size))
            chunks.append(chunk)
            remaining -= len(chunk)
    finally:
        sock.settimeout(original_timeout)
    return b"".join(chunks)


def receive_message(sock: socket.socket) -> Message:
    """读取一条消息。空闲等待不限时；前缀读完后整条消息共用 2 秒 deadline。"""
    prefix = recv_exact(sock, MESSAGE_PREFIX.size)
    magic, header_length, payload_length = MESSAGE_PREFIX.unpack(prefix)
    if magic != MAGIC:
        raise ValueError("非法消息 magic: %r" % (magic,))
    if header_length > MAX_HEADER_BYTES:
        raise ValueError("消息头长度 %d 超过上限 %d" % (header_length, MAX_HEADER_BYTES))
    if payload_length > MAX_PAYLOAD_BYTES:
        raise ValueError("消息载荷长度 %d 超过上限 %d" % (payload_length, MAX_PAYLOAD_BYTES))

    deadline = time.monotonic() + MESSAGE_DEADLINE_SECONDS
    header_bytes = recv_exact(sock, header_length, deadline)
    try:
        header = json.loads(header_bytes.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("消息头不是合法 JSON: %s" % error) from error
    if not isinstance(header, dict):
        raise ValueError("消息头必须是 JSON 对象")
    validate_header(header, payload_length)
    payload = recv_exact(sock, payload_length, deadline)
    return header, payload


def validate_header(header: Dict[str, Any], payload_length: int) -> None:
    """校验消息头语义与载荷长度；在读取载荷之前调用。"""
    if header.get("version") != PROTOCOL_VERSION:
        raise ValueError("不支持的消息版本: %r" % (header.get("version"),))
    message_type = header.get("type")
    if message_type == "pose":
        validate_pose_header(header, payload_length)
    elif message_type == "cloud":
        validate_cloud_header(header, payload_length)
    else:
        raise ValueError("未知消息类型: %r" % (message_type,))


def is_integer(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def require_integer(header: Dict[str, Any], key: str) -> int:
    value = header.get(key)
    if not is_integer(value):
        raise ValueError("%s 必须是整数: %r" % (key, value))
    return value


def require_finite_number(header: Dict[str, Any], key: str) -> float:
    value = header.get(key)
    if not is_finite_number(value):
        raise ValueError("%s 必须是有限数值: %r" % (key, value))
    return value


def require_finite_vector(header: Dict[str, Any], key: str, length: int) -> List[float]:
    value = header.get(key)
    if not isinstance(value, list) or len(value) != length:
        raise ValueError("%s 必须是长度为 %d 的数组: %r" % (key, length, value))
    for item in value:
        if not is_finite_number(item):
            raise ValueError("%s 必须是有限数值: %r" % (key, value))
    return value


def require_non_empty_string(header: Dict[str, Any], key: str) -> str:
    value = header.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError("%s 必须是非空字符串: %r" % (key, value))
    return value


def validate_stamp(header: Dict[str, Any]) -> None:
    if require_integer(header, "stamp_sec") < 0:
        raise ValueError("stamp_sec 不能为负")
    stamp_nsec = require_integer(header, "stamp_nsec")
    if not 0 <= stamp_nsec < 1000000000:
        raise ValueError("stamp_nsec 超出范围: %r" % (stamp_nsec,))


def validate_pose_header(header: Dict[str, Any], payload_length: int) -> None:
    if payload_length != 0:
        raise ValueError("pose 消息不应携带载荷")
    if require_integer(header, "seq") < 0:
        raise ValueError("seq 不能为负")
    validate_stamp(header)
    require_finite_vector(header, "position", 3)
    orientation = require_finite_vector(header, "orientation", 4)
    if all(component == 0 for component in orientation):
        raise ValueError("orientation 四元数不能为零")


def validate_cloud_header(header: Dict[str, Any], payload_length: int) -> None:
    if require_integer(header, "frame_seq") < 0:
        raise ValueError("frame_seq 不能为负")
    if require_integer(header, "pose_seq") < 0:
        raise ValueError("pose_seq 不能为负")
    validate_stamp(header)
    require_non_empty_string(header, "frame_id")
    point_count = require_integer(header, "point_count")
    if point_count < 0:
        raise ValueError("point_count 不能为负: %r" % (point_count,))
    point_bytes = require_integer(header, "point_bytes")
    if point_bytes < 0:
        raise ValueError("point_bytes 不能为负: %r" % (point_bytes,))
    expected = point_count * POINT_BYTES
    if point_bytes != expected:
        raise ValueError("point_bytes 与 point_count 不符: %d != %d*%d" % (
            point_bytes, point_count, POINT_BYTES))
    if point_bytes > MAX_PAYLOAD_BYTES:
        raise ValueError("point_bytes 超过载荷上限: %d" % (point_bytes,))
    if payload_length != point_bytes:
        raise ValueError("载荷长度与 point_bytes 不符: %d != %d" % (
            payload_length, point_bytes))
