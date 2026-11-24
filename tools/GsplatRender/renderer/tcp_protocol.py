"""GsplatRender TCP 消息协议。

消息格式：``!4sII`` 前缀（magic ``GSPT``、JSON 字节数、二进制载荷字节数），
后接 UTF-8 JSON 头与原始二进制载荷。协议不依赖 ROS，可在普通 Python 环境运行。
"""

import json
import math
import socket
import struct
import time
from typing import Any, Dict, List, Optional, Tuple

MAGIC = b"GSPT"
PROTOCOL_VERSION = 1
MESSAGE_PREFIX = struct.Struct("!4sII")
MAX_HEADER_BYTES = 16 * 1024
MAX_PAYLOAD_BYTES = 64 * 1024 * 1024
MESSAGE_DEADLINE_SECONDS = 2.0

RGB_ENCODING = "rgb8"
DEPTH_ENCODING = "16UC1"
FRAME_ID = "camera"

Message = Tuple[Dict[str, Any], bytes]


def encode_message(header: Dict[str, Any], payload: bytes = b"") -> bytes:
    """把消息头与载荷编码为可直接写入 socket 的字节串。只做长度与 JSON 校验。"""
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
    elif message_type == "camera_info":
        validate_camera_info_header(header, payload_length)
    elif message_type == "frame":
        validate_frame_header(header, payload_length)
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


def require_positive_integer(header: Dict[str, Any], key: str) -> int:
    value = require_integer(header, key)
    if value <= 0:
        raise ValueError("%s 必须为正整数: %r" % (key, value))
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


def validate_camera_info_header(header: Dict[str, Any], payload_length: int) -> None:
    if payload_length != 0:
        raise ValueError("camera_info 消息不应携带载荷")
    require_positive_integer(header, "width")
    require_positive_integer(header, "height")
    for key in ("fx", "fy", "cx", "cy"):
        require_finite_number(header, key)
    require_non_empty_string(header, "frame_id")
    cam_names = header.get("cam_names")
    if not isinstance(cam_names, list) or not cam_names:
        raise ValueError("cam_names 必须是非空数组: %r" % (cam_names,))
    for cam_name in cam_names:
        if not isinstance(cam_name, str) or not cam_name:
            raise ValueError("cam_names 必须由非空字符串组成: %r" % (cam_names,))
    require_non_empty_string(header, "rgb_encoding")
    require_non_empty_string(header, "depth_encoding")


def validate_frame_header(header: Dict[str, Any], payload_length: int) -> None:
    if require_integer(header, "frame_seq") < 0:
        raise ValueError("frame_seq 不能为负")
    if require_integer(header, "pose_seq") < 0:
        raise ValueError("pose_seq 不能为负")
    validate_stamp(header)
    width = require_positive_integer(header, "width")
    height = require_positive_integer(header, "height")
    if header.get("rgb_encoding") != RGB_ENCODING:
        raise ValueError("不支持的 RGB 编码: %r" % (header.get("rgb_encoding"),))
    if header.get("depth_encoding") != DEPTH_ENCODING:
        raise ValueError("不支持的深度编码: %r" % (header.get("depth_encoding"),))
    rgb_bytes = require_integer(header, "rgb_bytes")
    depth_bytes = require_integer(header, "depth_bytes")
    if rgb_bytes != width * height * 3:
        raise ValueError("rgb_bytes 与尺寸不符: %d != %d*%d*3" % (rgb_bytes, width, height))
    if depth_bytes != width * height * 2:
        raise ValueError("depth_bytes 与尺寸不符: %d != %d*%d*2" % (depth_bytes, width, height))
    if rgb_bytes + depth_bytes != payload_length:
        raise ValueError("载荷长度与 rgb_bytes+depth_bytes 不符: %d != %d" % (
            payload_length, rgb_bytes + depth_bytes))
