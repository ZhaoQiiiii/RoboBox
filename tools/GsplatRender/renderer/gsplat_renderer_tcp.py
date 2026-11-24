"""纯 TCP 的 3DGS 渲染服务端。

位姿连接（bridge → renderer）接收 pose 消息；图像连接（renderer → bridge）先发送
camera_info，再持续发送 frame。渲染、位姿接收、帧发送分别在不同线程运行。
本模块顶层只依赖标准库与 tcp_protocol，GPU 核心在 main() 中按需导入。
"""

import argparse
import logging
import signal
import socket
import sys
import threading
import time
import traceback
from collections import namedtuple
from pathlib import Path
from typing import Any, List, Optional

from tcp_protocol import (
    DEPTH_ENCODING,
    MAX_PAYLOAD_BYTES,
    MESSAGE_DEADLINE_SECONDS,
    RGB_ENCODING,
    receive_message,
    send_message,
)

PoseSample = namedtuple(
    "PoseSample", "connection_id seq stamp_sec stamp_nsec position orientation")
FrameSample = namedtuple("FrameSample", "serial header payload")

LOGGER = logging.getLogger("gsplat_renderer_tcp")

ACCEPT_TIMEOUT_SECONDS = 0.5
THREAD_JOIN_SECONDS = 3.0


class TcpRenderer:
    def __init__(self, core: Any, host: str = "127.0.0.1",
                 pose_port: int = 19080, frame_port: int = 19081) -> None:
        self.core = core
        self.host = host
        self.pose_port = pose_port
        self.frame_port = frame_port

        self.stop_event = threading.Event()
        self.lifecycle_lock = threading.Lock()
        self.started = False
        self.stopped = False
        self.threads = []  # type: List[threading.Thread]

        self.pose_listener = None  # type: Optional[socket.socket]
        self.frame_listener = None  # type: Optional[socket.socket]

        self.state_lock = threading.Lock()
        self.pose_client = None  # type: Optional[socket.socket]
        self.pose_connection_id = 0
        self.latest_pose = None  # type: Optional[PoseSample]
        self.rendered_pose = None  # type: Optional[tuple]

        self.frame_lock = threading.Lock()
        self.frame_client = None  # type: Optional[socket.socket]
        self.frame_connection_id = 0
        self.frame_ready = False
        self.frame_min_serial = 0
        self.next_frame_serial = 0
        self.pending_frame = None  # type: Optional[FrameSample]
        self.pending_event = threading.Event()

        metadata = self.core.camera_metadata()
        self.camera_info_header = {"version": 1, "type": "camera_info"}
        self.camera_info_header.update(metadata)
        self.width = int(metadata["width"])
        self.height = int(metadata["height"])
        self.expected_rgb_bytes = self.width * self.height * 3
        self.expected_depth_bytes = self.width * self.height * 2
        frame_bytes = self.expected_rgb_bytes + self.expected_depth_bytes
        if frame_bytes > MAX_PAYLOAD_BYTES:
            raise ValueError("渲染尺寸 %dx%d 的帧载荷 %d 字节超过协议上限 %d" % (
                self.width, self.height, frame_bytes, MAX_PAYLOAD_BYTES))

    # 生命周期
    def start(self) -> None:
        with self.lifecycle_lock:
            if self.started:
                return
            if self.stopped:
                raise RuntimeError("renderer 已停止，不能重新启动")
            pose_listener = self.create_listener(self.pose_port)
            try:
                frame_listener = self.create_listener(self.frame_port)
            except OSError:
                pose_listener.close()
                raise
            self.pose_listener = pose_listener
            self.frame_listener = frame_listener
            self.pose_port = pose_listener.getsockname()[1]
            self.frame_port = frame_listener.getsockname()[1]
            self.started = True
            self.threads = [
                threading.Thread(target=self.accept_pose_connections,
                                 name="pose-accept", daemon=True),
                threading.Thread(target=self.accept_frame_connections,
                                 name="frame-accept", daemon=True),
                threading.Thread(target=self.send_frames, name="frame-send", daemon=True),
                threading.Thread(target=self.render_loop, name="render", daemon=True),
            ]
            for thread in self.threads:
                thread.start()

    def run(self) -> None:
        self.start()
        try:
            self.stop_event.wait()
        finally:
            self.stop()

    def stop(self) -> None:
        with self.lifecycle_lock:
            if self.stopped:
                return
            self.stopped = True
        self.stop_event.set()

        self.close_socket(self.pose_listener)
        self.close_socket(self.frame_listener)

        with self.state_lock:
            pose_client = self.pose_client
            self.pose_client = None
            self.latest_pose = None
        self.close_socket(pose_client)

        with self.frame_lock:
            frame_client = self.frame_client
            self.frame_client = None
            self.frame_ready = False
            self.pending_frame = None
            self.pending_event.set()
        self.close_socket(frame_client)

        deadline = time.monotonic() + THREAD_JOIN_SECONDS
        current = threading.current_thread()
        with self.lifecycle_lock:
            threads = list(self.threads)
        for thread in threads:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if thread is not current:
                thread.join(timeout=remaining)

        with self.lifecycle_lock:
            self.threads = [thread for thread in self.threads if thread.is_alive()]

    def create_listener(self, port: int) -> socket.socket:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            listener.bind((self.host, port))
            listener.listen(1)
            listener.settimeout(ACCEPT_TIMEOUT_SECONDS)
        except OSError:
            listener.close()
            raise
        return listener

    @staticmethod
    def close_socket(sock: Optional[socket.socket]) -> None:
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def start_thread(self, target: Any, name: str, args: tuple = ()) -> threading.Thread:
        thread = threading.Thread(target=target, args=args, name=name, daemon=True)
        with self.lifecycle_lock:
            self.threads.append(thread)
        thread.start()
        return thread

    # 位姿通道
    def accept_pose_connections(self) -> None:
        while not self.stop_event.is_set():
            try:
                client, _ = self.pose_listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with self.state_lock:
                if self.pose_client is not None:
                    LOGGER.warning("拒绝第二个位姿连接：已有活动连接")
                    client.close()
                    continue
                self.pose_connection_id += 1
                connection_id = self.pose_connection_id
                self.pose_client = client
                self.latest_pose = None
            LOGGER.info("位姿连接 %d 已建立", connection_id)
            self.start_thread(self.read_pose_messages, "pose-reader", (client, connection_id))

    def read_pose_messages(self, client: socket.socket, connection_id: int) -> None:
        try:
            while not self.stop_event.is_set():
                header, _ = receive_message(client)
                pose = PoseSample(
                    connection_id=connection_id,
                    seq=header["seq"],
                    stamp_sec=header["stamp_sec"],
                    stamp_nsec=header["stamp_nsec"],
                    position=tuple(header["position"]),
                    orientation=tuple(header["orientation"]),
                )
                with self.state_lock:
                    if self.pose_client is not client:
                        break
                    self.latest_pose = pose
        except EOFError:
            LOGGER.info("位姿连接 %d 断开", connection_id)
        except ValueError as error:
            LOGGER.warning("位姿连接 %d 协议错误，关闭: %s", connection_id, error)
        except (OSError, TimeoutError) as error:
            LOGGER.warning("位姿连接 %d 关闭: %s", connection_id, error)
        finally:
            with self.state_lock:
                if self.pose_client is client:
                    self.pose_client = None
                    self.latest_pose = None
            self.close_socket(client)

    # 渲染
    def render_loop(self) -> None:
        rate = float(self.core.publish_rate)
        interval = 1.0 / rate if rate > 0 else 1.0
        next_tick = time.monotonic() + interval
        while not self.stop_event.is_set():
            wait = next_tick - time.monotonic()
            if wait > 0 and self.stop_event.wait(wait):
                break
            try:
                self.render_once()
            except Exception:
                traceback.print_exc()
            next_tick += interval
            now = time.monotonic()
            if next_tick < now - interval:
                next_tick = now + interval  # 落后过多时重新对齐，不补帧突发

    def render_once(self) -> None:
        with self.state_lock:
            pose = self.latest_pose
            if pose is None:
                return
            pose_key = (pose.connection_id, pose.seq)
            if self.rendered_pose == pose_key:
                return
            self.rendered_pose = pose_key

        rgb, depth = self.core.render(pose.position, pose.orientation)
        rgb_payload = bytes(rgb)
        depth_payload = bytes(depth)
        if len(rgb_payload) != self.expected_rgb_bytes:
            raise ValueError("RGB 载荷 %d 字节与 %dx%dx3 不符" % (
                len(rgb_payload), self.width, self.height))
        if len(depth_payload) != self.expected_depth_bytes:
            raise ValueError("depth 载荷 %d 字节与 %dx%dx2 不符" % (
                len(depth_payload), self.width, self.height))

        with self.frame_lock:
            self.next_frame_serial += 1
            serial = self.next_frame_serial
            header = {
                "version": 1,
                "type": "frame",
                "frame_seq": serial,
                "pose_seq": pose.seq,
                "stamp_sec": pose.stamp_sec,
                "stamp_nsec": pose.stamp_nsec,
                "width": self.width,
                "height": self.height,
                "rgb_encoding": RGB_ENCODING,
                "depth_encoding": DEPTH_ENCODING,
                "rgb_bytes": len(rgb_payload),
                "depth_bytes": len(depth_payload),
            }
            self.pending_frame = FrameSample(serial=serial, header=header,
                                             payload=rgb_payload + depth_payload)
            self.pending_event.set()

    # 图像通道
    def accept_frame_connections(self) -> None:
        while not self.stop_event.is_set():
            try:
                client, _ = self.frame_listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with self.frame_lock:
                if self.frame_client is not None:
                    LOGGER.warning("拒绝第二个图像连接：已有活动连接")
                    client.close()
                    continue
                self.frame_connection_id += 1
                connection_id = self.frame_connection_id
                self.frame_client = client
                self.frame_ready = False
                self.frame_min_serial = self.next_frame_serial + 1
                self.pending_frame = None
                self.pending_event.clear()
            LOGGER.info("图像连接 %d 已建立", connection_id)

            try:
                send_message(client, self.camera_info_header, timeout=MESSAGE_DEADLINE_SECONDS)
            except (OSError, TimeoutError) as error:
                LOGGER.warning("图像连接 %d 发送 camera_info 失败: %s", connection_id, error)
                with self.frame_lock:
                    if self.frame_connection_id == connection_id:
                        self.frame_client = None
                        self.frame_ready = False
                        self.pending_frame = None
                        self.pending_event.clear()
                self.close_socket(client)
                continue

            with self.frame_lock:
                if self.frame_connection_id == connection_id:
                    self.frame_ready = True
                    if self.pending_frame is not None:
                        self.pending_event.set()

    def send_frames(self) -> None:
        while not self.stop_event.is_set():
            if not self.pending_event.wait(0.1):
                continue
            frame = None
            with self.frame_lock:
                if (self.pending_frame is not None and self.frame_client is not None
                        and self.frame_ready
                        and self.pending_frame.serial >= self.frame_min_serial):
                    frame = self.pending_frame
                    self.pending_frame = None
                    self.pending_event.clear()
                    client = self.frame_client
                    connection_id = self.frame_connection_id
                else:
                    self.pending_event.clear()
            if frame is None:
                continue
            try:
                send_message(client, frame.header, frame.payload,
                             timeout=MESSAGE_DEADLINE_SECONDS)
            except (OSError, TimeoutError) as error:
                LOGGER.warning("图像连接 %d 发送帧 %d 失败: %s",
                               connection_id, frame.serial, error)
                with self.frame_lock:
                    if self.frame_connection_id == connection_id:
                        self.frame_client = None
                        self.frame_ready = False
                        self.pending_frame = None
                        self.pending_event.clear()
                self.close_socket(client)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="TCP 3DGS Renderer")
    parser.add_argument("--ply_file", type=Path, default=Path("3dgs.ply"))
    parser.add_argument("--params_file", type=Path,
                        default=Path(__file__).resolve().parent.parent / "config" / "renderer_params.yaml")
    parser.add_argument("--enable_gimbal", action="store_true", default=False)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--pose-port", type=int, default=19080)
    parser.add_argument("--frame-port", type=int, default=19081)
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    from gsplat_core import GsplatCore

    core = GsplatCore(args.ply_file, args.params_file, args.enable_gimbal)
    try:
        renderer = TcpRenderer(core, args.host, args.pose_port, args.frame_port)
    except ValueError as error:
        print("TCP renderer 配置无效: %s" % error, file=sys.stderr)
        return 1

    def handle_signal(signum: int, frame: Any) -> None:
        renderer.stop()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    try:
        renderer.run()
    except OSError as error:
        print("TCP renderer 启动失败: %s" % error, file=sys.stderr)
        return 1
    finally:
        renderer.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
