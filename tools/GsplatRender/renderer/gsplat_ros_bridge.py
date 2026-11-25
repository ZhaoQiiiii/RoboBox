"""ROS 1 bridge：把 TCP renderer 的位姿与图像转成原有 ROS 话题。

bridge 主动连接 renderer：位姿连接只发 pose，图像连接先收 camera_info 再收 frame。
ROS odom 回调只更新容量为 1 的位姿槽；图像接收与 ROS 转换发布分线程进行，
待发布帧同样只有容量 1 的完整帧槽。
"""

import argparse
import signal
import socket
import struct
import sys
import threading
import time
import traceback
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rospy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from tcp_protocol import FRAME_ID, MESSAGE_DEADLINE_SECONDS, receive_message, send_message

CAMERA_NAMES = ["camera0", "camera1", "camera2", "camera3"]
RECONNECT_INTERVAL_SECONDS = 1.0
CONNECT_TIMEOUT_SECONDS = 3.0
THREAD_JOIN_SECONDS = 3.0
STATUS_INTERVAL_SECONDS = 10.0

CV_BRIDGE = CvBridge()


def pose_to_header(message: Odometry, seq: int) -> Dict[str, Any]:
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return {
        "version": 1,
        "type": "pose",
        "seq": int(seq),
        "stamp_sec": int(message.header.stamp.secs),
        "stamp_nsec": int(message.header.stamp.nsecs),
        "position": [float(position.x), float(position.y), float(position.z)],
        "orientation": [float(orientation.x), float(orientation.y),
                        float(orientation.z), float(orientation.w)],
    }


def metadata_to_camera_info(metadata: Dict[str, Any]) -> CameraInfo:
    info = CameraInfo()
    info.header.frame_id = metadata.get("frame_id", FRAME_ID)
    info.width = int(metadata["width"])
    info.height = int(metadata["height"])
    info.distortion_model = "plumb_bob"
    info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    fx = float(metadata["fx"])
    fy = float(metadata["fy"])
    cx = float(metadata["cx"])
    cy = float(metadata["cy"])
    info.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


def frame_to_messages(header: Dict[str, Any],
                      payload: bytes) -> Dict[str, Any]:
    """把一条 frame 消息转换为 4 个原有图像消息（raw/JPEG RGB、raw/PNG depth）。"""
    width = int(header["width"])
    height = int(header["height"])
    rgb_bytes = int(header["rgb_bytes"])
    depth_bytes = int(header["depth_bytes"])
    if len(payload) != rgb_bytes + depth_bytes:
        raise ValueError("载荷长度 %d 与 rgb_bytes+depth_bytes 不符" % len(payload))

    rgb_image = np.frombuffer(payload[:rgb_bytes], dtype=np.uint8).reshape(height, width, 3)
    depth_image = np.frombuffer(payload[rgb_bytes:], dtype="<u2").reshape(height, width).copy()
    stamp = rospy.Time(int(header["stamp_sec"]), int(header["stamp_nsec"]))
    frame_id = header.get("frame_id", FRAME_ID)

    rgb_raw = Image()
    rgb_raw.header.stamp = stamp
    rgb_raw.header.frame_id = frame_id
    rgb_raw.height = height
    rgb_raw.width = width
    rgb_raw.encoding = "rgb8"
    rgb_raw.is_bigendian = False
    rgb_raw.step = width * 3
    rgb_raw.data = rgb_image.tobytes()

    depth_raw = Image()
    depth_raw.header.stamp = stamp
    depth_raw.header.frame_id = frame_id
    depth_raw.height = height
    depth_raw.width = width
    depth_raw.encoding = "16UC1"
    depth_raw.is_bigendian = False
    depth_raw.step = width * 2
    depth_raw.data = depth_image.tobytes()

    # cv_bridge 的 JPEG 压缩需要 BGR 输入（与原实现一致）。
    bgr_image = np.ascontiguousarray(rgb_image[..., ::-1])
    rgb_compressed = CV_BRIDGE.cv2_to_compressed_imgmsg(bgr_image, dst_format="jpg")
    rgb_compressed.header.stamp = stamp
    rgb_compressed.header.frame_id = frame_id

    depth_compressed = CV_BRIDGE.cv2_to_compressed_imgmsg(depth_image, dst_format="png")
    depth_compressed.data = struct.pack("<iff", 0, 0.0, 0.0) + depth_compressed.data
    depth_compressed.format = "16UC1; compressedDepth png"
    depth_compressed.header.stamp = stamp
    depth_compressed.header.frame_id = frame_id

    return {
        "rgb_raw": rgb_raw,
        "rgb_compressed": rgb_compressed,
        "depth_raw": depth_raw,
        "depth_compressed": depth_compressed,
    }


class RosBridge:
    def __init__(self, host: str = "127.0.0.1", pose_port: int = 19080,
                 frame_port: int = 19081,
                 odom_topic: str = "/super/sim/lidar_slam/odom") -> None:
        self.host = host
        self.pose_port = int(pose_port)
        self.frame_port = int(frame_port)
        self.odom_topic = odom_topic

        self.stop_event = threading.Event()
        self.lifecycle_lock = threading.Lock()
        self.started = False
        self.stopped = False
        self.threads = []  # type: List[threading.Thread]

        self.pose_lock = threading.Lock()
        self.pose_slot = None  # type: Optional[Odometry]
        self.pose_event = threading.Event()
        self.pose_client = None  # type: Optional[socket.socket]
        self.pose_seq = 0

        self.frame_lock = threading.Lock()
        self.frame_slot = None  # type: Optional[Tuple[Dict[str, Any], bytes]]
        self.frame_event = threading.Event()
        self.frame_client = None  # type: Optional[socket.socket]

        self.metadata_lock = threading.Lock()
        self.camera_info_count = 0
        self.odom_messages = 0
        self.frames_published = 0

        self.image_publishers = {
            "rgb_raw": rospy.Publisher(
                "/gsplat_render/camera0/color/image", Image, queue_size=10),
            "rgb_compressed": rospy.Publisher(
                "/gsplat_render/camera0/color/image/compressed", CompressedImage, queue_size=10),
            "depth_raw": rospy.Publisher(
                "/gsplat_render/camera0/depth/image", Image, queue_size=10),
            "depth_compressed": rospy.Publisher(
                "/gsplat_render/camera0/depth/image/compressed", CompressedImage, queue_size=10),
        }
        self.color_info_publishers = {}  # type: Dict[str, rospy.Publisher]
        self.depth_info_publishers = {}  # type: Dict[str, rospy.Publisher]
        for camera_name in CAMERA_NAMES:
            self.color_info_publishers[camera_name] = rospy.Publisher(
                "/gsplat_render/%s/color/info" % camera_name, CameraInfo,
                queue_size=1, latch=True)
            self.depth_info_publishers[camera_name] = rospy.Publisher(
                "/gsplat_render/%s/depth/info" % camera_name, CameraInfo,
                queue_size=1, latch=True)
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

    # 生命周期
    def start(self) -> None:
        with self.lifecycle_lock:
            if self.started:
                return
            if self.stopped:
                raise RuntimeError("bridge 已停止，不能重新启动")
            self.started = True
            self.threads = [
                threading.Thread(target=self.connect_pose_connection,
                                 name="pose-send", daemon=True),
                threading.Thread(target=self.connect_frame_connection,
                                 name="frame-receive", daemon=True),
                threading.Thread(target=self.publish_loop, name="frame-publish", daemon=True),
                threading.Thread(target=self.report_status, name="status", daemon=True),
            ]
            for thread in self.threads:
                thread.start()

    def run(self) -> None:
        self.start()
        try:
            while not self.stop_event.wait(0.5):
                if rospy.is_shutdown():
                    break
        finally:
            self.stop()

    def stop(self) -> None:
        with self.lifecycle_lock:
            if self.stopped:
                return
            self.stopped = True
        self.stop_event.set()

        self.close_socket(self.pose_client)
        self.pose_client = None
        self.close_socket(self.frame_client)
        self.frame_client = None
        with self.frame_lock:
            self.frame_slot = None
        self.frame_event.set()

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

        self.odom_subscriber.unregister()
        for publisher in self.image_publishers.values():
            publisher.unregister()
        for publisher in self.color_info_publishers.values():
            publisher.unregister()
        for publisher in self.depth_info_publishers.values():
            publisher.unregister()

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

    # 位姿通道
    def odom_callback(self, message: Odometry) -> None:
        with self.pose_lock:
            self.pose_slot = message
            self.odom_messages += 1
        self.pose_event.set()

    def connect_pose_connection(self) -> None:
        while not self.stop_event.is_set():
            try:
                client = socket.create_connection(
                    (self.host, self.pose_port), timeout=CONNECT_TIMEOUT_SECONDS)
            except OSError as error:
                rospy.logwarn_throttle(
                    5.0, "bridge 无法连接位姿端口 %s:%d: %s" % (
                        self.host, self.pose_port, error))
                self.stop_event.wait(RECONNECT_INTERVAL_SECONDS)
                continue
            # connect timeout 只用于建连；空闲时消息等待不能继承它。
            client.settimeout(None)
            self.pose_client = client
            self.pose_seq = 0
            try:
                while not self.stop_event.is_set():
                    if not self.pose_event.wait(0.1):
                        continue
                    with self.pose_lock:
                        message = self.pose_slot
                        self.pose_slot = None
                        self.pose_event.clear()
                    if message is None:
                        continue
                    try:
                        send_message(client, pose_to_header(message, self.pose_seq),
                                     timeout=MESSAGE_DEADLINE_SECONDS)
                    except ValueError as error:
                        rospy.logwarn_throttle(5.0, "bridge 丢弃非法位姿: %s" % error)
                        continue
                    except (OSError, TimeoutError):
                        # 发送失败时把最新位姿放回槽里，重连后继续发送。
                        with self.pose_lock:
                            if self.pose_slot is None:
                                self.pose_slot = message
                        self.pose_event.set()
                        break
                    self.pose_seq += 1
            except (OSError, TimeoutError) as error:
                rospy.logwarn_throttle(5.0, "bridge 位姿连接断开: %s" % error)
            finally:
                self.close_socket(client)
                if self.pose_client is client:
                    self.pose_client = None
            self.stop_event.wait(RECONNECT_INTERVAL_SECONDS)

    # 图像通道
    def connect_frame_connection(self) -> None:
        while not self.stop_event.is_set():
            try:
                client = socket.create_connection(
                    (self.host, self.frame_port), timeout=CONNECT_TIMEOUT_SECONDS)
            except OSError as error:
                rospy.logwarn_throttle(
                    5.0, "bridge 无法连接图像端口 %s:%d: %s" % (
                        self.host, self.frame_port, error))
                self.stop_event.wait(RECONNECT_INTERVAL_SECONDS)
                continue
            # 图像连接会长时间空闲；不能沿用 connect timeout。
            client.settimeout(None)
            self.frame_client = client
            self.clear_frame_slot()
            try:
                while not self.stop_event.is_set():
                    header, payload = receive_message(client)
                    message_type = header.get("type")
                    if message_type == "camera_info":
                        self.publish_camera_info(header)
                    elif message_type == "frame":
                        self.accept_frame(header, payload)
                    else:
                        raise ValueError("bridge 收到未知消息类型: %r" % (message_type,))
            except EOFError:
                rospy.loginfo_throttle(5.0, "bridge 图像连接由 renderer 关闭，重新连接")
            except (OSError, TimeoutError, ValueError) as error:
                rospy.logwarn_throttle(5.0, "bridge 图像连接断开: %s" % error)
            finally:
                self.close_socket(client)
                if self.frame_client is client:
                    self.frame_client = None
                self.clear_frame_slot()
            self.stop_event.wait(RECONNECT_INTERVAL_SECONDS)

    def publish_camera_info(self, metadata: Dict[str, Any]) -> None:
        info = metadata_to_camera_info(metadata)
        for publishers in (self.color_info_publishers, self.depth_info_publishers):
            for publisher in publishers.values():
                publisher.publish(info)
        with self.metadata_lock:
            self.camera_info_count += 1

    def accept_frame(self, header: Dict[str, Any], payload: bytes) -> None:
        with self.frame_lock:
            self.frame_slot = (header, payload)
        self.frame_event.set()

    def clear_frame_slot(self) -> None:
        with self.frame_lock:
            self.frame_slot = None
        self.frame_event.clear()

    def publish_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self.frame_event.wait(0.1):
                continue
            with self.frame_lock:
                message = self.frame_slot
                self.frame_slot = None
                self.frame_event.clear()
            if message is None:
                continue
            try:
                self.publish_frame(*message)
            except Exception:
                traceback.print_exc()

    def publish_frame(self, header: Dict[str, Any], payload: bytes) -> None:
        messages = frame_to_messages(header, payload)
        for name, publisher in self.image_publishers.items():
            publisher.publish(messages[name])
        self.frames_published += 1

    # 周期状态
    def report_status(self) -> None:
        last_frames = 0
        last_odom = 0
        while not self.stop_event.wait(STATUS_INTERVAL_SECONDS):
            frames = self.frames_published
            odom = self.odom_messages
            rospy.loginfo(
                "bridge 状态：发布 %.1f 帧/秒，odom %.1f 条/秒，位姿连接 %s，图像连接 %s",
                (frames - last_frames) / STATUS_INTERVAL_SECONDS,
                (odom - last_odom) / STATUS_INTERVAL_SECONDS,
                "已建立" if self.pose_client is not None else "无",
                "已建立" if self.frame_client is not None else "无")
            last_frames, last_odom = frames, odom


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="GsplatRender ROS 1 bridge")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--pose-port", type=int, default=19080)
    parser.add_argument("--frame-port", type=int, default=19081)
    parser.add_argument("--odom-topic", default="/super/sim/lidar_slam/odom")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    rospy.init_node("gsplat_ros_bridge", disable_signals=True)
    bridge = RosBridge(args.host, args.pose_port, args.frame_port, args.odom_topic)

    def handle_signal(signum: int, frame: Any) -> None:
        bridge.stop()
        rospy.signal_shutdown("收到信号 %d" % signum)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    try:
        bridge.run()
    finally:
        bridge.stop()
        rospy.signal_shutdown("bridge 退出")
    return 0


if __name__ == "__main__":
    sys.exit(main())
