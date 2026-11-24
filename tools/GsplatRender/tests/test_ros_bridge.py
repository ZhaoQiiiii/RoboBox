import os
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

RENDERER_DIR = Path(__file__).resolve().parent.parent / "renderer"
sys.path.insert(0, str(RENDERER_DIR))

import cv2
import numpy as np
import rospy
import rosgraph
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from unittest import mock

import gsplat_renderer_tcp
import gsplat_ros_bridge
import tcp_protocol

ODOM_TOPIC = "/super/sim/lidar_slam/odom"


class FakeCore:
    def __init__(self, width=4, height=3, publish_rate=0.01, rgb_fill=17, depth_fill=1250):
        self.width = width
        self.height = height
        self.publish_rate = publish_rate
        self.rgb_fill = rgb_fill
        self.depth_fill = depth_fill

    def camera_metadata(self):
        return {
            "width": self.width,
            "height": self.height,
            "fx": 100.0,
            "fy": 100.0,
            "cx": 2.0,
            "cy": 1.5,
            "frame_id": "camera",
            "cam_names": ["camera0", "camera1", "camera2", "camera3"],
            "rgb_encoding": "rgb8",
            "depth_encoding": "16UC1",
        }

    def render(self, position, orientation):
        rgb = bytes([self.rgb_fill]) * (self.width * self.height * 3)
        depth = struct.pack("<H", self.depth_fill) * (self.width * self.height)
        return rgb, depth


class TopicCollector:
    def __init__(self, topic, message_type, test_case):
        self.topic = topic
        self.messages = []
        self.lock = threading.Lock()
        self.subscriber = rospy.Subscriber(topic, message_type, self.callback)
        test_case.addCleanup(self.subscriber.unregister)

    def callback(self, message):
        with self.lock:
            self.messages.append(message)

    def snapshot(self):
        with self.lock:
            return list(self.messages)


def wait_until(predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


def wait_topic_connection(publisher, timeout=10.0):
    return wait_until(lambda: publisher.get_num_connections() >= 1, timeout)


def frame_header(width, height, frame_seq=1, pose_seq=0, stamp_sec=1, stamp_nsec=0):
    return {
        "version": 1,
        "type": "frame",
        "frame_seq": frame_seq,
        "pose_seq": pose_seq,
        "stamp_sec": stamp_sec,
        "stamp_nsec": stamp_nsec,
        "width": width,
        "height": height,
        "rgb_encoding": "rgb8",
        "depth_encoding": "16UC1",
        "rgb_bytes": width * height * 3,
        "depth_bytes": width * height * 2,
    }


def frame_payload(rgb, depth):
    return rgb.tobytes() + np.ascontiguousarray(depth, dtype="<u2").tobytes()


class RosBridgeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        probe = socket.socket()
        probe.bind(("127.0.0.1", 0))
        cls.master_port = probe.getsockname()[1]
        probe.close()

        cls.ros_home = tempfile.TemporaryDirectory()
        environment = dict(os.environ)
        environment["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % cls.master_port
        environment["ROS_HOME"] = cls.ros_home.name
        cls.roscore = subprocess.Popen(
            ["roscore", "-p", str(cls.master_port)], env=environment,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        os.environ["ROS_MASTER_URI"] = environment["ROS_MASTER_URI"]
        os.environ["ROS_HOME"] = environment["ROS_HOME"]

        master = rosgraph.Master("/gsplat_bridge_test_probe")
        deadline = time.monotonic() + 20.0
        while True:
            try:
                master.getPid()
                break
            except Exception:
                if time.monotonic() > deadline:
                    cls.roscore.kill()
                    raise RuntimeError("隔离 roscore 未能启动")
                time.sleep(0.1)

        rospy.init_node("gsplat_bridge_test", anonymous=True, disable_signals=True)

    @classmethod
    def tearDownClass(cls):
        rospy.signal_shutdown("tests finished")
        cls.roscore.terminate()
        try:
            cls.roscore.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            cls.roscore.kill()
            cls.roscore.wait()
        cls.ros_home.cleanup()

    def reserve_ports(self, count):
        sockets = []
        ports = []
        for _ in range(count):
            sock = socket.socket()
            sock.bind(("127.0.0.1", 0))
            ports.append(sock.getsockname()[1])
            sockets.append(sock)
        for sock in sockets:
            sock.close()
        return ports

    def start_renderer(self, ports):
        core = FakeCore()
        renderer = gsplat_renderer_tcp.TcpRenderer(core, "127.0.0.1", ports[0], ports[1])
        renderer.start()
        self.addCleanup(renderer.stop)
        return renderer, core

    def start_bridge(self, ports):
        bridge = gsplat_ros_bridge.RosBridge("127.0.0.1", ports[0], ports[1], ODOM_TOPIC)
        bridge.start()
        self.addCleanup(bridge.stop)
        return bridge

    def publish_odom(self, position_x, stamp_sec=100, stamp_nsec=500):
        publisher = rospy.Publisher(ODOM_TOPIC, Odometry, queue_size=1)
        self.addCleanup(publisher.unregister)
        self.assertTrue(wait_topic_connection(publisher), "bridge 未订阅 odom topic")
        message = Odometry()
        message.header.stamp = rospy.Time(stamp_sec, stamp_nsec)
        message.pose.pose.position.x = position_x
        message.pose.pose.orientation.w = 1.0
        publisher.publish(message)
        return publisher

    def publish_odom_until_pose(self, renderer, position_x, stamp_sec, timeout=10.0):
        publisher = rospy.Publisher(ODOM_TOPIC, Odometry, queue_size=1)
        self.addCleanup(publisher.unregister)
        self.assertTrue(wait_topic_connection(publisher), "bridge 未订阅 odom topic")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = Odometry()
            message.header.stamp = rospy.Time(stamp_sec, 0)
            message.pose.pose.position.x = position_x
            message.pose.pose.orientation.w = 1.0
            publisher.publish(message)
            if self.pose_stamp(renderer) == stamp_sec:
                return
            time.sleep(0.1)
        self.fail("renderer 重启后未收到位姿")

    def image_collectors(self):
        return {
            "color": TopicCollector("/gsplat_render/camera0/color/image", Image, self),
            "depth": TopicCollector("/gsplat_render/camera0/depth/image", Image, self),
            "color_compressed": TopicCollector(
                "/gsplat_render/camera0/color/image/compressed", CompressedImage, self),
            "depth_compressed": TopicCollector(
                "/gsplat_render/camera0/depth/image/compressed", CompressedImage, self),
        }

    def render_if_pose(self, renderer):
        self.assertTrue(wait_until(lambda: self.latest_pose(renderer) is not None),
                        "renderer 未收到 bridge 转发的位姿")
        renderer.render_once()

    def latest_pose(self, renderer):
        with renderer.state_lock:
            return renderer.latest_pose

    def pose_stamp(self, renderer):
        pose = self.latest_pose(renderer)
        return None if pose is None else pose.stamp_sec

    def wait_camera_info(self, bridge, minimum=1):
        return wait_until(lambda: bridge.camera_info_count >= minimum)

    def flush_stale_frame_client(self, renderer, bridge, first_seq=500):
        seq = first_seq
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if bridge.camera_info_count >= 1:
                return True
            self.publish_odom(0.0, stamp_sec=seq)
            self.assertTrue(wait_until(lambda seq=seq: self.pose_stamp(renderer) == seq))
            renderer.render_once()
            seq += 1
            time.sleep(0.05)
        return bridge.camera_info_count >= 1

    def test_pose_to_header(self):
        message = Odometry()
        message.header.stamp = rospy.Time(123, 456)
        message.pose.pose.position.x = 1.5
        message.pose.pose.position.y = -2.0
        message.pose.pose.position.z = 0.25
        message.pose.pose.orientation.x = 0.1
        message.pose.pose.orientation.y = 0.2
        message.pose.pose.orientation.z = 0.3
        message.pose.pose.orientation.w = 0.9

        header = gsplat_ros_bridge.pose_to_header(message, 7)
        self.assertEqual(header["version"], 1)
        self.assertEqual(header["type"], "pose")
        self.assertEqual(header["seq"], 7)
        self.assertEqual((header["stamp_sec"], header["stamp_nsec"]), (123, 456))
        self.assertEqual(header["position"], [1.5, -2.0, 0.25])
        self.assertEqual(header["orientation"], [0.1, 0.2, 0.3, 0.9])

    def test_raw_rgb_exact(self):
        rgb = np.arange(2 * 2 * 3, dtype=np.uint8).reshape(2, 2, 3)
        depth = np.arange(4, dtype=np.uint16).reshape(2, 2)
        messages = gsplat_ros_bridge.frame_to_messages(
            frame_header(2, 2, stamp_sec=12, stamp_nsec=345), frame_payload(rgb, depth))

        raw = messages["rgb_raw"]
        self.assertEqual(raw.data, rgb.tobytes())
        self.assertEqual((raw.width, raw.height, raw.step), (2, 2, 6))
        self.assertEqual(raw.encoding, "rgb8")
        self.assertFalse(raw.is_bigendian)
        self.assertEqual((raw.header.stamp.secs, raw.header.stamp.nsecs), (12, 345))
        self.assertEqual(raw.header.frame_id, "camera")

    def test_raw_depth_exact(self):
        rgb = np.zeros((2, 2, 3), dtype=np.uint8)
        depth = np.array([[100, 200], [300, 65535]], dtype=np.uint16)
        messages = gsplat_ros_bridge.frame_to_messages(
            frame_header(2, 2, stamp_sec=12, stamp_nsec=345), frame_payload(rgb, depth))

        raw = messages["depth_raw"]
        self.assertEqual(raw.data, depth.astype("<u2").tobytes())
        self.assertEqual((raw.width, raw.height, raw.step), (2, 2, 4))
        self.assertEqual(raw.encoding, "16UC1")
        self.assertFalse(raw.is_bigendian)
        self.assertEqual((raw.header.stamp.secs, raw.header.stamp.nsecs), (12, 345))
        self.assertEqual(raw.header.frame_id, "camera")

    def test_jpeg_channel_order(self):
        rgb = np.zeros((8, 8, 3), dtype=np.uint8)
        rgb[..., 0] = 255
        depth = np.zeros((8, 8), dtype=np.uint16)
        messages = gsplat_ros_bridge.frame_to_messages(frame_header(8, 8), frame_payload(rgb, depth))

        compressed = messages["rgb_compressed"]
        decoded = cv2.imdecode(np.frombuffer(compressed.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        self.assertIsNotNone(decoded)
        self.assertGreater(float(decoded[..., 2].mean()), 200.0)
        self.assertLess(float(decoded[..., 0].mean()), 50.0)
        self.assertLess(float(decoded[..., 1].mean()), 50.0)

    def test_depth_png_roundtrip(self):
        rgb = np.zeros((2, 2, 3), dtype=np.uint8)
        depth = np.array([[0, 100], [2000, 65535]], dtype=np.uint16)
        messages = gsplat_ros_bridge.frame_to_messages(frame_header(2, 2), frame_payload(rgb, depth))

        compressed = messages["depth_compressed"]
        self.assertEqual(compressed.data[:12], struct.pack("<iff", 0, 0.0, 0.0))
        self.assertEqual(compressed.format, "16UC1; compressedDepth png")
        decoded = cv2.imdecode(
            np.frombuffer(compressed.data[12:], dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        np.testing.assert_array_equal(decoded, depth)
        self.assertEqual((compressed.header.stamp.secs, compressed.header.stamp.nsecs), (1, 0))
        self.assertEqual(compressed.header.frame_id, "camera")

    def test_camera_info_matches_legacy(self):
        metadata = FakeCore().camera_metadata()
        info = gsplat_ros_bridge.metadata_to_camera_info(metadata)
        self.assertEqual(info.header.frame_id, "camera")
        self.assertEqual((info.width, info.height), (4, 3))
        self.assertEqual(info.distortion_model, "plumb_bob")
        self.assertEqual(list(info.D), [0.0] * 5)
        self.assertEqual(list(info.K), [100.0, 0.0, 2.0, 0.0, 100.0, 1.5, 0.0, 0.0, 1.0])
        self.assertEqual(list(info.R), [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.assertEqual(list(info.P), [100.0, 0.0, 2.0, 0.0, 0.0, 100.0, 1.5, 0.0, 0.0, 0.0, 1.0, 0.0])

        bridge = self.start_bridge(self.reserve_ports(2))
        bridge.publish_camera_info(metadata)
        for camera_name in ("camera0", "camera1", "camera2", "camera3"):
            for kind in ("color", "depth"):
                topic = "/gsplat_render/%s/%s/info" % (camera_name, kind)
                received = rospy.wait_for_message(topic, CameraInfo, timeout=3.0)
                self.assertEqual(list(received.K), list(info.K))
                self.assertEqual(received.header.frame_id, "camera")

    def test_stamp_reset_accepted(self):
        rgb = np.zeros((2, 2, 3), dtype=np.uint8)
        depth = np.zeros((2, 2), dtype=np.uint16)
        payload = frame_payload(rgb, depth)
        newer = gsplat_ros_bridge.frame_to_messages(
            frame_header(2, 2, stamp_sec=200), payload)
        older = gsplat_ros_bridge.frame_to_messages(
            frame_header(2, 2, stamp_sec=100), payload)
        self.assertEqual(newer["rgb_raw"].header.stamp.secs, 200)
        self.assertEqual(older["rgb_raw"].header.stamp.secs, 100)
        self.assertEqual(older["depth_raw"].header.stamp.secs, 100)

    def test_bridge_starts_first(self):
        ports = self.reserve_ports(2)
        bridge = self.start_bridge(ports)
        collectors = self.image_collectors()
        self.publish_odom(1.25, stamp_sec=11, stamp_nsec=22)

        renderer, core = self.start_renderer(ports)
        self.assertTrue(self.wait_camera_info(bridge), "bridge 未收到 camera_info")
        self.render_if_pose(renderer)

        self.assertTrue(wait_until(lambda: collectors["color"].snapshot()),
                        "bridge 未发布 RGB")
        rgb_message = collectors["color"].snapshot()[0]
        self.assertEqual(rgb_message.data, bytes([17]) * 36)
        self.assertEqual((rgb_message.header.stamp.secs, rgb_message.header.stamp.nsecs), (11, 22))

        depth_message = collectors["depth"].snapshot()[0]
        self.assertEqual(depth_message.data, struct.pack("<H", 1250) * 12)
        self.assertEqual(depth_message.encoding, "16UC1")

    def test_renderer_restart(self):
        ports = self.reserve_ports(2)
        renderer, core = self.start_renderer(ports)
        bridge = self.start_bridge(ports)
        collectors = self.image_collectors()
        self.publish_odom(1.0, stamp_sec=100, stamp_nsec=500)
        self.render_if_pose(renderer)
        self.assertTrue(wait_until(lambda: collectors["color"].snapshot()))
        self.assertEqual(collectors["color"].snapshot()[0].header.stamp.secs, 100)

        renderer.stop()
        info_count = bridge.camera_info_count
        restarted, restarted_core = self.start_renderer(ports)
        self.publish_odom_until_pose(restarted, 2.0, 300)
        self.assertTrue(self.wait_camera_info(bridge, info_count + 1),
                        "renderer 重启后 bridge 未收到新的 camera_info")
        restarted.render_once()

        self.assertTrue(wait_until(lambda: len(collectors["color"].snapshot()) >= 2),
                        "renderer 重启后 bridge 未恢复发布")
        latest = collectors["color"].snapshot()[-1]
        self.assertEqual((latest.header.stamp.secs, latest.header.stamp.nsecs), (300, 0))

        info = rospy.wait_for_message("/gsplat_render/camera0/color/info", CameraInfo, timeout=3.0)
        self.assertEqual((info.width, info.height), (4, 3))

    def test_bridge_restart(self):
        ports = self.reserve_ports(2)
        renderer, core = self.start_renderer(ports)
        first_bridge = self.start_bridge(ports)
        collectors = self.image_collectors()
        self.publish_odom(1.0, stamp_sec=100, stamp_nsec=500)
        self.render_if_pose(renderer)
        self.assertTrue(wait_until(lambda: collectors["color"].snapshot()))

        first_bridge.stop()
        second_bridge = self.start_bridge(ports)

        # 反复渲染以清掉 renderer 侧的旧图像连接，直到新 bridge 收到 camera_info。
        self.assertTrue(self.flush_stale_frame_client(renderer, second_bridge),
                        "renderer 未接受新 bridge 的图像连接")

        self.publish_odom(3.0, stamp_sec=400, stamp_nsec=0)
        self.assertTrue(wait_until(lambda: self.pose_stamp(renderer) == 400))
        renderer.render_once()
        self.assertTrue(wait_until(
            lambda: any(message.header.stamp.secs == 400
                        for message in collectors["color"].snapshot())),
            "bridge 重启后未恢复发布")

    def test_idle_frame_connection_does_not_reconnect(self):
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", 0))
        listener.listen(2)
        frame_port = listener.getsockname()[1]
        pose_port = self.reserve_ports(1)[0]
        accepted = []
        stop = threading.Event()
        info_header = {"version": 1, "type": "camera_info"}
        info_header.update(FakeCore().camera_metadata())

        def fake_renderer():
            listener.settimeout(0.2)
            while not stop.is_set():
                try:
                    client, _ = listener.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                accepted.append(client)
                try:
                    tcp_protocol.send_message(client, info_header)
                except OSError:
                    pass

        server_thread = threading.Thread(target=fake_renderer, daemon=True)
        server_thread.start()
        self.addCleanup(stop.set)
        self.addCleanup(listener.close)

        with mock.patch.object(gsplat_ros_bridge, "CONNECT_TIMEOUT_SECONDS", 0.3):
            bridge = gsplat_ros_bridge.RosBridge("127.0.0.1", pose_port, frame_port, ODOM_TOPIC)
            bridge.start()
            self.addCleanup(bridge.stop)
            try:
                self.assertTrue(wait_until(lambda: bridge.camera_info_count >= 1))
                time.sleep(2.0)
                self.assertEqual(len(accepted), 1, "空闲图像连接不应超时重连")
                self.assertIsNotNone(bridge.frame_client)
            finally:
                bridge.stop()

        for client in accepted:
            client.close()
        server_thread.join(timeout=2.0)

    def test_invalid_odom_does_not_kill_pose_thread(self):
        ports = self.reserve_ports(2)
        renderer, core = self.start_renderer(ports)
        bridge = self.start_bridge(ports)
        publisher = rospy.Publisher(ODOM_TOPIC, Odometry, queue_size=1)
        self.addCleanup(publisher.unregister)
        self.assertTrue(wait_topic_connection(publisher))

        bad_message = Odometry()
        bad_message.header.stamp = rospy.Time(1, 0)
        bad_message.pose.pose.position.x = float("nan")
        bad_message.pose.pose.orientation.w = 1.0
        with mock.patch.object(gsplat_ros_bridge.rospy, "logwarn_throttle") as warning:
            publisher.publish(bad_message)
            self.assertTrue(
                wait_until(lambda: any("非法" in str(call)
                                       for call in warning.call_args_list)),
                "非法位姿应记录告警")

        self.publish_odom(5.0, stamp_sec=777)
        self.assertTrue(wait_until(lambda: self.pose_stamp(renderer) == 777),
                        "非法位姿后位姿线程应继续转发")

    def test_slow_ros_publisher(self):
        bridge = self.start_bridge(self.reserve_ports(2))
        collectors = self.image_collectors()

        original_publish = bridge.publish_frame

        def slow_publish(header, payload):
            time.sleep(0.2)
            original_publish(header, payload)

        bridge.publish_frame = slow_publish
        rgb = np.full((3, 4, 3), 17, dtype=np.uint8)
        depth = np.full((3, 4), 1250, dtype=np.uint16)
        payload = frame_payload(rgb, depth)
        for frame_seq in range(1, 6):
            bridge.accept_frame(frame_header(4, 3, frame_seq=frame_seq), payload)
            with bridge.frame_lock:
                depth_in_slot = 0 if bridge.frame_slot is None else 1
            self.assertLessEqual(depth_in_slot, 1, "待发布帧必须不超过 1")

        self.assertTrue(wait_until(lambda: collectors["color"].snapshot()),
                        "慢发布期间仍有帧应被发布")
        self.assertLessEqual(len(collectors["color"].snapshot()), 5)


if __name__ == "__main__":
    unittest.main()
