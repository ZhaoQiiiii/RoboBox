import socket
import struct
import subprocess
import sys
import threading
import time
import unittest
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
RENDERER_DIR = PACKAGE_ROOT / "renderer"
sys.path.insert(0, str(RENDERER_DIR))

import gsplat_renderer_tcp
import tcp_protocol

CAMERA_METADATA = {
    "width": 4,
    "height": 3,
    "fx": 100.0,
    "fy": 100.0,
    "cx": 2.0,
    "cy": 1.5,
    "frame_id": "camera",
    "cam_names": ["camera0", "camera1", "camera2", "camera3"],
    "rgb_encoding": "rgb8",
    "depth_encoding": "16UC1",
}


class FakeCore:
    def __init__(self, width=4, height=3, publish_rate=0.01, rgb_fill=17, depth_fill=1250):
        self.width = width
        self.height = height
        self.publish_rate = publish_rate
        self.rgb_fill = rgb_fill
        self.depth_fill = depth_fill
        self.render_calls = []
        self.render_lock = threading.Lock()

    def camera_metadata(self):
        metadata = dict(CAMERA_METADATA)
        metadata["width"] = self.width
        metadata["height"] = self.height
        return metadata

    def render(self, position, orientation):
        with self.render_lock:
            self.render_calls.append(([float(value) for value in position],
                                      [float(value) for value in orientation]))
        rgb = bytes([self.rgb_fill]) * (self.width * self.height * 3)
        depth = struct.pack("<H", self.depth_fill) * (self.width * self.height)
        return rgb, depth


def wait_until(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


class TcpRuntimeTest(unittest.TestCase):
    def make_renderer(self, core=None):
        core = core or FakeCore()
        renderer = gsplat_renderer_tcp.TcpRenderer(core, "127.0.0.1", 0, 0)
        renderer.start()
        self.addCleanup(renderer.stop)
        return renderer, core

    def connect(self, port):
        client = socket.create_connection(("127.0.0.1", port), timeout=3.0)
        self.addCleanup(client.close)
        return client

    def latest_pose_seq(self, renderer):
        with renderer.state_lock:
            return None if renderer.latest_pose is None else renderer.latest_pose.seq

    def send_pose(self, client, seq, position=None, stamp_sec=100, stamp_nsec=500):
        header = {
            "version": 1,
            "type": "pose",
            "seq": seq,
            "stamp_sec": stamp_sec,
            "stamp_nsec": stamp_nsec,
            "position": position or [float(seq), 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0, 1.0],
        }
        tcp_protocol.send_message(client, header)

    def receive_frame(self, client):
        header, payload = tcp_protocol.receive_message(client)
        self.assertEqual(header["type"], "frame")
        return header, payload

    def pending_depth(self, renderer):
        with renderer.frame_lock:
            return 0 if renderer.pending_frame is None else 1

    def test_no_pose_no_render(self):
        renderer, core = self.make_renderer()
        time.sleep(0.2)
        self.assertEqual(core.render_calls, [])
        self.assertIsNone(self.latest_pose_seq(renderer))

    def test_latest_pose_wins(self):
        renderer, core = self.make_renderer()
        pose = self.connect(renderer.pose_port)

        self.send_pose(pose, 1, [1.0, 0.0, 0.0])
        self.send_pose(pose, 2, [2.0, 0.0, 0.0])
        self.assertTrue(wait_until(lambda: self.latest_pose_seq(renderer) == 2))

        renderer.render_once()
        self.assertEqual(len(core.render_calls), 1)
        self.assertEqual(core.render_calls[0][0], [2.0, 0.0, 0.0])

        renderer.render_once()
        self.assertEqual(len(core.render_calls), 1, "已消费的位姿不应重复渲染")

        self.send_pose(pose, 3, [3.0, 0.0, 0.0])
        self.assertTrue(wait_until(lambda: self.latest_pose_seq(renderer) == 3))
        renderer.render_once()
        self.assertEqual(len(core.render_calls), 2)
        self.assertEqual(core.render_calls[1][0], [3.0, 0.0, 0.0])

    def test_source_stamp_preserved(self):
        renderer, core = self.make_renderer()
        frame_client = self.connect(renderer.frame_port)
        info, _ = tcp_protocol.receive_message(frame_client)
        self.assertEqual(info["type"], "camera_info")

        pose = self.connect(renderer.pose_port)
        self.send_pose(pose, 1, [1.0, 0.0, 0.0], stamp_sec=100, stamp_nsec=500)
        self.assertTrue(wait_until(lambda: self.latest_pose_seq(renderer) == 1))
        renderer.render_once()
        header, _ = self.receive_frame(frame_client)
        self.assertEqual((header["stamp_sec"], header["stamp_nsec"]), (100, 500))
        self.assertEqual(header["pose_seq"], 1)

        # 相同时间戳的新 seq 仍要消费。
        self.send_pose(pose, 2, [2.0, 0.0, 0.0], stamp_sec=100, stamp_nsec=500)
        self.assertTrue(wait_until(lambda: self.latest_pose_seq(renderer) == 2))
        renderer.render_once()
        header, _ = self.receive_frame(frame_client)
        self.assertEqual((header["stamp_sec"], header["stamp_nsec"]), (100, 500))
        self.assertEqual(header["pose_seq"], 2)

        # 仿真时间回退的新 seq 仍要消费。
        self.send_pose(pose, 3, [3.0, 0.0, 0.0], stamp_sec=50, stamp_nsec=0)
        self.assertTrue(wait_until(lambda: self.latest_pose_seq(renderer) == 3))
        renderer.render_once()
        header, _ = self.receive_frame(frame_client)
        self.assertEqual((header["stamp_sec"], header["stamp_nsec"]), (50, 0))
        self.assertEqual(header["pose_seq"], 3)
        self.assertEqual(len(core.render_calls), 3)

    def test_slow_reader_bounded_queue(self):
        core = FakeCore(width=640, height=480)
        renderer, core = self.make_renderer(core)
        slow_reader = socket.socket()
        slow_reader.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
        slow_reader.connect(("127.0.0.1", renderer.frame_port))
        self.addCleanup(slow_reader.close)
        info, _ = tcp_protocol.receive_message(slow_reader)
        self.assertEqual(info["type"], "camera_info")

        pose = self.connect(renderer.pose_port)
        for seq in range(1, 9):
            self.send_pose(pose, seq, [float(seq), 0.0, 0.0])
            self.assertTrue(wait_until(lambda seq=seq: self.latest_pose_seq(renderer) == seq))
            renderer.render_once()
            self.assertLessEqual(self.pending_depth(renderer), 1)

        self.assertEqual(len(core.render_calls), 8, "慢读期间渲染仍应前进")
        self.send_pose(pose, 9, [9.0, 0.0, 0.0])
        self.assertTrue(wait_until(lambda: self.latest_pose_seq(renderer) == 9),
                        "慢读期间位姿接收仍应前进")

    def clear_frame_client(self, renderer, pose, first_seq=100):
        seq = first_seq
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            with renderer.frame_lock:
                if renderer.frame_client is None:
                    return
            seq += 1
            self.send_pose(pose, seq, [0.0, 0.0, 0.0])
            self.assertTrue(wait_until(lambda seq=seq: self.latest_pose_seq(renderer) == seq))
            renderer.render_once()
            time.sleep(0.02)
        self.fail("renderer 未清除已断开的图像客户端")

    def test_reconnect_metadata_first(self):
        renderer, core = self.make_renderer()
        frame_client = self.connect(renderer.frame_port)
        info, _ = tcp_protocol.receive_message(frame_client)
        self.assertEqual(info["type"], "camera_info")

        pose = self.connect(renderer.pose_port)
        self.send_pose(pose, 1, [1.0, 0.0, 0.0])
        self.assertTrue(wait_until(lambda: self.latest_pose_seq(renderer) == 1))
        renderer.render_once()
        old_header, _ = self.receive_frame(frame_client)
        old_frame_seq = old_header["frame_seq"]

        frame_client.shutdown(socket.SHUT_RDWR)
        frame_client.close()
        self.clear_frame_client(renderer, pose)

        # 无客户端期间生成积压帧，新连接不得补发。
        for seq in range(200, 204):
            self.send_pose(pose, seq, [float(seq), 0.0, 0.0])
            self.assertTrue(wait_until(lambda seq=seq: self.latest_pose_seq(renderer) == seq))
            renderer.render_once()

        new_client = self.connect(renderer.frame_port)
        info, _ = tcp_protocol.receive_message(new_client)
        self.assertEqual(info["type"], "camera_info")

        new_client.settimeout(0.3)
        with self.assertRaises(TimeoutError):
            tcp_protocol.receive_message(new_client)
        new_client.settimeout(3.0)

        self.send_pose(pose, 300, [300.0, 0.0, 0.0])
        self.assertTrue(wait_until(lambda: self.latest_pose_seq(renderer) == 300))
        renderer.render_once()
        header, _ = self.receive_frame(new_client)
        self.assertEqual(header["pose_seq"], 300)
        self.assertGreater(header["frame_seq"], old_frame_seq)

    def test_pose_reconnect_clears_state(self):
        renderer, core = self.make_renderer()
        first_pose = self.connect(renderer.pose_port)
        self.send_pose(first_pose, 1, [1.0, 0.0, 0.0])
        self.assertTrue(wait_until(lambda: self.latest_pose_seq(renderer) == 1))

        first_pose.close()
        self.assertTrue(wait_until(lambda: self.latest_pose_seq(renderer) is None),
                        "位姿断开后应清空旧位姿")

        second_pose = self.connect(renderer.pose_port)
        self.assertTrue(wait_until(lambda: renderer.pose_client is not None))
        renderer.render_once()
        self.assertEqual(core.render_calls, [], "新连接收到位姿前不得渲染旧位姿")

        self.send_pose(second_pose, 1, [7.0, 0.0, 0.0])
        self.assertTrue(wait_until(lambda: self.latest_pose_seq(renderer) == 1))
        renderer.render_once()
        self.assertEqual(core.render_calls[-1][0], [7.0, 0.0, 0.0])

    def assert_closed(self, client):
        client.settimeout(1.0)
        try:
            data = client.recv(1)
        except ConnectionResetError:
            return
        self.assertEqual(data, b"", "第二个连接应被关闭")

    def test_second_client_rejected(self):
        renderer, core = self.make_renderer()
        first_pose = self.connect(renderer.pose_port)
        self.send_pose(first_pose, 1)
        self.assertTrue(wait_until(lambda: self.latest_pose_seq(renderer) == 1))

        second_pose = socket.create_connection(("127.0.0.1", renderer.pose_port), timeout=3.0)
        self.addCleanup(second_pose.close)
        self.assert_closed(second_pose)

        self.send_pose(first_pose, 2)
        self.assertTrue(wait_until(lambda: self.latest_pose_seq(renderer) == 2),
                        "被拒绝的新连接不应影响现有位姿连接")

        first_frame = self.connect(renderer.frame_port)
        info, _ = tcp_protocol.receive_message(first_frame)
        self.assertEqual(info["type"], "camera_info")

        second_frame = socket.create_connection(("127.0.0.1", renderer.frame_port), timeout=3.0)
        self.addCleanup(second_frame.close)
        self.assert_closed(second_frame)

        renderer.render_once()
        header, _ = self.receive_frame(first_frame)
        self.assertEqual(header["type"], "frame")

    def test_stop_closes_connections(self):
        renderer, core = self.make_renderer()
        pose = self.connect(renderer.pose_port)
        frame = self.connect(renderer.frame_port)
        tcp_protocol.receive_message(frame)

        renderer.stop()
        for thread in renderer.threads:
            self.assertFalse(thread.is_alive(), "%s 未退出" % thread.name)

        for client in (pose, frame):
            client.settimeout(1.0)
            try:
                data = client.recv(1)
            except ConnectionResetError:
                continue
            self.assertEqual(data, b"", "stop 后连接应关闭")

        for port in (renderer.pose_port, renderer.frame_port):
            probe = socket.socket()
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(("127.0.0.1", port))
            probe.close()

    def test_oversized_frame_configuration_rejected(self):
        core = FakeCore(width=8000, height=8000)
        with self.assertRaises(ValueError):
            gsplat_renderer_tcp.TcpRenderer(core, "127.0.0.1", 0, 0)

    def test_second_client_rejection_logged(self):
        renderer, core = self.make_renderer()
        first_pose = self.connect(renderer.pose_port)
        self.send_pose(first_pose, 1)
        self.assertTrue(wait_until(lambda: self.latest_pose_seq(renderer) == 1))

        with self.assertLogs("gsplat_renderer_tcp", level="WARNING") as logs:
            second_pose = socket.create_connection(
                ("127.0.0.1", renderer.pose_port), timeout=3.0)
            self.addCleanup(second_pose.close)
            self.assert_closed(second_pose)

        self.assertTrue(any("位姿" in line and ("拒绝" in line or "已有" in line)
                            for line in logs.output), logs.output)

    def test_import_does_not_load_gpu_core(self):
        code = (
            "import sys\n"
            "sys.path.insert(0, %r)\n"
            "import gsplat_renderer_tcp\n"
            "forbidden = [name for name in ('gsplat_core', 'torch', 'numpy', 'gsplat', "
            "'rospy', 'cv_bridge', 'sensor_msgs') if name in sys.modules]\n"
            "assert not forbidden, forbidden\n" % str(RENDERER_DIR)
        )
        result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
