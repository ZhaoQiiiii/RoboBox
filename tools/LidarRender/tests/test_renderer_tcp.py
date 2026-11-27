import os
import socket
import subprocess
import sys
import threading
import time
import unittest
from pathlib import Path

import numpy as np

from tcp_protocol import receive_message, send_message

ROOT = Path(__file__).resolve().parent.parent
BINARY = ROOT / "build" / "lidar_render_tcp"
CONFIG = ROOT / "config" / "params.yaml"
PCD = ROOT / "tests" / "data" / "small_map.pcd"
POSE_PORT = 19182
CLOUD_PORT = 19183


def pose_header(z=0.5):
    return {
        "version": 1,
        "type": "pose",
        "seq": 1,
        "stamp_sec": 1,
        "stamp_nsec": 0,
        "position": [0.0, 0.0, z],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }


@unittest.skipUnless(os.environ.get("DISPLAY"), "DISPLAY is required for OpenGL")
class RendererTcpTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not PCD.is_file():
            subprocess.run([sys.executable, str(ROOT / "tests" / "make_small_map.py")],
                           check=True)

    def setUp(self):
        self.process = subprocess.Popen(
            [
                str(BINARY),
                "--pcd_file", str(PCD),
                "--config_file", str(CONFIG),
                "--host", "127.0.0.1",
                "--pose_port", str(POSE_PORT),
                "--cloud_port", str(CLOUD_PORT),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.addCleanup(self.stop_process)
        self.output = []
        self.output_lock = threading.Lock()
        self.reader = threading.Thread(target=self.read_output, daemon=True)
        self.reader.start()
        self.wait_for_log("pose listening")
        self.wait_for_log("cloud listening")

    def read_output(self):
        for line in self.process.stdout:
            with self.output_lock:
                self.output.append(line)

    def stop_process(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        if self.reader.is_alive():
            self.reader.join(timeout=1)
        if self.process.stdout is not None:
            self.process.stdout.close()

    def tearDown(self):
        self.stop_process()

    def wait_for_log(self, marker, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.output_lock:
                text = "".join(self.output)
            if marker in text:
                return
            if self.process.poll() is not None:
                self.fail("renderer exited:\n" + text)
            time.sleep(0.05)
        with self.output_lock:
            text = "".join(self.output)
        self.fail("timeout waiting for %r:\n%s" % (marker, text))

    def test_pose_produces_cloud(self):
        cloud_socket = socket.create_connection(("127.0.0.1", CLOUD_PORT), timeout=5)
        self.addCleanup(cloud_socket.close)
        pose_socket = socket.create_connection(("127.0.0.1", POSE_PORT), timeout=5)
        self.addCleanup(pose_socket.close)
        send_message(pose_socket, pose_header())

        cloud_socket.settimeout(5.0)
        header, payload = receive_message(cloud_socket)
        self.assertEqual(header["type"], "cloud")
        self.assertEqual(header["frame_id"], "world")
        self.assertGreater(header["point_count"], 0)
        self.assertEqual(header["point_bytes"], header["point_count"] * 16)
        self.assertEqual(len(payload), header["point_bytes"])

        points = np.frombuffer(payload, dtype="<f4").reshape(-1, 4)
        self.assertTrue(np.isfinite(points).all())
        offsets = points[:, :3] - np.array([0.0, 0.0, 0.5], dtype=np.float32)
        self.assertLessEqual(float(np.linalg.norm(offsets, axis=1).max()), 10.5)

    def test_empty_scan_sends_zero_point_cloud(self):
        cloud_socket = socket.create_connection(("127.0.0.1", CLOUD_PORT), timeout=5)
        self.addCleanup(cloud_socket.close)
        pose_socket = socket.create_connection(("127.0.0.1", POSE_PORT), timeout=5)
        self.addCleanup(pose_socket.close)
        send_message(pose_socket, pose_header(z=100.0))

        cloud_socket.settimeout(5.0)
        header, payload = receive_message(cloud_socket)
        self.assertEqual(header["point_count"], 0)
        self.assertEqual(header["point_bytes"], 0)
        self.assertEqual(payload, b"")

    def test_no_pose_sends_nothing(self):
        cloud_socket = socket.create_connection(("127.0.0.1", CLOUD_PORT), timeout=5)
        self.addCleanup(cloud_socket.close)
        cloud_socket.settimeout(1.0)
        with self.assertRaises(TimeoutError):
            receive_message(cloud_socket)


if __name__ == "__main__":
    unittest.main()
