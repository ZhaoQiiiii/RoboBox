import socket
import time
import types
import unittest

import numpy as np

from frame_protocol import recv_frame
from ros_tee import (
    RateGate,
    SinkLink,
    cloud_frame,
    compressed_image_frame,
    depth_image_frame,
    marker_frame,
    marker_frames,
    odom_frame,
    path_frame,
    split_topic_spec,
    subscription_plan,
)


def _header(frame_id="world", sec=3.0):
    return types.SimpleNamespace(
        frame_id=frame_id, stamp=types.SimpleNamespace(to_sec=lambda: sec)
    )


def _pose(x=1.0, y=2.0, z=3.0):
    return types.SimpleNamespace(
        position=types.SimpleNamespace(x=x, y=y, z=z),
        orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def _marker(type_, points=(), colors=(), ns="super", marker_id=7, action=0):
    return types.SimpleNamespace(
        header=_header(),
        ns=ns,
        id=marker_id,
        type=type_,
        action=action,
        pose=_pose(),
        scale=types.SimpleNamespace(x=1.0, y=2.0, z=3.0),
        color=types.SimpleNamespace(r=0.0, g=1.0, b=0.0, a=1.0),
        points=[types.SimpleNamespace(x=p[0], y=p[1], z=p[2]) for p in points],
        colors=[
            types.SimpleNamespace(r=c[0], g=c[1], b=c[2], a=c[3]) for c in colors
        ],
        text="",
    )


class TeeTest(unittest.TestCase):
    def test_rate_gate(self):
        gate = RateGate(10.0)
        self.assertTrue(gate.allow(0.0))
        self.assertFalse(gate.allow(0.05))
        self.assertTrue(gate.allow(0.11))

    def test_compressed_image_frame(self):
        msg = types.SimpleNamespace(
            data=b"\xff\xd8jpeg",
            header=types.SimpleNamespace(
                frame_id="camera0", stamp=types.SimpleNamespace(to_sec=lambda: 3.0)
            ),
        )
        header, payload = compressed_image_frame(msg)
        self.assertEqual(header["kind"], "color")
        self.assertEqual(header["media_type"], "image/jpeg")
        self.assertEqual(payload, b"\xff\xd8jpeg")

    def test_depth_image_frame(self):
        msg = types.SimpleNamespace(
            encoding="16UC1",
            width=2,
            height=2,
            data=b"\x00" * 8,
            header=types.SimpleNamespace(
                frame_id="camera0", stamp=types.SimpleNamespace(to_sec=lambda: 3.0)
            ),
        )
        header, payload = depth_image_frame(msg)
        self.assertEqual(header["kind"], "depth")
        self.assertEqual((header["width"], header["height"]), (2, 2))

    def test_depth_rejects_float_encoding(self):
        msg = types.SimpleNamespace(
            encoding="32FC1",
            width=2,
            height=2,
            data=b"",
            header=types.SimpleNamespace(
                frame_id="camera0", stamp=types.SimpleNamespace(to_sec=lambda: 3.0)
            ),
        )
        with self.assertRaises(ValueError):
            depth_image_frame(msg)

    def test_cloud_frame_downsample(self):
        points = np.arange(400, dtype="<f4").reshape(100, 4)
        msg = types.SimpleNamespace(
            data=points.tobytes(),
            point_step=16,
            is_bigendian=False,
            fields=[
                types.SimpleNamespace(name=n, offset=o, datatype=7)
                for n, o in [("x", 0), ("y", 4), ("z", 8), ("intensity", 12)]
            ],
            header=types.SimpleNamespace(
                frame_id="world", stamp=types.SimpleNamespace(to_sec=lambda: 3.0)
            ),
        )
        header, payload = cloud_frame(msg, 10)
        self.assertEqual(header["kind"], "cloud")
        self.assertEqual(header["point_count"], 10)
        self.assertEqual(len(payload), 10 * 16)

    def test_sink_link_drops_without_blocking(self):
        link = SinkLink("127.0.0.1:1")  # 没有监听者
        started = time.monotonic()
        self.assertFalse(link.send({"kind": "color"}, b"x"))
        self.assertLess(time.monotonic() - started, 0.5)

    def test_sink_link_sends_to_listener(self):
        server = socket.socket()
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        try:
            link = SinkLink("127.0.0.1:%d" % server.getsockname()[1])
            try:
                self.assertTrue(link.send({"kind": "color"}, b"jpeg"))
                conn, _ = server.accept()
                try:
                    header, payload = recv_frame(conn)
                finally:
                    conn.close()
                self.assertEqual(header["kind"], "color")
                self.assertEqual(payload, b"jpeg")
            finally:
                link.close()
        finally:
            server.close()


class OdomMarkerFrameTest(unittest.TestCase):
    def test_odom_frame(self):
        msg = types.SimpleNamespace(
            header=_header(), pose=types.SimpleNamespace(pose=_pose())
        )
        header, payload = odom_frame(msg)
        self.assertEqual(header["kind"], "odom")
        self.assertEqual(header["position"], [1.0, 2.0, 3.0])
        self.assertEqual(header["orientation"], [0.0, 0.0, 0.0, 1.0])
        self.assertEqual(payload, b"")

    def test_marker_line_strip_frame(self):
        header, payload = marker_frame(
            _marker(4, [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
        )
        self.assertEqual(header["kind"], "marker")
        self.assertEqual((header["ns"], header["id"], header["type"]), ("super", 7, 4))
        self.assertEqual(header["point_count"], 2)
        self.assertEqual(len(payload), 2 * 3 * 4)

    def test_marker_frames_splits_array(self):
        msg = types.SimpleNamespace(markers=[_marker(4), _marker(1, marker_id=8)])
        frames = marker_frames(msg)
        self.assertEqual([frame[0]["id"] for frame in frames], [7, 8])

    def test_marker_keeps_per_point_colors(self):
        header, payload = marker_frame(
            _marker(8, [(0.0, 0.0, 0.0)], [(1.0, 0.5, 0.0, 1.0)])
        )
        self.assertEqual(header["color_count"], 1)
        self.assertEqual(len(payload), 3 * 4 + 4 * 4)

    def test_path_frame(self):
        msg = types.SimpleNamespace(
            header=_header("world"),
            poses=[
                types.SimpleNamespace(pose=_pose(0.0, 0.0, 0.0)),
                types.SimpleNamespace(pose=_pose(1.0, 2.0, 3.0)),
            ],
        )
        header, payload = path_frame(msg)
        self.assertEqual(header["kind"], "path")
        self.assertEqual(header["point_count"], 2)
        np.testing.assert_allclose(
            np.frombuffer(payload, dtype="<f4").reshape(-1, 3),
            [[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]],
        )

    def test_split_topic_spec(self):
        self.assertEqual(split_topic_spec("/a@/b/c"), ("/a", "/b/c"))
        self.assertEqual(split_topic_spec("/a"), ("/a", ""))


class SubscriptionPlanTest(unittest.TestCase):
    def _args(self, **overrides):
        values = {
            "image_topic": "",
            "depth_topic": "",
            "cloud_topic": [],
            "odom_topic": "",
            "path_topic": [],
            "marker_topic": [],
            "marker_array_topic": [],
            "image_hz": 15.0,
            "depth_hz": 5.0,
            "cloud_hz": 5.0,
            "odom_hz": 10.0,
            "marker_hz": 5.0,
            "path_hz": 5.0,
            "max_points": 100,
        }
        values.update(overrides)
        return types.SimpleNamespace(**values)

    def test_plan_includes_odom_and_markers(self):
        plan = subscription_plan(
            self._args(
                odom_topic="/odom",
                marker_topic=["/fov"],
                marker_array_topic=["/map"],
            )
        )
        summary = [
            (kind, topic, msg_type, multi, base)
            for kind, topic, msg_type, _builder, _hz, multi, base in plan
        ]
        self.assertEqual(
            summary,
            [
                ("odom", "/odom", "Odometry", False, ""),
                ("marker", "/fov", "Marker", False, ""),
                ("marker", "/map", "MarkerArray", True, ""),
            ],
        )

    def test_plan_carries_topic_bases(self):
        plan = subscription_plan(
            self._args(
                cloud_topic=["/cloud@/world/drone/perception/lidar/cloud"],
                marker_topic=["/fov@/world/drone/planning/fov"],
                marker_array_topic=["/graph@/world/common/graph_nav/graph"],
                path_topic=["/path@/world/drone/planning/path"],
            )
        )
        summary = [
            (kind, topic, msg_type, multi, base)
            for kind, topic, msg_type, _builder, _hz, multi, base in plan
        ]
        self.assertEqual(
            summary,
            [
                ("cloud", "/cloud", "PointCloud2", False, "/world/drone/perception/lidar/cloud"),
                ("marker", "/fov", "Marker", False, "/world/drone/planning/fov"),
                ("marker", "/graph", "MarkerArray", True, "/world/common/graph_nav/graph"),
                ("path", "/path", "Path", False, "/world/drone/planning/path"),
            ],
        )

    def test_plan_skips_empty_topics(self):
        self.assertEqual(subscription_plan(self._args()), [])


if __name__ == "__main__":
    unittest.main()
