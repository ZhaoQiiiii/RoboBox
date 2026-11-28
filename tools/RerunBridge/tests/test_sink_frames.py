import socket
import struct
import unittest
from unittest import mock

import numpy as np
import rerun as rr

from frame_protocol import send_frame
from rerun_sink import (
    OdomTrail,
    ServerHealth,
    build_archetype,
    build_blueprint,
    marker_archetype,
    probe_server,
    pump_connection,
    retry_connect,
    serve_stream,
    world_frame_mismatch,
)


def _marker_header(**overrides):
    header = {
        "kind": "marker",
        "stamp": [0, 0],
        "frame_id": "world",
        "ns": "fov",
        "id": 1,
        "type": 4,
        "action": 0,
        "position": [0.0, 0.0, 0.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
        "scale": [1.0, 1.0, 1.0],
        "color": [0.0, 1.0, 0.0, 1.0],
        "text": "",
        "point_count": 2,
        "color_count": 0,
    }
    header.update(overrides)
    return header


class SinkFramesTest(unittest.TestCase):
    def test_color_returns_encoded_image(self):
        entity, archetype = build_archetype(
            {"kind": "color", "media_type": "image/jpeg"}, b"\xff\xd8jpeg"
        )
        self.assertEqual(entity, "/world/robokit/perception/camera/color")
        self.assertIsInstance(archetype, rr.EncodedImage)

    def test_custom_entity_prefix(self):
        entity, _ = build_archetype(
            {"kind": "color", "media_type": "image/jpeg"},
            b"\xff\xd8x",
            entity_prefix="/world/drone",
        )
        self.assertEqual(entity, "/world/drone/perception/camera/color")

    def test_blueprint_uses_the_entity_prefix(self):
        import rerun.blueprint as rrb

        blueprint = build_blueprint("/world/drone")
        root = blueprint.root_container
        self.assertIsInstance(root, rrb.Horizontal)
        self.assertEqual(root.contents[1].origin, "/world/drone/perception/camera/color")

    def test_depth_returns_depth_image(self):
        payload = np.array([[100, 200], [300, 400]], dtype="<u2").tobytes()
        entity, archetype = build_archetype(
            {"kind": "depth", "width": 2, "height": 2}, payload
        )
        self.assertEqual(entity, "/world/robokit/perception/camera/depth")
        self.assertIsInstance(archetype, rr.DepthImage)

    def test_cloud_returns_points3d(self):
        points = np.array([[0, 0, 0, 0], [1, 1, 1, 100]], dtype="<f4")
        entity, archetype = build_archetype(
            {"kind": "cloud", "point_count": 2}, points.tobytes()
        )
        self.assertEqual(entity, "/world/robokit/perception/lidar/cloud")
        self.assertIsInstance(archetype, rr.Points3D)

    def test_cloud_uses_an_explicit_base(self):
        points = np.array([[0, 0, 0, 0]], dtype="<f4")
        entity, _ = build_archetype(
            {
                "kind": "cloud",
                "base": "/world/drone/perception/rog_map/inf_occ",
                "point_count": 1,
            },
            points.tobytes(),
        )
        self.assertEqual(entity, "/world/drone/perception/rog_map/inf_occ")

    def test_path_returns_line_strip(self):
        points = np.array([[0, 0, 0], [1, 0, 0]], dtype="<f4")
        entity, archetype = build_archetype(
            {
                "kind": "path",
                "base": "/world/drone/planning/path",
                "point_count": 2,
            },
            points.tobytes(),
        )
        self.assertEqual(entity, "/world/drone/planning/path")
        self.assertIsInstance(archetype, rr.LineStrips3D)

    def test_unknown_kind_raises(self):
        with self.assertRaises(ValueError):
            build_archetype({"kind": "imu"}, b"")

    def test_retry_connect_retries_until_success(self):
        calls = []

        def connect():
            calls.append(1)
            if len(calls) < 3:
                raise RuntimeError("server not ready")
            return "stream"

        self.assertEqual(
            retry_connect(connect, delay=0.0, sleep=lambda _: None), "stream"
        )
        self.assertEqual(len(calls), 3)

    def test_probe_server_raises_when_nothing_listens(self):
        with self.assertRaises(OSError):
            probe_server("rerun+http://127.0.0.1:1/proxy", timeout=0.2)

    def test_probe_server_accepts_listener(self):
        with socket.socket() as server:
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            probe_server(
                "rerun+http://127.0.0.1:%d/proxy" % server.getsockname()[1],
                timeout=0.2,
            )

    def test_server_health_clears_and_recovers(self):
        state = {"up": False}

        def probe(_url):
            if not state["up"]:
                raise OSError("no server")

        health = ServerHealth("rerun+http://127.0.0.1:1/proxy", probe=probe)
        self.assertFalse(health.check())
        state["up"] = True
        self.assertTrue(health.check())

    def test_blueprint_is_one_tab_with_lidar_and_gsplat_side_by_side(self):
        import rerun.blueprint as rrb

        blueprint = build_blueprint()
        root = blueprint.root_container
        self.assertIsInstance(root, rrb.Horizontal)
        views = root.contents
        self.assertEqual(views[0].origin, "/world")
        self.assertEqual(views[1].origin, "/world/robokit/perception/camera/color")

    def test_serve_stream_keeps_a_saved_viewer_layout_active(self):
        recording = mock.Mock()
        with (
            mock.patch("rerun.init"),
            mock.patch("rerun.connect_grpc"),
            mock.patch("rerun.get_global_data_recording", return_value=recording),
            mock.patch("rerun_sink.probe_server"),
        ):
            serve_stream("rerun+http://127.0.0.1:9877/proxy", None, "robokit")
        recording.send_blueprint.assert_called_once()
        self.assertIs(recording.send_blueprint.call_args.kwargs.get("make_active"), False)

    def test_world_frame_mismatch_accepts_world(self):
        self.assertIsNone(world_frame_mismatch({"frame_id": "world"}))
        self.assertIsNone(world_frame_mismatch({"frame_id": "/world"}))
        self.assertIsNone(world_frame_mismatch({}))

    def test_world_frame_mismatch_reports_odom(self):
        self.assertEqual(world_frame_mismatch({"frame_id": "odom"}), "odom")

    def test_serve_stream_passes_recording_identity(self):
        recording = mock.Mock()
        with (
            mock.patch("rerun.init") as init,
            mock.patch("rerun.connect_grpc"),
            mock.patch("rerun.get_global_data_recording", return_value=recording),
            mock.patch("rerun_sink.probe_server"),
        ):
            serve_stream(
                "rerun+http://127.0.0.1:9877/proxy",
                None,
                "ZeroAgent",
                "123e4567-e89b-42d3-a456-426614174000",
            )
        self.assertEqual(init.call_args.args[0], "ZeroAgent")
        self.assertEqual(
            init.call_args.kwargs["recording_id"],
            "123e4567-e89b-42d3-a456-426614174000",
        )

    def test_pump_connection_returns_on_reset(self):
        received = []
        server, client = socket.socketpair()
        try:
            send_frame(client, {"kind": "color"}, b"jpeg")
            client.sendall(b"RBK1" + (2).to_bytes(4, "big") + (100).to_bytes(4, "big"))
            client.setsockopt(
                socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0)
            )
            client.close()
            logged = pump_connection(server, lambda _entity, _archetype: received.append(1))
            self.assertEqual(logged, 1)
            self.assertEqual(len(received), 1)
        finally:
            server.close()


class OdomMarkerSinkTest(unittest.TestCase):
    def test_odom_trail_accumulates(self):
        trail = OdomTrail()
        updates = trail.update({"kind": "odom", "position": [1.0, 2.0, 3.0]})
        updates += trail.update({"kind": "odom", "position": [4.0, 5.0, 6.0]})
        entities = [entity for entity, _ in updates]
        self.assertIn("/world/robokit/odom/pos", entities)
        self.assertIn("/world/robokit/odom/trail", entities)
        trails = [a for e, a in updates if e == "/world/robokit/odom/trail"]
        self.assertIsInstance(trails[-1], rr.LineStrips3D)

    def test_custom_odom_prefix(self):
        trail = OdomTrail(entity_prefix="/world/drone")
        updates = trail.update({"kind": "odom", "position": [1.0, 2.0, 3.0]})
        entities = [entity for entity, _ in updates]
        self.assertIn("/world/drone/odom/pos", entities)
        self.assertIn("/world/drone/odom/trail", entities)

    def test_robot_body_from_odom(self):
        trail = OdomTrail(entity_prefix="/world/drone", robot_body=True)
        updates = trail.update(
            {
                "kind": "odom",
                "position": [1.0, 2.0, 3.0],
                "orientation": [0.0, 0.0, 0.0, 1.0],
            }
        )
        entities = [entity for entity, _ in updates]
        self.assertIn("/world/drone/state/robot", entities)
        self.assertIn("/world/drone/state/robot/body", entities)
        self.assertIn("/world/drone/state/robot/heading", entities)

    def test_line_strip_marker(self):
        payload = np.array([[0, 0, 0], [1, 0, 0]], dtype="<f4").tobytes()
        entity, archetype = marker_archetype(_marker_header(), payload)
        self.assertEqual(entity, "/world/robokit/markers/fov/1")
        self.assertIsInstance(archetype, rr.LineStrips3D)

    def test_custom_marker_prefix(self):
        payload = np.array([[0, 0, 0], [1, 0, 0]], dtype="<f4").tobytes()
        entity, _ = marker_archetype(
            _marker_header(), payload, entity_prefix="/world/drone"
        )
        self.assertEqual(entity, "/world/drone/markers/fov/1")

    def test_marker_uses_an_explicit_single_base(self):
        payload = np.array([[0, 0, 0], [1, 0, 0]], dtype="<f4").tobytes()
        entity, _ = marker_archetype(
            _marker_header(base="/world/drone/planning/fov"), payload
        )
        self.assertEqual(entity, "/world/drone/planning/fov")

    def test_marker_array_uses_base_and_namespace(self):
        payload = np.array([[0, 0, 0], [1, 0, 0]], dtype="<f4").tobytes()
        entity, _ = marker_archetype(
            _marker_header(
                base="/world/common/graph_nav/skeleton",
                marker_array=True,
            ),
            payload,
        )
        self.assertEqual(entity, "/world/common/graph_nav/skeleton/fov/1")

    def test_arrow_marker(self):
        payload = np.array([[0, 0, 0], [1, 0, 0]], dtype="<f4").tobytes()
        entity, archetype = marker_archetype(_marker_header(type=0), payload)
        self.assertEqual(entity, "/world/robokit/markers/fov/1")
        self.assertIsInstance(archetype, rr.Arrows3D)

    def test_marker_array_delete_all_clears_base(self):
        entity, archetype = marker_archetype(
            _marker_header(
                base="/world/common/graph_nav/graph",
                marker_array=True,
                action=3,
            ),
            b"",
        )
        self.assertEqual(entity, "/world/common/graph_nav/graph")
        self.assertIsInstance(archetype, rr.Clear)

    def test_line_strip_uses_per_point_colors(self):
        points = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]], dtype="<f4")
        colors = np.array(
            [[1.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.0, 1.0], [0.0, 0.0, 1.0, 1.0]],
            dtype="<f4",
        )
        header = _marker_header(
            color=[0.0, 0.0, 0.0, 0.0], point_count=3, color_count=3
        )
        entity, archetype = marker_archetype(
            header, points.tobytes() + colors.tobytes()
        )
        self.assertIsInstance(archetype, rr.LineStrips3D)
        self.assertEqual(len(archetype.colors), 2)

    def test_cube_list_marker(self):
        payload = np.array([[0, 0, 0], [1, 1, 1]], dtype="<f4").tobytes()
        entity, archetype = marker_archetype(_marker_header(type=6), payload)
        self.assertIsInstance(archetype, rr.Boxes3D)

    def test_delete_marker_clears(self):
        entity, archetype = marker_archetype(_marker_header(action=2), b"")
        self.assertIsInstance(archetype, rr.Clear)

    def test_unsupported_text_marker_raises(self):
        with self.assertRaises(ValueError):
            marker_archetype(_marker_header(type=9), b"")

    def test_pump_connection_logs_odom_trail(self):
        received = []
        server, client = socket.socketpair()
        try:
            send_frame(
                client,
                {
                    "kind": "odom",
                    "position": [0.0, 0.0, 0.0],
                    "orientation": [0, 0, 0, 1],
                },
            )
            client.shutdown(socket.SHUT_WR)
            logged = pump_connection(
                server,
                lambda entity, archetype: received.append(entity),
                trail=OdomTrail(),
            )
            self.assertGreaterEqual(logged, 2)
            self.assertIn("/world/robokit/odom/trail", received)
        finally:
            client.close()
            server.close()


if __name__ == "__main__":
    unittest.main()
