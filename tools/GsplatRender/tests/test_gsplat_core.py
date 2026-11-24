import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np
import torch
import yaml
from plyfile import PlyData, PlyElement
from scipy.spatial.transform import Rotation

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
RENDERER_DIR = PACKAGE_ROOT / "renderer"
PARAMS_FILE = PACKAGE_ROOT / "config" / "renderer_params.yaml"
sys.path.insert(0, str(RENDERER_DIR))

import gsplat_core


def write_tiny_ply(path):
    vertex = np.zeros(2, dtype=[
        ("x", "f4"), ("y", "f4"), ("z", "f4"),
        ("f_dc_0", "f4"), ("f_dc_1", "f4"), ("f_dc_2", "f4"),
        ("opacity", "f4"),
        ("rot_0", "f4"), ("rot_1", "f4"), ("rot_2", "f4"), ("rot_3", "f4"),
        ("scale_0", "f4"), ("scale_1", "f4"), ("scale_2", "f4"),
    ])
    vertex["rot_0"] = 1.0
    vertex["scale_0"] = -3.0
    vertex["opacity"] = 1.0
    PlyData([PlyElement.describe(vertex, "vertex")], text=False).write(str(path))


def legacy_view_matrix(position, orientation, enable_gimbal):
    if enable_gimbal:
        yaw = Rotation.from_quat(orientation).as_euler("zyx")[0]
        R_w2b = Rotation.from_euler("z", yaw).as_matrix().T
    else:
        R_w2b = Rotation.from_quat(orientation).as_matrix().T

    R_b2c = np.array([
        [0, -1, 0],
        [0, 0, -1],
        [1, 0, 0],
    ], dtype=np.float32)

    R = R_b2c @ R_w2b
    t = -R @ position
    viewmat = np.eye(4, dtype=np.float32)
    viewmat[:3, :3] = R
    viewmat[:3, 3] = t
    return viewmat


class GsplatCoreTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp_dir = tempfile.TemporaryDirectory()
        cls.ply_file = Path(cls.temp_dir.name) / "tiny.ply"
        write_tiny_ply(cls.ply_file)

    @classmethod
    def tearDownClass(cls):
        cls.temp_dir.cleanup()

    def make_core(self, enable_gimbal=False):
        return gsplat_core.GsplatCore(self.ply_file, PARAMS_FILE, enable_gimbal)

    def test_rgb_channels_and_depth_mask(self):
        rgb = np.zeros((2, 3, 3), dtype=np.float32)
        rgb[0, 0] = [1.0, 0.0, 0.0]
        depth = np.full((2, 3, 1), 1.25, dtype=np.float32)
        alpha = np.ones((2, 3, 1), dtype=np.float32)

        rgb_uint8, depth_uint16 = gsplat_core.encode_render_arrays(rgb, depth, alpha)
        self.assertEqual(rgb_uint8.dtype, np.uint8)
        self.assertEqual(depth_uint16.dtype, np.uint16)
        np.testing.assert_array_equal(rgb_uint8[0, 0], [255, 0, 0])
        np.testing.assert_array_equal(depth_uint16, np.full((2, 3), 1250, dtype=np.uint16))

        depth = np.full((2, 3, 1), 1.25, dtype=np.float32)
        alpha = np.full((2, 3, 1), 0.49, dtype=np.float32)
        _, depth_uint16 = gsplat_core.encode_render_arrays(rgb, depth, alpha)
        np.testing.assert_array_equal(depth_uint16, np.zeros((2, 3), dtype=np.uint16))

        alpha = np.full((2, 3, 1), 0.5, dtype=np.float32)
        _, depth_uint16 = gsplat_core.encode_render_arrays(rgb, depth, alpha)
        np.testing.assert_array_equal(depth_uint16, np.full((2, 3), 1250, dtype=np.uint16))

        depth = np.full((2, 3, 1), 0.05, dtype=np.float32)
        _, depth_uint16 = gsplat_core.encode_render_arrays(rgb, depth, np.ones((2, 3, 1), np.float32))
        np.testing.assert_array_equal(depth_uint16, np.full((2, 3), 100, dtype=np.uint16))

        depth = np.full((2, 3, 1), 25.0, dtype=np.float32)
        _, depth_uint16 = gsplat_core.encode_render_arrays(rgb, depth, np.ones((2, 3, 1), np.float32))
        np.testing.assert_array_equal(depth_uint16, np.full((2, 3), 20000, dtype=np.uint16))

        # 旧实现先转 uint16 再裁剪：70000 mm 溢出为 4464，不裁剪到 20000。
        depth = np.full((2, 3, 1), 70.0, dtype=np.float32)
        _, depth_uint16 = gsplat_core.encode_render_arrays(rgb, depth, np.ones((2, 3, 1), np.float32))
        np.testing.assert_array_equal(depth_uint16, np.full((2, 3), 4464, dtype=np.uint16))

    def test_camera0_view_matrix(self):
        core = self.make_core()
        cases = [
            (np.array([1.0, 2.0, 3.0], dtype=np.float32), [0.0, 0.0, 0.0, 1.0]),
            (np.array([-1.5, 0.25, 2.0], dtype=np.float32),
             Rotation.from_euler("z", 60.0, degrees=True).as_quat()),
            (np.array([0.5, -2.0, 1.0], dtype=np.float32),
             Rotation.from_euler("xyz", [20.0, -15.0, 110.0], degrees=True).as_quat()),
        ]
        for position, orientation in cases:
            with self.subTest(position=position, orientation=orientation):
                expected = legacy_view_matrix(position, np.asarray(orientation, dtype=np.float32),
                                              enable_gimbal=False)
                actual = core.camera0_view_matrix(position, np.asarray(orientation, dtype=np.float32))
                self.assertEqual(actual.dtype, np.float32)
                np.testing.assert_allclose(actual, expected, rtol=1e-6, atol=1e-6)
                np.testing.assert_array_equal(actual[3], [0.0, 0.0, 0.0, 1.0])

    def test_gimbal_ignores_roll_pitch(self):
        position = np.array([1.0, -2.0, 3.0], dtype=np.float32)
        orientation = Rotation.from_euler("xyz", [30.0, 20.0, 45.0], degrees=True).as_quat()

        gimbal_core = self.make_core(enable_gimbal=True)
        expected = legacy_view_matrix(position, orientation, enable_gimbal=True)
        actual = gimbal_core.camera0_view_matrix(position, orientation)
        np.testing.assert_allclose(actual, expected, rtol=1e-6, atol=1e-6)

        full_core = self.make_core(enable_gimbal=False)
        full = full_core.camera0_view_matrix(position, orientation)
        self.assertFalse(np.allclose(actual, full))

    def test_render_only_camera0(self):
        core = self.make_core()
        core.width = 4
        core.height = 3
        calls = []

        def fake_rasterization(**kwargs):
            calls.append(kwargs)
            colors = torch.ones(1, kwargs["height"], kwargs["width"], 4, dtype=torch.float32)
            colors[..., :3] = 0.5
            colors[..., 3] = 2.0
            alphas = torch.ones(1, kwargs["height"], kwargs["width"], 1, dtype=torch.float32)
            return colors, alphas, {}

        position = np.array([1.0, 2.0, 3.0], dtype=np.float32)
        orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        with mock.patch.object(gsplat_core, "rasterization", fake_rasterization):
            rgb_uint8, depth_uint16 = core.render(position, orientation)
            rgb_again, _ = core.render(position, orientation)

        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[0]["render_mode"], "RGB+ED")
        self.assertEqual(calls[0]["width"], 4)
        self.assertEqual(calls[0]["height"], 3)
        self.assertEqual(tuple(calls[0]["viewmats"].shape), (1, 4, 4))
        self.assertEqual(tuple(calls[0]["Ks"].shape), (1, 3, 3))
        self.assertEqual(rgb_uint8.shape, (3, 4, 3))
        self.assertEqual(depth_uint16.shape, (3, 4))
        np.testing.assert_array_equal(rgb_uint8, np.full((3, 4, 3), 127, dtype=np.uint8))
        np.testing.assert_array_equal(depth_uint16, np.full((3, 4), 2000, dtype=np.uint16))
        np.testing.assert_array_equal(rgb_again, rgb_uint8)

    def test_camera_metadata_matches_yaml(self):
        core = self.make_core()
        with open(PARAMS_FILE, "r") as params_stream:
            render_params = yaml.safe_load(params_stream)["render"]

        metadata = core.camera_metadata()
        self.assertEqual(metadata["width"], render_params["width"])
        self.assertEqual(metadata["height"], render_params["height"])
        self.assertEqual(metadata["fx"], render_params["fx"])
        self.assertEqual(metadata["fy"], render_params["fy"])
        self.assertEqual(metadata["cx"], render_params["cx"])
        self.assertEqual(metadata["cy"], render_params["cy"])
        self.assertEqual(metadata["frame_id"], "camera")
        self.assertEqual(metadata["cam_names"], ["camera0", "camera1", "camera2", "camera3"])
        self.assertEqual(metadata["rgb_encoding"], "rgb8")
        self.assertEqual(metadata["depth_encoding"], "16UC1")

    def test_import_does_not_pull_ros(self):
        code = (
            "import sys\n"
            "sys.path.insert(0, %r)\n"
            "import gsplat_core\n"
            "forbidden = [name for name in ('rospy', 'cv_bridge', 'sensor_msgs') if name in sys.modules]\n"
            "assert not forbidden, forbidden\n" % str(RENDERER_DIR)
        )
        result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
