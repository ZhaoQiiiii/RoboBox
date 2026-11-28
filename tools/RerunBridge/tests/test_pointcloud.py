import types
import unittest

import numpy as np

from pointcloud import CloudFormatError, cloud_colors, parse_cloud2, subsample


def cloud_msg(points, fields=None, point_step=16, is_bigendian=False):
    fields = fields or [
        {"name": "x", "offset": 0, "datatype": 7},
        {"name": "y", "offset": 4, "datatype": 7},
        {"name": "z", "offset": 8, "datatype": 7},
        {"name": "intensity", "offset": 12, "datatype": 7},
    ]
    return types.SimpleNamespace(
        data=np.asarray(points, dtype="<f4").tobytes(),
        point_step=point_step,
        is_bigendian=is_bigendian,
        fields=[types.SimpleNamespace(**field) for field in fields],
    )


class PointCloudTest(unittest.TestCase):
    def test_parses_xyz_intensity(self):
        points = np.array([[1, 2, 3, 0.5], [4, 5, 6, 1.0]], dtype="<f4")
        parsed = parse_cloud2(cloud_msg(points))
        np.testing.assert_allclose(parsed, points)

    def test_missing_intensity_is_zero(self):
        fields = [
            {"name": "x", "offset": 0, "datatype": 7},
            {"name": "y", "offset": 4, "datatype": 7},
            {"name": "z", "offset": 8, "datatype": 7},
        ]
        parsed = parse_cloud2(
            cloud_msg(np.array([[1, 2, 3]], dtype="<f4"), fields, 12)
        )
        np.testing.assert_allclose(parsed, [[1, 2, 3, 0]])

    def test_rejects_big_endian(self):
        with self.assertRaises(CloudFormatError):
            parse_cloud2(cloud_msg(np.zeros((1, 4), "<f4"), is_bigendian=True))

    def test_rejects_non_float32(self):
        fields = [{"name": "x", "offset": 0, "datatype": 6}]  # FLOAT64
        with self.assertRaises(CloudFormatError):
            parse_cloud2(cloud_msg(np.zeros((1, 4), "<f4"), fields))

    def test_subsample_is_stride_based(self):
        points = np.arange(100, dtype="<f4").reshape(25, 4)
        kept = subsample(points, 5)
        self.assertEqual(len(kept), 5)
        np.testing.assert_allclose(kept[0], points[0])

    def test_cloud_colors_in_uint8_range(self):
        colors = cloud_colors(np.array([0.0, 50.0, 100.0], dtype=np.float32))
        self.assertEqual(colors.shape, (3, 3))
        self.assertEqual(colors.dtype, np.uint8)
        # 低强度偏蓝（B > R），高强度偏红（R > B）。
        self.assertGreater(int(colors[0][2]), int(colors[0][0]))
        self.assertGreater(int(colors[-1][0]), int(colors[-1][2]))


if __name__ == "__main__":
    unittest.main()
