import struct
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from bridge_helpers import (
    cloud_payload_to_array,
    load_global_cloud_or_none,
    pose_to_header,
)


def make_odom():
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(secs=1, nsecs=2)),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
    )


class BridgeHelpersTest(unittest.TestCase):
    def test_pose_to_header(self):
        header = pose_to_header(make_odom(), 7)
        self.assertEqual(header, {
            "version": 1,
            "type": "pose",
            "seq": 7,
            "stamp_sec": 1,
            "stamp_nsec": 2,
            "position": [1.0, 2.0, 3.0],
            "orientation": [0.0, 0.0, 0.0, 1.0],
        })

    def test_cloud_payload_to_array(self):
        payload = struct.pack("<8f", 1, 2, 3, 4, 5, 6, 7, 8)
        array = cloud_payload_to_array({"point_count": 2}, payload)
        self.assertEqual(array.shape, (2, 4))
        np.testing.assert_allclose(array, [[1, 2, 3, 4], [5, 6, 7, 8]])

    def test_cloud_payload_length_mismatch(self):
        payload = struct.pack("<4f", 1, 2, 3, 4)
        with self.assertRaises(ValueError):
            cloud_payload_to_array({"point_count": 2}, payload)

    def test_global_cloud_returns_none_for_unsupported_pcd(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cloud.pcd"
            path.write_text(
                "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
                "TYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\n"
                "DATA ascii\n1 2 3\n"
            )
            self.assertIsNone(load_global_cloud_or_none(str(path)))

    def test_global_cloud_returns_none_for_missing_file(self):
        self.assertIsNone(load_global_cloud_or_none("/nonexistent/cloud.pcd"))


if __name__ == "__main__":
    unittest.main()
