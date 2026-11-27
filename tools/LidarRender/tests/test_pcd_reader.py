import struct
import tempfile
import unittest
from pathlib import Path

import numpy as np

from pcd_reader import PcdFormatError, load_pcd_xyz_i


def write_pcd(path, fields, points, data="binary"):
    points = np.asarray(points, dtype="<f4")
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS %s\n"
        "SIZE %s\n"
        "TYPE %s\n"
        "COUNT %s\n"
        "WIDTH %d\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %d\n"
        "DATA %s\n"
        % (
            " ".join(fields),
            " ".join("4" for _ in fields),
            " ".join("F" for _ in fields),
            " ".join("1" for _ in fields),
            points.shape[0],
            points.shape[0],
            data,
        )
    )
    with Path(path).open("wb") as handle:
        handle.write(header.encode("ascii"))
        if data == "binary":
            points.tofile(handle)
        elif data == "ascii":
            np.savetxt(handle, points, fmt="%.6f")


class PcdReaderTest(unittest.TestCase):
    def test_reads_xyz_and_intensity(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cloud.pcd"
            points = np.array(
                [
                    [1.0, 2.0, 3.0, 10.0, 0.1],
                    [4.0, 5.0, 6.0, 20.0, 0.2],
                    [7.0, 8.0, 9.0, 30.0, 0.3],
                ],
                dtype="<f4",
            )
            write_pcd(path, ["x", "y", "z", "intensity", "normal_x"], points)
            cloud = load_pcd_xyz_i(str(path))
            self.assertEqual(cloud.shape, (3, 4))
            self.assertEqual(cloud.dtype, np.dtype("<f4"))
            np.testing.assert_allclose(cloud, points[:, :4])

    def test_missing_intensity_is_zero(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cloud.pcd"
            points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype="<f4")
            write_pcd(path, ["x", "y", "z"], points)
            cloud = load_pcd_xyz_i(str(path))
            self.assertEqual(cloud.shape, (2, 4))
            np.testing.assert_allclose(cloud[:, :3], points)
            np.testing.assert_allclose(cloud[:, 3], np.zeros(2, dtype="<f4"))

    def test_rejects_ascii(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cloud.pcd"
            write_pcd(path, ["x", "y", "z"], np.zeros((1, 3), dtype="<f4"), data="ascii")
            with self.assertRaises(PcdFormatError):
                load_pcd_xyz_i(str(path))

    def test_rejects_binary_compressed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cloud.pcd"
            write_pcd(path, ["x", "y", "z"], np.zeros((1, 3), dtype="<f4"),
                      data="binary_compressed")
            with self.assertRaises(PcdFormatError):
                load_pcd_xyz_i(str(path))

    def test_rejects_truncated_data(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cloud.pcd"
            header = (
                "# .PCD v0.7 - Point Cloud Data file format\n"
                "VERSION 0.7\n"
                "FIELDS x y z\n"
                "SIZE 4 4 4\n"
                "TYPE F F F\n"
                "COUNT 1 1 1\n"
                "WIDTH 3\n"
                "HEIGHT 1\n"
                "POINTS 3\n"
                "DATA binary\n"
            )
            path.write_bytes(header.encode("ascii") + struct.pack("<6f", 1, 2, 3, 4, 5, 6))
            with self.assertRaises(PcdFormatError):
                load_pcd_xyz_i(str(path))

    def test_rejects_missing_xyz(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cloud.pcd"
            write_pcd(path, ["x", "y"], np.zeros((1, 2), dtype="<f4"))
            with self.assertRaises(PcdFormatError):
                load_pcd_xyz_i(str(path))


if __name__ == "__main__":
    unittest.main()
