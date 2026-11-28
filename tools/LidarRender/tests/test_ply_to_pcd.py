"""Tests for the PLY -> binary PCD converter used by LidarRender."""

import tempfile
import unittest
from pathlib import Path

import numpy as np

from pcd_reader import load_pcd_xyz_i
from ply_to_pcd import PlyFormatError, convert_ply_to_pcd


def write_ascii_ply(path, rows, fields=("x", "y", "z", "intensity"), extra_lines=()):
    header = ["ply", "format ascii 1.0", "element vertex %d" % len(rows)]
    header.extend("property float %s" % name for name in fields)
    header.extend(extra_lines)
    header.append("end_header")
    body = [" ".join(str(value) for value in row) for row in rows]
    path.write_text("\n".join(header + body) + "\n", encoding="ascii")


def write_binary_ply(path, rows, fields=("x", "y", "z", "intensity")):
    header = ["ply", "format binary_little_endian 1.0", "element vertex %d" % len(rows)]
    header.extend("property float %s" % name for name in fields)
    header.append("end_header")
    data = np.asarray(rows, dtype="<f4").reshape(len(rows), len(fields)).tobytes()
    path.write_bytes(("\n".join(header) + "\n").encode("ascii") + data)


class PlyToPcdTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.root = Path(self.temp_dir.name)

    def test_ascii_ply_with_intensity(self):
        source = self.root / "ascii.ply"
        output = self.root / "ascii.pcd"
        write_ascii_ply(source, [(1, 2, 3, 0.5), (4, 5, 6, 0.75)])

        count = convert_ply_to_pcd(source, output)

        self.assertEqual(count, 2)
        points = load_pcd_xyz_i(str(output))
        np.testing.assert_allclose(
            points,
            np.array([[1, 2, 3, 0.5], [4, 5, 6, 0.75]], dtype="<f4"),
        )

    def test_binary_ply_with_intensity(self):
        source = self.root / "binary.ply"
        output = self.root / "binary.pcd"
        write_binary_ply(source, [(10, 20, 30, 1.5), (40, 50, 60, 2.5)])

        count = convert_ply_to_pcd(source, output)

        self.assertEqual(count, 2)
        points = load_pcd_xyz_i(str(output))
        np.testing.assert_allclose(
            points,
            np.array([[10, 20, 30, 1.5], [40, 50, 60, 2.5]], dtype="<f4"),
        )

    def test_missing_intensity_fills_zero(self):
        source = self.root / "no_intensity.ply"
        output = self.root / "no_intensity.pcd"
        write_ascii_ply(source, [(1, 2, 3), (4, 5, 6)], fields=("x", "y", "z"))

        convert_ply_to_pcd(source, output)

        points = load_pcd_xyz_i(str(output))
        np.testing.assert_allclose(
            points,
            np.array([[1, 2, 3, 0], [4, 5, 6, 0]], dtype="<f4"),
        )

    def test_extra_vertex_properties_are_ignored(self):
        source = self.root / "extra.ply"
        output = self.root / "extra.pcd"
        write_ascii_ply(
            source,
            [(1, 2, 3, 4, 9)],
            fields=("x", "y", "z", "intensity", "red"),
        )

        convert_ply_to_pcd(source, output)

        points = load_pcd_xyz_i(str(output))
        np.testing.assert_allclose(points, np.array([[1, 2, 3, 4]], dtype="<f4"))

    def test_missing_coordinate_raises(self):
        source = self.root / "missing_z.ply"
        output = self.root / "missing_z.pcd"
        write_ascii_ply(source, [(1, 2)], fields=("x", "y"))

        with self.assertRaises(PlyFormatError):
            convert_ply_to_pcd(source, output)

    def test_non_ply_input_raises(self):
        source = self.root / "not_ply.txt"
        output = self.root / "not_ply.pcd"
        source.write_text("VERSION 0.7\n", encoding="ascii")

        with self.assertRaises(PlyFormatError):
            convert_ply_to_pcd(source, output)


if __name__ == "__main__":
    unittest.main()
