#!/usr/bin/env python3
"""Generate a deterministic 10 m x 10 m ground plane PCD for LidarRender tests."""

from pathlib import Path

import numpy as np


def main() -> None:
    coordinates = np.linspace(-5.0, 5.0, 201, dtype=np.float32)
    x_grid, y_grid = np.meshgrid(coordinates, coordinates, indexing="xy")
    points = np.empty((x_grid.size, 4), dtype="<f4")
    points[:, 0] = x_grid.reshape(-1)
    points[:, 1] = y_grid.reshape(-1)
    points[:, 2] = 0.0
    points[:, 3] = 100.0

    output = Path(__file__).resolve().parent / "data" / "small_map.pcd"
    output.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {points.shape[0]}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {points.shape[0]}\n"
        "DATA binary\n"
    )
    with output.open("wb") as handle:
        handle.write(header.encode("ascii"))
        points.tofile(handle)
    print(f"wrote {output} ({points.shape[0]} points)")


if __name__ == "__main__":
    main()
