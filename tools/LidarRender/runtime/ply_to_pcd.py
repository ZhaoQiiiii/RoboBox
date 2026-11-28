"""Convert a point-cloud PLY file to the binary PCD format used by LidarRender."""

import argparse
import sys
from pathlib import Path

import numpy as np

PLY_TYPES = {
    "char": "i1",
    "int8": "i1",
    "uchar": "u1",
    "uint8": "u1",
    "short": "i2",
    "int16": "i2",
    "ushort": "u2",
    "uint16": "u2",
    "int": "i4",
    "int32": "i4",
    "uint": "u4",
    "uint32": "u4",
    "float": "f4",
    "float32": "f4",
    "double": "f8",
    "float64": "f8",
}

BYTE_ORDER = {
    "binary_little_endian": "<",
    "binary_big_endian": ">",
}


class PlyFormatError(ValueError):
    """Raised when a PLY file cannot be converted to a point cloud."""


def read_ply_header(path):
    """Return (format, vertex element, data offset) for a PLY file."""
    with Path(path).open("rb") as handle:
        if handle.readline().strip() != b"ply":
            raise PlyFormatError("not a PLY file")
        ply_format = None
        elements = []
        current = None
        while True:
            line = handle.readline()
            if not line:
                raise PlyFormatError("missing end_header")
            try:
                text = line.decode("ascii").strip()
            except UnicodeDecodeError as error:
                raise PlyFormatError("PLY header is not ASCII") from error
            if not text or text.startswith("comment") or text.startswith("obj_info"):
                continue
            parts = text.split()
            key = parts[0]
            if key == "format":
                if len(parts) < 2 or parts[1] not in ("ascii",) + tuple(BYTE_ORDER):
                    raise PlyFormatError("unsupported PLY format: %s" % text)
                ply_format = parts[1]
            elif key == "element":
                if len(parts) < 3:
                    raise PlyFormatError("invalid element line: %s" % text)
                current = {"name": parts[1], "count": int(parts[2]), "properties": []}
                elements.append(current)
            elif key == "property":
                if current is None:
                    raise PlyFormatError("property before element")
                if parts[1] == "list":
                    if len(parts) < 5:
                        raise PlyFormatError("invalid list property: %s" % text)
                    current["properties"].append(
                        {
                            "kind": "list",
                            "count_type": parts[2],
                            "item_type": parts[3],
                            "name": parts[4],
                        }
                    )
                else:
                    if len(parts) < 3:
                        raise PlyFormatError("invalid property: %s" % text)
                    current["properties"].append(
                        {"kind": "scalar", "type": parts[1], "name": parts[2]}
                    )
            elif key == "end_header":
                data_offset = handle.tell()
                break
        if ply_format is None:
            raise PlyFormatError("missing format line")
        if not elements or elements[0]["name"] != "vertex":
            raise PlyFormatError("vertex element must come first")
        return ply_format, elements[0], data_offset


def read_ascii_columns(path, count, property_count):
    with Path(path).open("r", encoding="ascii") as handle:
        for line in handle:
            if line.strip() == "end_header":
                break
        rows = []
        while len(rows) < count:
            line = handle.readline()
            if not line:
                raise PlyFormatError("PLY ended before %d vertices" % count)
            parts = line.split()
            if not parts:
                continue
            if len(parts) < property_count:
                raise PlyFormatError(
                    "vertex line has %d values, expected %d"
                    % (len(parts), property_count)
                )
            rows.append(parts[:property_count])
    try:
        return np.array(rows, dtype=np.float64)
    except ValueError as error:
        raise PlyFormatError("non-numeric vertex data") from error


def read_binary_columns(path, data_offset, count, properties, ply_format):
    byte_order = BYTE_ORDER[ply_format]
    dtype = np.dtype(
        [(prop["name"], byte_order + PLY_TYPES[prop["type"]]) for prop in properties]
    )
    with Path(path).open("rb") as handle:
        handle.seek(data_offset)
        data = np.fromfile(handle, dtype=dtype, count=count)
    if data.size != count:
        raise PlyFormatError("PLY ended before %d vertices" % count)
    return np.column_stack(
        [data[prop["name"]].astype(np.float64) for prop in properties]
    )


def load_vertex_columns(path):
    ply_format, vertex, data_offset = read_ply_header(path)
    properties = vertex["properties"]
    if any(prop["kind"] != "scalar" for prop in properties):
        raise PlyFormatError("list properties in the vertex element are not supported")
    names = [prop["name"] for prop in properties]
    for required in ("x", "y", "z"):
        if required not in names:
            raise PlyFormatError("missing %s property" % required)
    for prop in properties:
        if prop["type"] not in PLY_TYPES:
            raise PlyFormatError("unsupported property type: %s" % prop["type"])

    count = vertex["count"]
    if ply_format == "ascii":
        columns = read_ascii_columns(path, count, len(properties))
    else:
        columns = read_binary_columns(
            path, data_offset, count, properties, ply_format
        )
    return names, columns


def write_binary_pcd(path, points):
    count = points.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        "WIDTH {width}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS {width}\n"
        "DATA binary\n"
    ).format(width=count)
    with Path(path).open("wb") as handle:
        handle.write(header.encode("ascii"))
        np.asarray(points, dtype="<f4").tofile(handle)


def convert_ply_to_pcd(input_path, output_path):
    names, columns = load_vertex_columns(input_path)
    index = {name: position for position, name in enumerate(names)}
    points = np.empty((columns.shape[0], 4), dtype="<f4")
    points[:, 0] = columns[:, index["x"]]
    points[:, 1] = columns[:, index["y"]]
    points[:, 2] = columns[:, index["z"]]
    if "intensity" in index:
        points[:, 3] = columns[:, index["intensity"]]
    else:
        points[:, 3] = 0.0
    write_binary_pcd(output_path, points)
    return points.shape[0]


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Convert a point-cloud PLY file to binary PCD."
    )
    parser.add_argument("input", help="input PLY (ascii or binary)")
    parser.add_argument("output", help="output binary PCD")
    args = parser.parse_args(argv)
    try:
        count = convert_ply_to_pcd(args.input, args.output)
    except (PlyFormatError, OSError) as error:
        print("错误: %s" % error, file=sys.stderr)
        return 2
    print("PLY -> PCD: %s -> %s (%d points)" % (args.input, args.output, count))
    return 0


if __name__ == "__main__":
    sys.exit(main())
