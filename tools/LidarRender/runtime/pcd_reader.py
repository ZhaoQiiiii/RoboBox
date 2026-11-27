"""Minimal binary PCD reader for LidarRender global point clouds."""

from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np


class PcdFormatError(ValueError):
    """Raised when a PCD file is not a supported binary xyz/intensity cloud."""


def read_header(path: str) -> Tuple[Dict[str, str], bytes]:
    metadata: Dict[str, str] = {}
    with Path(path).open("rb") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise PcdFormatError("missing DATA line")
            try:
                text = line.decode("ascii").strip()
            except UnicodeDecodeError as error:
                raise PcdFormatError("PCD header is not ASCII") from error
            if not text or text.startswith("#"):
                continue
            key, _, value = text.partition(" ")
            metadata[key.upper()] = value.strip()
            if key.upper() == "DATA":
                break
        data = handle.read()
    return metadata, data


def parse_int_list(metadata: Dict[str, str], key: str) -> List[int]:
    value = metadata.get(key)
    if not value:
        raise PcdFormatError("missing %s field" % key)
    try:
        return [int(item) for item in value.split()]
    except ValueError as error:
        raise PcdFormatError("invalid %s field: %s" % (key, value)) from error


def load_pcd_xyz_i(path: str) -> np.ndarray:
    """Load a PCD file as an (N, 4) float32 array of x, y, z, intensity."""
    metadata, data = read_header(path)
    data_mode = metadata.get("DATA", "").lower()
    if data_mode != "binary":
        raise PcdFormatError("only DATA binary is supported, got %r" % (data_mode,))

    fields = metadata.get("FIELDS", "").split()
    if not fields:
        raise PcdFormatError("missing FIELDS field")
    sizes = parse_int_list(metadata, "SIZE")
    types = metadata.get("TYPE", "").split()
    counts = parse_int_list(metadata, "COUNT")
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise PcdFormatError("FIELDS/SIZE/TYPE/COUNT lengths differ")

    for name in ("x", "y", "z"):
        if name not in fields:
            raise PcdFormatError("missing %s field" % name)

    points = parse_int_list(metadata, "POINTS")
    if len(points) != 1:
        raise PcdFormatError("invalid POINTS field")
    point_count = points[0]
    point_step = sum(size * count for size, count in zip(sizes, counts))
    if point_count < 0 or point_step <= 0:
        raise PcdFormatError("invalid point count or step")
    if len(data) != point_count * point_step:
        raise PcdFormatError(
            "data length %d does not match %d points x %d bytes"
            % (len(data), point_count, point_step)
        )

    raw = np.frombuffer(data, dtype=np.uint8)
    offsets = []
    offset = 0
    for size, count in zip(sizes, counts):
        offsets.append(offset)
        offset += size * count

    result = np.empty((point_count, 4), dtype="<f4")
    for column, name in enumerate(("x", "y", "z", "intensity")):
        if name not in fields:
            result[:, column] = 0.0
            continue
        index = fields.index(name)
        if types[index] != "F" or sizes[index] != 4 or counts[index] != 1:
            raise PcdFormatError(
                "%s must be FLOAT32 with COUNT 1" % name
            )
        values = np.ndarray(
            shape=(point_count,),
            dtype="<f4",
            buffer=raw,
            offset=offsets[index],
            strides=(point_step,),
        )
        result[:, column] = values
    return result
