"""PointCloud2 parsing helpers shared by the ROS tee and the Rerun sink."""

import math

import numpy as np

_FLOAT32 = 7
_GRADIENT = np.array(
    [[0, 0, 255], [0, 255, 255], [255, 255, 0], [255, 0, 0]], dtype=np.float32
)


class CloudFormatError(ValueError):
    """The message is not the float32 x/y/z PointCloud2 this bridge supports."""


def parse_cloud2(msg) -> np.ndarray:
    """Return an (N, 4) float32 array of x, y, z, intensity from a PointCloud2."""
    if getattr(msg, "is_bigendian", False):
        raise CloudFormatError("big-endian PointCloud2 is not supported")

    point_step = int(getattr(msg, "point_step", 0))
    if point_step <= 0:
        raise CloudFormatError(f"invalid point_step: {point_step}")

    data = np.frombuffer(msg.data, dtype=np.uint8)
    if len(data) % point_step:
        raise CloudFormatError(
            f"PointCloud2 data length {len(data)} is not a multiple of point_step {point_step}"
        )
    count = len(data) // point_step
    rows = data.reshape(count, point_step) if count else data.reshape(0, point_step)

    fields = {field.name: field for field in getattr(msg, "fields", [])}
    offsets = {}
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None:
            raise CloudFormatError(f"PointCloud2 is missing the {name!r} field")
        _require_float32(field, point_step)
        offsets[name] = int(field.offset)

    intensity_field = fields.get("intensity")
    if intensity_field is not None:
        _require_float32(intensity_field, point_step)
        offsets["intensity"] = int(intensity_field.offset)

    points = np.zeros((count, 4), dtype=np.float32)
    for column, name in enumerate(("x", "y", "z", "intensity")):
        offset = offsets.get(name)
        if offset is None:
            continue
        points[:, column] = np.ndarray(
            shape=(count,),
            dtype="<f4",
            buffer=rows,
            offset=offset,
            strides=(point_step,),
        )
    return points


def subsample(points: np.ndarray, max_points: int) -> np.ndarray:
    """Keep every stride-th point so the result is at most max_points long."""
    if max_points <= 0 or len(points) <= max_points:
        return points
    stride = int(math.ceil(len(points) / max_points))
    return points[::stride]


def cloud_colors(intensity: np.ndarray) -> np.ndarray:
    """Map intensity to an (N, 3) uint8 blue-cyan-yellow-red gradient."""
    values = np.asarray(intensity, dtype=np.float32)
    if values.size == 0:
        return np.zeros((0, 3), dtype=np.uint8)

    low, high = np.percentile(values, [2.0, 98.0])
    if not np.isfinite(low) or not np.isfinite(high) or high - low < 1e-6:
        scaled = np.full(values.shape, 0.5, dtype=np.float32)
    else:
        scaled = np.clip((values - low) / (high - low), 0.0, 1.0)

    position = scaled * (len(_GRADIENT) - 1)
    index = np.minimum(position.astype(np.int32), len(_GRADIENT) - 2)
    fraction = (position - index)[:, None]
    rgb = _GRADIENT[index] * (1.0 - fraction) + _GRADIENT[index + 1] * fraction
    return np.rint(rgb).astype(np.uint8)


def _require_float32(field, point_step: int) -> None:
    if int(field.datatype) != _FLOAT32:
        raise CloudFormatError(
            f"field {field.name!r} has datatype {field.datatype}, expected FLOAT32"
        )
    if int(field.offset) + 4 > point_step:
        raise CloudFormatError(
            f"field {field.name!r} offset {field.offset} exceeds point_step {point_step}"
        )
