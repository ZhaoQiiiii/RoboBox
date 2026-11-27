"""Build sensor_msgs/PointCloud2 messages from (N, 4) xyz/intensity arrays."""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def cloud2_from_xyz_i(points, stamp, frame_id):
    """Return a PointCloud2 with FLOAT32 x, y, z, intensity fields."""
    points = np.ascontiguousarray(points, dtype="<f4")
    if points.ndim != 2 or points.shape[1] != 4:
        raise ValueError("points must have shape (N, 4)")

    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = 1
    message.width = int(points.shape[0])
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = 16 * message.width
    message.is_dense = bool(np.isfinite(points).all())
    message.data = points.tobytes()
    return message
