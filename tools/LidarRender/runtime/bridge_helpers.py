"""Pure helpers shared by the LidarRender ROS bridge and its tests."""

from typing import Any, Dict, Optional

import numpy as np

from pcd_reader import PcdFormatError, load_pcd_xyz_i


def pose_to_header(message, seq: int) -> Dict[str, Any]:
    """Convert a nav_msgs/Odometry message into a pose header."""
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return {
        "version": 1,
        "type": "pose",
        "seq": int(seq),
        "stamp_sec": int(message.header.stamp.secs),
        "stamp_nsec": int(message.header.stamp.nsecs),
        "position": [float(position.x), float(position.y), float(position.z)],
        "orientation": [float(orientation.x), float(orientation.y),
                        float(orientation.z), float(orientation.w)],
    }


def cloud_payload_to_array(header: Dict[str, Any], payload: bytes) -> np.ndarray:
    """Convert a cloud frame payload into an (N, 4) float32 array."""
    point_count = int(header["point_count"])
    values = np.frombuffer(payload, dtype="<f4")
    if values.size != point_count * 4:
        raise ValueError("payload has %d floats, expected %d" % (
            values.size, point_count * 4))
    return values.reshape(point_count, 4).copy()


def load_global_cloud_or_none(path: str) -> Optional[np.ndarray]:
    """Load the global PCD, returning None for unsupported or missing files."""
    try:
        return load_pcd_xyz_i(path)
    except (PcdFormatError, OSError):
        return None
