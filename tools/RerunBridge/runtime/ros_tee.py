"""Subscribe to RoboKit ROS topics and tee frames to the Rerun sink.

Runs on Python 3.8 (rospy) only; it must never import rerun.
"""

import argparse
import socket
import time

import numpy as np

from frame_protocol import send_frame
from pointcloud import parse_cloud2, subsample


class RateGate:
    """Allow one event per 1/hz seconds; hz <= 0 disables throttling."""

    def __init__(self, hz: float):
        self.interval = 0.0 if hz <= 0 else 1.0 / float(hz)
        self.last = None

    def allow(self, now: float) -> bool:
        if self.interval <= 0:
            return True
        if self.last is None or now - self.last >= self.interval:
            self.last = now
            return True
        return False


class SinkLink:
    """Lazy, non-blocking TCP link to the sink; frames are dropped on failure."""

    def __init__(self, endpoint: str):
        host, _, port = endpoint.rpartition(":")
        if not host or not port.isdigit():
            raise ValueError(f"sink endpoint must be host:port, got {endpoint!r}")
        self.host = host
        self.port = int(port)
        self.sock = None

    def send(self, header: dict, payload: bytes = b"") -> bool:
        try:
            if self.sock is None:
                self.sock = socket.create_connection((self.host, self.port), timeout=0.2)
                self.sock.settimeout(0.2)
            send_frame(self.sock, header, payload)
            return True
        except (OSError, ValueError):
            self.close()
            return False

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None


def compressed_image_frame(msg):
    header = _base_header(msg, "color")
    header["media_type"] = "image/jpeg"
    return header, bytes(msg.data)


def depth_image_frame(msg):
    encoding = getattr(msg, "encoding", "")
    if encoding != "16UC1":
        raise ValueError(f"unsupported depth encoding {encoding!r}")
    width = int(msg.width)
    height = int(msg.height)
    payload = bytes(msg.data)
    if len(payload) != width * height * 2:
        raise ValueError(
            f"depth payload is {len(payload)} bytes, expected {width * height * 2}"
        )
    header = _base_header(msg, "depth")
    header.update(width=width, height=height)
    return header, payload


def cloud_frame(msg, max_points: int):
    points = subsample(parse_cloud2(msg), max_points)
    header = _base_header(msg, "cloud")
    header["point_count"] = int(len(points))
    payload = np.ascontiguousarray(points, dtype="<f4").tobytes()
    return header, payload


def _base_header(msg, kind: str) -> dict:
    header = getattr(msg, "header", None)
    return {
        "kind": kind,
        "stamp": _stamp_parts(getattr(header, "stamp", None)),
        "frame_id": getattr(header, "frame_id", "") or "",
    }


def _pose_parts(pose):
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    return (
        [
            float(getattr(position, "x", 0.0)),
            float(getattr(position, "y", 0.0)),
            float(getattr(position, "z", 0.0)),
        ],
        [
            float(getattr(orientation, "x", 0.0)),
            float(getattr(orientation, "y", 0.0)),
            float(getattr(orientation, "z", 0.0)),
            float(getattr(orientation, "w", 1.0)),
        ],
    )


def odom_frame(msg):
    pose = getattr(getattr(msg, "pose", None), "pose", None)
    position, orientation = _pose_parts(pose)
    header = _base_header(msg, "odom")
    header["position"] = position
    header["orientation"] = orientation
    return header, b""


def marker_frame(marker):
    position, orientation = _pose_parts(getattr(marker, "pose", None))
    scale = getattr(marker, "scale", None)
    color = getattr(marker, "color", None)
    raw_points = getattr(marker, "points", ()) or ()
    points = np.array(
        [[float(p.x), float(p.y), float(p.z)] for p in raw_points], dtype="<f4"
    )
    if points.size == 0:
        points = np.empty((0, 3), dtype="<f4")
    raw_colors = getattr(marker, "colors", ()) or ()
    colors = np.array(
        [[float(c.r), float(c.g), float(c.b), float(c.a)] for c in raw_colors],
        dtype="<f4",
    )
    if colors.size == 0:
        colors = np.empty((0, 4), dtype="<f4")

    header = _base_header(marker, "marker")
    header.update(
        {
            "ns": getattr(marker, "ns", "") or "",
            "id": int(getattr(marker, "id", 0)),
            "type": int(getattr(marker, "type", 0)),
            "action": int(getattr(marker, "action", 0)),
            "position": position,
            "orientation": orientation,
            "scale": [
                float(getattr(scale, "x", 0.0)),
                float(getattr(scale, "y", 0.0)),
                float(getattr(scale, "z", 0.0)),
            ],
            "color": [
                float(getattr(color, "r", 0.0)),
                float(getattr(color, "g", 0.0)),
                float(getattr(color, "b", 0.0)),
                float(getattr(color, "a", 0.0)),
            ],
            "text": getattr(marker, "text", "") or "",
            "point_count": int(len(points)),
            "color_count": int(len(colors)),
        }
    )
    payload = np.ascontiguousarray(points, dtype="<f4").tobytes()
    if len(colors):
        payload += np.ascontiguousarray(colors, dtype="<f4").tobytes()
    return header, payload


def marker_frames(msg):
    markers = getattr(msg, "markers", None)
    if markers is None:
        return [marker_frame(msg)]
    return [marker_frame(marker) for marker in markers]


def _stamp_parts(stamp) -> list:
    try:
        seconds = float(stamp.to_sec())
    except (AttributeError, TypeError, ValueError):
        return [0, 0]
    whole = int(seconds)
    nanos = int(round((seconds - whole) * 1e9))
    if nanos >= 1000000000:
        whole += 1
        nanos -= 1000000000
    return [whole, nanos]


def split_topic_spec(spec: str):
    """Split ``topic@entity`` into the ROS topic and the optional entity base."""
    topic, separator, base = spec.partition("@")
    return topic, base if separator else ""


def path_frame(msg):
    points = np.array(
        [
            [p.pose.position.x, p.pose.position.y, p.pose.position.z]
            for p in getattr(msg, "poses", ())
        ],
        dtype="<f4",
    )
    if points.size == 0:
        points = np.empty((0, 3), dtype="<f4")
    header = _base_header(msg, "path")
    header["point_count"] = len(points)
    return header, points.tobytes()


def subscription_plan(args):
    """Return (kind, topic, message type, builder, hz, multi, base) per topic."""
    plan = []
    if args.image_topic:
        plan.append(
            (
                "color",
                args.image_topic,
                "CompressedImage",
                compressed_image_frame,
                args.image_hz,
                False,
                "",
            )
        )
    if args.depth_topic:
        plan.append(
            (
                "depth",
                args.depth_topic,
                "Image",
                depth_image_frame,
                args.depth_hz,
                False,
                "",
            )
        )
    for spec in args.cloud_topic:
        topic, base = split_topic_spec(spec)
        plan.append(
            (
                "cloud",
                topic,
                "PointCloud2",
                lambda msg: cloud_frame(msg, args.max_points),
                args.cloud_hz,
                False,
                base,
            )
        )
    if args.odom_topic:
        plan.append(
            (
                "odom",
                args.odom_topic,
                "Odometry",
                odom_frame,
                args.odom_hz,
                False,
                "",
            )
        )
    for spec in args.marker_topic:
        topic, base = split_topic_spec(spec)
        plan.append(
            ("marker", topic, "Marker", marker_frame, args.marker_hz, False, base)
        )
    for spec in args.marker_array_topic:
        topic, base = split_topic_spec(spec)
        plan.append(
            (
                "marker",
                topic,
                "MarkerArray",
                marker_frames,
                args.marker_hz,
                True,
                base,
            )
        )
    for spec in args.path_topic:
        topic, base = split_topic_spec(spec)
        plan.append(
            ("path", topic, "Path", path_frame, args.path_hz, False, base)
        )
    return plan


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sink", default="127.0.0.1:19084")
    parser.add_argument(
        "--image-topic", default="/gsplat_render/camera0/color/image/compressed"
    )
    parser.add_argument("--depth-topic", default="")
    parser.add_argument("--cloud-topic", action="append", default=None)
    parser.add_argument("--odom-topic", default="")
    parser.add_argument("--marker-topic", action="append", default=[])
    parser.add_argument("--marker-array-topic", action="append", default=[])
    parser.add_argument("--path-topic", action="append", default=[])
    parser.add_argument("--image-hz", type=float, default=15.0)
    parser.add_argument("--depth-hz", type=float, default=5.0)
    parser.add_argument("--cloud-hz", type=float, default=5.0)
    parser.add_argument("--odom-hz", type=float, default=10.0)
    parser.add_argument("--marker-hz", type=float, default=5.0)
    parser.add_argument("--path-hz", type=float, default=5.0)
    parser.add_argument("--max-points", type=int, default=20000)
    args = parser.parse_args()
    if args.cloud_topic is None:
        args.cloud_topic = ["/super/sim/cloud_registered"]

    import rospy
    from nav_msgs.msg import Odometry, Path
    from sensor_msgs.msg import CompressedImage, Image, PointCloud2
    from visualization_msgs.msg import Marker, MarkerArray

    rospy.init_node("robokit_rerun_tee", anonymous=True)
    link = SinkLink(args.sink)
    stats = {
        "color": 0,
        "depth": 0,
        "cloud": 0,
        "odom": 0,
        "marker": 0,
        "path": 0,
        "dropped": 0,
    }
    reported = [0]

    def report() -> None:
        total = sum(stats.values()) - stats["dropped"]
        if total - reported[0] < 100:
            return
        reported[0] = total
        print(
            "tee: color=%d depth=%d cloud=%d odom=%d marker=%d path=%d dropped=%d"
            % (
                stats["color"],
                stats["depth"],
                stats["cloud"],
                stats["odom"],
                stats["marker"],
                stats["path"],
                stats["dropped"],
            ),
            flush=True,
        )

    def make_callback(kind, gate, builder, base=""):
        def callback(msg):
            if not gate.allow(time.monotonic()):
                return
            try:
                header, payload = builder(msg)
                if base:
                    header["base"] = base
            except (KeyError, ValueError) as error:
                stats["dropped"] += 1
                if stats["dropped"] % 100 == 1:
                    print(f"tee: dropping {kind} frame: {error}", flush=True)
                return
            if link.send(header, payload):
                stats[kind] += 1
            else:
                stats["dropped"] += 1
            report()

        return callback

    def make_multi_callback(kind, gate, builder, base=""):
        def callback(msg):
            if not gate.allow(time.monotonic()):
                return
            try:
                frames = builder(msg)
            except (KeyError, ValueError) as error:
                stats["dropped"] += 1
                if stats["dropped"] % 100 == 1:
                    print(f"tee: dropping {kind} frame: {error}", flush=True)
                return
            for header, payload in frames:
                if base:
                    header["base"] = base
                if kind == "marker":
                    header["marker_array"] = True
                if link.send(header, payload):
                    stats[kind] += 1
                else:
                    stats["dropped"] += 1
            report()

        return callback

    message_types = {
        "CompressedImage": CompressedImage,
        "Image": Image,
        "PointCloud2": PointCloud2,
        "Odometry": Odometry,
        "Marker": Marker,
        "MarkerArray": MarkerArray,
        "Path": Path,
    }
    subscriptions = []
    for kind, topic, msg_type, builder, hz, multi, base in subscription_plan(args):
        callback = (
            make_multi_callback(kind, RateGate(hz), builder, base)
            if multi
            else make_callback(kind, RateGate(hz), builder, base)
        )
        subscriptions.append(
            rospy.Subscriber(topic, message_types[msg_type], callback, queue_size=1)
        )
    if not subscriptions:
        print("tee: no topics configured", flush=True)
    else:
        print(
            "tee: sink=%s image=%r depth=%r clouds=%r odom=%r markers=%r "
            "marker_arrays=%r paths=%r"
            % (
                args.sink,
                args.image_topic,
                args.depth_topic,
                args.cloud_topic,
                args.odom_topic,
                args.marker_topic,
                args.marker_array_topic,
                args.path_topic,
            ),
            flush=True,
        )
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
