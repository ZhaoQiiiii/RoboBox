"""Log RoboKit frames into Rerun (runs on Python >= 3.10 in /opt/rerun-venv)."""

import argparse
from collections import deque
import os
import socket
import sys
import threading
import time
from urllib.parse import urlparse

import numpy as np

from frame_protocol import recv_frame
from pointcloud import cloud_colors

DEFAULT_ENTITY_PREFIX = "/world/robokit"
COLOR_SUFFIX = "perception/camera/color"
DEPTH_SUFFIX = "perception/camera/depth"
CLOUD_SUFFIX = "perception/lidar/cloud"
ODOM_POSITION_SUFFIX = "odom/pos"
ODOM_TRAIL_SUFFIX = "odom/trail"
ROBOT_SUFFIX = "state/robot"
PATH_SUFFIX = "planning/path"
MARKER_SUFFIX = "markers"
ODOM_COLOR = np.array([0, 200, 0, 255], dtype=np.uint8)


def entity_path(prefix: str, suffix: str) -> str:
    return f"{prefix.rstrip('/')}/{suffix}"


def world_frame_mismatch(header: dict) -> str | None:
    """Return a frame name when a 3D frame is not already world."""
    frame = (header.get("frame_id") or "").strip()
    if frame in ("", "world", "/world"):
        return None
    return frame


def build_archetype(
    header: dict, payload: bytes, entity_prefix: str = DEFAULT_ENTITY_PREFIX
):
    """Return (entity_path, rerun archetype) for one tee frame."""
    import rerun as rr

    kind = header.get("kind")
    base = header.get("base")
    if kind == "color":
        return base or entity_path(entity_prefix, COLOR_SUFFIX), rr.EncodedImage(
            contents=payload, media_type=header.get("media_type", "image/jpeg")
        )
    if kind == "depth":
        width = int(header["width"])
        height = int(header["height"])
        values = np.frombuffer(payload, dtype="<u2")
        if values.size != width * height:
            raise ValueError(
                f"depth payload has {values.size} pixels, expected {width * height}"
            )
        return base or entity_path(entity_prefix, DEPTH_SUFFIX), rr.DepthImage(
            values.reshape(height, width), meter=1000.0
        )
    if kind == "cloud":
        values = np.frombuffer(payload, dtype="<f4")
        if values.size % 4:
            raise ValueError(f"cloud payload has {values.size} floats, not a multiple of 4")
        points = values.reshape(-1, 4)
        positions = np.ascontiguousarray(points[:, :3], dtype=np.float32)
        colors = cloud_colors(points[:, 3])
        return base or entity_path(entity_prefix, CLOUD_SUFFIX), rr.Points3D(
            positions, colors=colors
        )
    if kind == "path":
        count = int(header.get("point_count", 0))
        values = np.frombuffer(payload, dtype="<f4")
        if count and values.size < count * 3:
            raise ValueError(
                f"path payload has {values.size} floats, expected {count * 3}"
            )
        points = values[: count * 3].reshape(count, 3) if count else np.empty((0, 3))
        return base or entity_path(entity_prefix, PATH_SUFFIX), rr.LineStrips3D(
            [points], colors=[[0, 255, 0]]
        )
    raise ValueError(f"unknown frame kind: {kind!r}")


class OdomTrail:
    """Accumulate odometry positions and return the current body marker and trail."""

    def __init__(
        self,
        max_points: int = 10000,
        entity_prefix: str = DEFAULT_ENTITY_PREFIX,
        robot_body: bool = False,
    ):
        self._points = deque(maxlen=max_points)
        self.entity_prefix = entity_prefix
        self.robot_body = robot_body

    def update(self, header: dict):
        import rerun as rr

        position = header.get("position")
        if not position or len(position) != 3:
            raise ValueError("odom frame needs a 3D position")
        self._points.append([float(value) for value in position])
        points = np.asarray(self._points, dtype="<f4")
        entries = [
            (
                entity_path(self.entity_prefix, ODOM_POSITION_SUFFIX),
                rr.Points3D(
                    [points[-1]], radii=[0.15], colors=[ODOM_COLOR]
                ),
            ),
            (
                entity_path(self.entity_prefix, ODOM_TRAIL_SUFFIX),
                rr.LineStrips3D([points], colors=[ODOM_COLOR]),
            ),
        ]
        if self.robot_body:
            robot = entity_path(self.entity_prefix, ROBOT_SUFFIX)
            orientation = header.get("orientation") or [0.0, 0.0, 0.0, 1.0]
            entries.extend(
                [
                    (
                        robot,
                        rr.Transform3D(
                            translation=points[-1], quaternion=orientation
                        ),
                    ),
                    (
                        f"{robot}/body",
                        rr.Boxes3D(
                            half_sizes=[[0.25, 0.25, 0.1]],
                            colors=[[255, 100, 0]],
                        ),
                    ),
                    (
                        f"{robot}/heading",
                        rr.Arrows3D(
                            origins=[[0.0, 0.0, 0.0]],
                            vectors=[[0.5, 0.0, 0.0]],
                            colors=[[255, 100, 0]],
                        ),
                    ),
                ]
            )
        return entries


def _rgba(header: dict) -> np.ndarray:
    color = header.get("color") or [0.0, 0.0, 0.0, 0.0]
    values = np.clip(np.asarray(color, dtype=np.float32) * 255.0, 0.0, 255.0)
    return values.astype(np.uint8)


def _marker_entity(
    header: dict,
    ns_only: bool = False,
    entity_prefix: str = DEFAULT_ENTITY_PREFIX,
    entity_base: str = "",
    marker_array: bool = False,
) -> str:
    ns = str(header.get("ns") or "default").replace("/", "_") or "default"
    if entity_base:
        if ns_only:
            return entity_base
        if not marker_array:
            return entity_base
        return f"{entity_base}/{ns}/{int(header.get('id', 0))}"
    root = entity_path(entity_prefix, MARKER_SUFFIX)
    if ns_only:
        return f"{root}/{ns}"
    return f"{root}/{ns}/{int(header.get('id', 0))}"


def _marker_points(header: dict, payload: bytes) -> np.ndarray:
    count = int(header.get("point_count", 0))
    if count <= 0:
        return np.empty((0, 3), dtype="<f4")
    values = np.frombuffer(payload, dtype="<f4")
    if values.size < count * 3:
        raise ValueError("marker payload is shorter than point_count")
    return values[: count * 3].reshape(count, 3)


def _marker_colors(header: dict, payload: bytes):
    count = int(header.get("color_count", 0))
    if count <= 0:
        return None
    start = int(header.get("point_count", 0)) * 3
    values = np.frombuffer(payload, dtype="<f4")
    if values.size < start + count * 4:
        raise ValueError("marker payload is shorter than color_count")
    colors = values[start : start + count * 4].reshape(count, 4)
    return np.clip(colors * 255.0, 0.0, 255.0).astype(np.uint8)


def _marker_colors_for(header: dict, payload: bytes, points: np.ndarray, base: np.ndarray):
    colors = _marker_colors(header, payload)
    if colors is not None:
        return colors
    return np.tile(base, (len(points), 1))


def marker_archetype(
    header: dict, payload: bytes, entity_prefix: str = DEFAULT_ENTITY_PREFIX
):
    """Return (entity_path, rerun archetype) for one marker frame."""
    import rerun as rr

    entity_base = header.get("base") or ""
    marker_array = bool(header.get("marker_array"))
    action = int(header.get("action", 0))
    if action == 2:
        return _marker_entity(
            header,
            entity_prefix=entity_prefix,
            entity_base=entity_base,
            marker_array=marker_array,
        ), rr.Clear(recursive=True)
    if action == 3:
        return _marker_entity(
            header,
            ns_only=True,
            entity_prefix=entity_prefix,
            entity_base=entity_base,
            marker_array=marker_array,
        ), rr.Clear(recursive=True)

    marker_type = int(header.get("type", 0))
    points = _marker_points(header, payload)
    colors = _marker_colors(header, payload)
    base = _rgba(header)
    scale = np.asarray(header.get("scale") or [0.0, 0.0, 0.0], dtype=np.float32)
    entity = _marker_entity(
        header,
        entity_prefix=entity_prefix,
        entity_base=entity_base,
        marker_array=marker_array,
    )

    if marker_type == 0:  # ARROW
        if len(points) < 2:
            raise ValueError("ARROW marker needs two points")
        return entity, rr.Arrows3D(
            origins=[points[0]],
            vectors=[points[1] - points[0]],
            colors=[base],
            radii=[max(float(scale[0]) / 2.0, 0.01)],
        )

    if marker_type == 4:  # LINE_STRIP
        if not len(points):
            raise ValueError("LINE_STRIP marker has no points")
        if colors is not None and len(colors) == len(points) and len(points) >= 2:
            return entity, rr.LineStrips3D(
                np.stack([points[:-1], points[1:]], axis=1), colors=colors[:-1]
            )
        return entity, rr.LineStrips3D([points], colors=[base])
    if marker_type == 5:  # LINE_LIST
        if len(points) < 2:
            raise ValueError("LINE_LIST marker needs at least two points")
        usable = len(points) - len(points) % 2
        pairs = points[:usable].reshape(-1, 2, 3)
        if colors is not None and len(colors) == len(points):
            return entity, rr.LineStrips3D(pairs, colors=colors[:usable:2])
        return entity, rr.LineStrips3D(pairs, colors=[base])
    if marker_type == 8:  # POINTS
        return entity, rr.Points3D(
            points, colors=_marker_colors_for(header, payload, points, base)
        )
    if marker_type == 7:  # SPHERE_LIST
        return entity, rr.Points3D(
            points,
            radii=[max(float(scale[0]) / 2.0, 0.01)],
            colors=_marker_colors_for(header, payload, points, base),
        )
    if marker_type == 6:  # CUBE_LIST
        return entity, rr.Boxes3D(
            centers=points,
            half_sizes=np.tile(scale / 2.0, (len(points), 1)),
            colors=_marker_colors_for(header, payload, points, base),
        )
    if marker_type == 1:  # CUBE
        pose = np.asarray(header.get("position") or [0.0, 0.0, 0.0], dtype=np.float32)
        quat = np.asarray(
            header.get("orientation") or [0.0, 0.0, 0.0, 1.0], dtype=np.float32
        )
        return entity, rr.Boxes3D(
            centers=[pose],
            half_sizes=[scale / 2.0],
            quaternions=[quat],
            colors=[base],
        )
    if marker_type == 2:  # SPHERE
        pose = np.asarray(header.get("position") or [0.0, 0.0, 0.0], dtype=np.float32)
        return entity, rr.Points3D(
            [pose], radii=[max(float(scale[0]) / 2.0, 0.01)], colors=[base]
        )
    raise ValueError(f"unsupported marker type: {marker_type}")


def retry_connect(connect, delay: float = 2.0, sleep=time.sleep):
    """Call connect until it succeeds; never gives up."""
    while True:
        try:
            return connect()
        except Exception as error:  # noqa: BLE001 - any failure means try again
            print(
                f"rerun sink: connect failed ({error}); retrying in {delay:.0f}s",
                file=sys.stderr,
                flush=True,
            )
            sleep(delay)


def probe_server(url, timeout: float = 0.5) -> None:
    """Open a TCP connection to the Rerun server; raise OSError when it is absent.

    rerun-sdk's connect_grpc() buffers silently instead of raising, so the
    retry loop needs this check to know the server is really there.
    """
    parsed = urlparse((url or "").replace("rerun+", "", 1))
    if not parsed.hostname or not parsed.port:
        raise ValueError(f"cannot parse rerun server url: {url!r}")
    with socket.create_connection((parsed.hostname, parsed.port), timeout=timeout):
        pass


class ServerHealth:
    """Track whether the Rerun server is reachable so frames are not buffered blindly."""

    def __init__(self, url, interval: float = 2.0, probe=probe_server):
        self.url = url
        self.interval = interval
        self.probe = probe
        self.ok = threading.Event()
        self.ok.set()

    def check(self) -> bool:
        try:
            self.probe(self.url)
        except (OSError, ValueError):
            self.ok.clear()
        else:
            self.ok.set()
        return self.ok.is_set()

    def start(self) -> None:
        def watch() -> None:
            was_up = True
            while True:
                up = self.check()
                if up != was_up:
                    print(
                        "rerun sink: server "
                        + ("reachable again" if up else "unreachable; pausing logging"),
                        flush=True,
                    )
                    was_up = up
                time.sleep(self.interval)

        threading.Thread(target=watch, name="rerun-server-health", daemon=True).start()


def build_blueprint(entity_prefix: str = DEFAULT_ENTITY_PREFIX):
    """A 3D world view with the Gsplat camera beside it."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="/world", name="World"),
            rrb.Spatial2DView(
                origin=entity_path(entity_prefix, COLOR_SUFFIX), name="Gsplat"
            ),
            column_shares=[3, 1],
            name="RoboKit",
        )
    )


def serve_stream(
    url,
    serve_port,
    app_id,
    recording_id=None,
    entity_prefix=DEFAULT_ENTITY_PREFIX,
):
    """Create the recording and either serve it or connect to an existing server."""
    import rerun as rr

    rr.init(app_id, spawn=False, recording_id=recording_id)
    if serve_port:
        rr.serve_grpc(grpc_port=int(serve_port))
    else:
        probe_server(url)
        rr.connect_grpc(url)
    recording = rr.get_global_data_recording()
    if recording is None:
        raise RuntimeError("rerun recording was not initialized")
    # Keep a layout saved in the viewer active; the default layout still
    # applies to a viewer that has no saved layout yet.
    recording.send_blueprint(
        build_blueprint(entity_prefix), make_active=False
    )
    return recording


def pump_connection(
    connection,
    log,
    on_frame=None,
    health=None,
    trail=None,
    entity_prefix=DEFAULT_ENTITY_PREFIX,
) -> int:
    """Read frames from one tee connection until it closes; never raises on RST."""
    logged = 0
    warned = set()
    warned_frames = set()
    while True:
        try:
            header, payload = recv_frame(connection)
        except (ValueError, OSError) as error:
            if str(error) != "connection closed before frame completed":
                print(f"rerun sink: dropping connection: {error}", flush=True)
            return logged
        if health is not None and not health.ok.is_set():
            continue
        if header.get("kind") in ("cloud", "odom", "marker"):
            frame = world_frame_mismatch(header)
            if frame is not None and frame not in warned_frames:
                warned_frames.add(frame)
                print(f"rerun sink: frame {frame!r} treated as world", flush=True)
        try:
            kind = header.get("kind")
            if kind == "odom":
                if trail is None:
                    raise ValueError("odom frame without a trail")
                entries = trail.update(header)
            elif kind == "marker":
                entries = [
                    marker_archetype(header, payload, entity_prefix=entity_prefix)
                ]
            else:
                entries = [
                    build_archetype(header, payload, entity_prefix=entity_prefix)
                ]
        except (KeyError, ValueError) as error:
            message = str(error)
            if message not in warned:
                warned.add(message)
                print(f"rerun sink: skipped frame: {error}", flush=True)
            continue
        for entity, archetype in entries:
            log(entity, archetype)
            logged += 1
            if logged % 100 == 0:
                print(f"rerun sink: logged {logged} frames", flush=True)
            if on_frame is not None:
                on_frame(entity)


def run_sink(
    listen_host,
    listen_port,
    url,
    serve_port,
    app_id,
    recording_id=None,
    on_frame=None,
    entity_prefix=DEFAULT_ENTITY_PREFIX,
    robot_body=False,
) -> None:
    """Serve one TCP connection at a time and log every frame it receives."""
    recording = retry_connect(
        lambda: serve_stream(
            url, serve_port, app_id, recording_id, entity_prefix
        )
    )
    trail = OdomTrail(entity_prefix=entity_prefix, robot_body=robot_body)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((listen_host, int(listen_port)))
    server.listen(1)
    print(f"rerun sink: listening on {listen_host}:{listen_port}", flush=True)

    health = None
    if serve_port is None:
        health = ServerHealth(url)
        health.start()

    while True:
        connection, _ = server.accept()
        with connection:
            pump_connection(
                connection,
                recording.log,
                on_frame,
                health,
                trail,
                entity_prefix,
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen", default="127.0.0.1:19084", help="tee endpoint")
    parser.add_argument(
        "--url", default="rerun+http://127.0.0.1:9877/proxy", help="Rerun gRPC server"
    )
    parser.add_argument("--serve", action="store_true", help="start our own gRPC server")
    parser.add_argument("--serve-port", type=int, default=9880)
    parser.add_argument("--app", default="ZeroAgent", help="Rerun application id")
    parser.add_argument(
        "--entity-prefix",
        default=os.environ.get(
            "ROBOKIT_RERUN_ENTITY_PREFIX", DEFAULT_ENTITY_PREFIX
        ),
        help="Rerun entity prefix, e.g. /world/drone",
    )
    parser.add_argument(
        "--recording-id",
        default=os.environ.get("ROBOKIT_RERUN_RECORDING_ID") or None,
        help="Rerun recording id shared with other producers",
    )
    parser.add_argument(
        "--robot-body",
        action="store_true",
        default=os.environ.get("ROBOKIT_RERUN_ROBOT_BODY") == "1",
        help="draw a proxy body from the odom pose",
    )
    args = parser.parse_args()

    host, _, port = args.listen.rpartition(":")
    if not host or not port.isdigit():
        parser.error(f"--listen needs host:port, got {args.listen!r}")
    serve_port = args.serve_port if args.serve else None
    run_sink(
        listen_host=host,
        listen_port=int(port),
        url=None if args.serve else args.url,
        serve_port=serve_port,
        app_id=args.app,
        recording_id=args.recording_id,
        entity_prefix=args.entity_prefix,
        robot_body=args.robot_body,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
