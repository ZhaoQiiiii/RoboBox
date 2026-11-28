"""Smoke test: run the sink in serve mode and push one frame of each kind."""

import socket
import threading
import time
import types

import numpy as np

from frame_protocol import send_frame
from rerun_sink import run_sink


def free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def main() -> int:
    received = []
    listen_port = free_port()
    serve_port = free_port()
    thread = threading.Thread(
        target=run_sink,
        kwargs=types.SimpleNamespace(
            listen_host="127.0.0.1",
            listen_port=listen_port,
            url=None,
            serve_port=serve_port,
            app_id="robokit-smoke",
            on_frame=received.append,
        ).__dict__,
        daemon=True,
    )
    thread.start()
    time.sleep(1.0)

    with socket.create_connection(("127.0.0.1", listen_port), timeout=5.0) as sock:
        send_frame(sock, {"kind": "color", "media_type": "image/jpeg"}, b"\xff\xd8x")
        send_frame(
            sock,
            {"kind": "depth", "width": 2, "height": 2},
            np.array([[1, 2], [3, 4]], dtype="<u2").tobytes(),
        )
        send_frame(
            sock,
            {"kind": "cloud", "point_count": 2},
            np.zeros((2, 4), dtype="<f4").tobytes(),
        )
        send_frame(
            sock,
            {
                "kind": "odom",
                "position": [0.0, 0.0, 0.0],
                "orientation": [0.0, 0.0, 0.0, 1.0],
            },
        )
        send_frame(
            sock,
            {
                "kind": "marker",
                "ns": "fov",
                "id": 1,
                "type": 4,
                "action": 0,
                "position": [0.0, 0.0, 0.0],
                "orientation": [0.0, 0.0, 0.0, 1.0],
                "scale": [1.0, 1.0, 1.0],
                "color": [0.0, 1.0, 0.0, 1.0],
                "text": "",
                "point_count": 2,
                "color_count": 0,
            },
            np.zeros((2, 3), dtype="<f4").tobytes(),
        )

    deadline = time.monotonic() + 5.0
    while len(received) < 6 and time.monotonic() < deadline:
        time.sleep(0.05)
    if len(received) != 6:
        print(f"smoke failed: received {len(received)} frames")
        return 1
    print("smoke ok: 6 entities")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
