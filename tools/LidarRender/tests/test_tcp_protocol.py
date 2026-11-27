import socket
import struct
import unittest

from tcp_protocol import (
    MAX_PAYLOAD_BYTES,
    receive_message,
    send_message,
    validate_header,
)


def pose_header(**overrides):
    header = {
        "version": 1,
        "type": "pose",
        "seq": 1,
        "stamp_sec": 1,
        "stamp_nsec": 2,
        "position": [0.0, 0.0, 1.5],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }
    header.update(overrides)
    return header


def cloud_header(**overrides):
    header = {
        "version": 1,
        "type": "cloud",
        "frame_seq": 1,
        "pose_seq": 1,
        "stamp_sec": 0,
        "stamp_nsec": 0,
        "frame_id": "world",
        "point_count": 2,
        "point_bytes": 32,
    }
    header.update(overrides)
    return header


class TcpProtocolTest(unittest.TestCase):
    def test_pose_roundtrip(self):
        left, right = socket.socketpair()
        self.addCleanup(left.close)
        self.addCleanup(right.close)
        header = pose_header()
        send_message(left, header)
        received, payload = receive_message(right)
        self.assertEqual(received, header)
        self.assertEqual(payload, b"")

    def test_cloud_roundtrip(self):
        left, right = socket.socketpair()
        self.addCleanup(left.close)
        self.addCleanup(right.close)
        payload = struct.pack("<8f", 1, 2, 3, 4, 5, 6, 7, 8)
        header = cloud_header()
        send_message(left, header, payload)
        received, received_payload = receive_message(right)
        self.assertEqual(received, header)
        self.assertEqual(received_payload, payload)

    def test_rejects_zero_orientation(self):
        with self.assertRaises(ValueError):
            validate_header(pose_header(orientation=[0.0, 0.0, 0.0, 0.0]), 0)

    def test_cloud_zero_points_roundtrip(self):
        left, right = socket.socketpair()
        self.addCleanup(left.close)
        self.addCleanup(right.close)
        header = cloud_header(point_count=0, point_bytes=0)
        send_message(left, header)
        received, payload = receive_message(right)
        self.assertEqual(received, header)
        self.assertEqual(payload, b"")

    def test_rejects_negative_point_count(self):
        with self.assertRaises(ValueError):
            validate_header(cloud_header(point_count=-1, point_bytes=0), 0)

    def test_rejects_cloud_point_bytes_mismatch(self):
        with self.assertRaises(ValueError):
            validate_header(cloud_header(point_bytes=16), 32)

    def test_rejects_cloud_payload_length_mismatch(self):
        with self.assertRaises(ValueError):
            validate_header(cloud_header(), 16)

    def test_rejects_unknown_type(self):
        with self.assertRaises(ValueError):
            validate_header(pose_header(type="unknown"), 0)

    def test_rejects_bad_magic(self):
        left, right = socket.socketpair()
        self.addCleanup(left.close)
        self.addCleanup(right.close)
        left.sendall(struct.pack("!4sII", b"BAD!", 2, 0) + b"{}")
        with self.assertRaises(ValueError):
            receive_message(right)

    def test_rejects_oversized_payload(self):
        left, right = socket.socketpair()
        self.addCleanup(left.close)
        self.addCleanup(right.close)
        left.sendall(struct.pack("!4sII", b"LDSM", 2, MAX_PAYLOAD_BYTES + 1))
        with self.assertRaises(ValueError):
            receive_message(right)


if __name__ == "__main__":
    unittest.main()
