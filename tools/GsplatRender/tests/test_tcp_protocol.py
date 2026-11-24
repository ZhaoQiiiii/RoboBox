import json
import socket
import struct
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

RENDERER_DIR = Path(__file__).resolve().parent.parent / "renderer"
sys.path.insert(0, str(RENDERER_DIR))

import tcp_protocol


def pose_header(**overrides):
    header = {
        "version": 1,
        "type": "pose",
        "seq": 7,
        "stamp_sec": 100,
        "stamp_nsec": 500,
        "position": [1.0, 2.0, 3.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }
    header.update(overrides)
    return header


def frame_header(**overrides):
    header = {
        "version": 1,
        "type": "frame",
        "frame_seq": 3,
        "pose_seq": 2,
        "stamp_sec": 100,
        "stamp_nsec": 500,
        "width": 4,
        "height": 3,
        "rgb_encoding": "rgb8",
        "depth_encoding": "16UC1",
        "rgb_bytes": 4 * 3 * 3,
        "depth_bytes": 4 * 3 * 2,
    }
    header.update(overrides)
    return header


def camera_info_header(**overrides):
    header = {
        "version": 1,
        "type": "camera_info",
        "width": 4,
        "height": 3,
        "fx": 100.0,
        "fy": 100.0,
        "cx": 2.0,
        "cy": 1.5,
        "frame_id": "camera",
        "cam_names": ["camera0", "camera1", "camera2", "camera3"],
        "rgb_encoding": "rgb8",
        "depth_encoding": "16UC1",
    }
    header.update(overrides)
    return header


class TcpProtocolTest(unittest.TestCase):
    def make_pair(self):
        reader, writer = socket.socketpair()
        self.addCleanup(reader.close)
        self.addCleanup(writer.close)
        return reader, writer

    def send_raw(self, writer, header_bytes, payload_length=0):
        writer.sendall(struct.pack("!4sII", b"GSPT", len(header_bytes), payload_length))
        writer.sendall(header_bytes)

    def test_fragmented_message(self):
        reader, writer = self.make_pair()
        header = frame_header(frame_seq=11)
        payload = bytes(range(4 * 3 * 3)) + b"\x01\x02" * (4 * 3 * 2 // 2)
        encoded = tcp_protocol.encode_message(header, payload)

        def send_byte_by_byte():
            for index in range(len(encoded)):
                writer.sendall(encoded[index:index + 1])

        sender = threading.Thread(target=send_byte_by_byte)
        sender.start()
        try:
            received_header, received_payload = tcp_protocol.receive_message(reader)
        finally:
            sender.join(timeout=5.0)
        self.assertFalse(sender.is_alive())
        self.assertEqual(received_header, header)
        self.assertEqual(received_payload, payload)

    def test_concatenated_messages(self):
        reader, writer = self.make_pair()
        first = pose_header(seq=1)
        second = camera_info_header()
        tcp_protocol.send_message(writer, first)
        tcp_protocol.send_message(writer, second)

        first_header, first_payload = tcp_protocol.receive_message(reader)
        second_header, second_payload = tcp_protocol.receive_message(reader)

        self.assertEqual(first_header, first)
        self.assertEqual(first_payload, b"")
        self.assertEqual(second_header, second)
        self.assertEqual(second_payload, b"")

    def test_partial_eof(self):
        reader, writer = self.make_pair()
        writer.sendall(struct.pack("!4sII", b"GSPT", 10, 0))
        writer.sendall(b"abc")
        writer.close()
        with self.assertRaises(EOFError):
            tcp_protocol.receive_message(reader)

    def test_invalid_lengths(self):
        oversized_headers = tcp_protocol.MAX_HEADER_BYTES + 1
        oversized_payloads = tcp_protocol.MAX_PAYLOAD_BYTES + 1
        for header_length, payload_length in ((oversized_headers, 0), (2, oversized_payloads)):
            with self.subTest(header_length=header_length, payload_length=payload_length):
                reader, writer = self.make_pair()
                reader.settimeout(1.0)
                writer.sendall(struct.pack("!4sII", b"GSPT", header_length, payload_length))
                started = time.monotonic()
                with self.assertRaises(ValueError):
                    tcp_protocol.receive_message(reader)
                self.assertLess(time.monotonic() - started, 1.0)

    def test_invalid_magic_version(self):
        reader, writer = self.make_pair()
        writer.sendall(struct.pack("!4sII", b"XXXX", 2, 0))
        writer.sendall(b"{}")
        with self.assertRaises(ValueError):
            tcp_protocol.receive_message(reader)

        reader, writer = self.make_pair()
        self.send_raw(writer, json.dumps(pose_header(version=2)).encode("utf-8"))
        with self.assertRaises(ValueError):
            tcp_protocol.receive_message(reader)

        reader, writer = self.make_pair()
        self.send_raw(writer, json.dumps(pose_header(type="bogus")).encode("utf-8"))
        with self.assertRaises(ValueError):
            tcp_protocol.receive_message(reader)

    def test_frame_shape_mismatch(self):
        mismatches = (
            {"rgb_bytes": 4 * 3 * 3 - 1},
            {"depth_bytes": 4 * 3 * 2 - 1},
            {"width": 0},
            {"rgb_encoding": "bgr8"},
        )
        for overrides in mismatches:
            with self.subTest(overrides=overrides):
                reader, writer = self.make_pair()
                reader.settimeout(1.0)
                header = frame_header(**overrides)
                payload_length = 4 * 3 * 3 + 4 * 3 * 2
                self.send_raw(writer, json.dumps(header).encode("utf-8"), payload_length)
                with self.assertRaises(ValueError):
                    tcp_protocol.receive_message(reader)

    def test_partial_message_deadline(self):
        reader, writer = self.make_pair()
        writer.sendall(struct.pack("!4sII", b"GSPT", 40, 0))
        writer.sendall(b"{")
        with mock.patch.object(tcp_protocol, "MESSAGE_DEADLINE_SECONDS", 0.2):
            started = time.monotonic()
            with self.assertRaises(TimeoutError):
                tcp_protocol.receive_message(reader)
            self.assertLess(time.monotonic() - started, 2.0)

    def test_idle_connection_does_not_time_out(self):
        reader, writer = self.make_pair()
        received = {}
        failure = {}

        def receive():
            try:
                received["message"] = tcp_protocol.receive_message(reader)
            except BaseException as error:  # noqa: BLE001 - 线程内异常需要带回主线程断言
                failure["error"] = error

        with mock.patch.object(tcp_protocol, "MESSAGE_DEADLINE_SECONDS", 0.2):
            receiver = threading.Thread(target=receive)
            receiver.start()
            time.sleep(0.4)
            self.assertTrue(receiver.is_alive(), "空闲等待不应触发消息 deadline")
            header = pose_header(seq=11)
            tcp_protocol.send_message(writer, header)
            receiver.join(timeout=2.0)

        self.assertFalse(receiver.is_alive())
        self.assertNotIn("error", failure)
        self.assertEqual(received["message"], (header, b""))

    def test_invalid_pose_rejected(self):
        bad_headers = (
            pose_header(position=[float("nan"), 0.0, 0.0]),
            pose_header(orientation=[0.0, 0.0, 0.0, 0.0]),
            pose_header(position=[1.0, 2.0]),
            pose_header(seq=-1),
        )
        for header in bad_headers:
            with self.subTest(header=header):
                reader, writer = self.make_pair()
                reader.settimeout(1.0)
                raw = json.dumps(header, allow_nan=True, separators=(",", ":")).encode("utf-8")
                self.send_raw(writer, raw)
                with self.assertRaises(ValueError):
                    tcp_protocol.receive_message(reader)


if __name__ == "__main__":
    unittest.main()
