import socket
import unittest

from frame_protocol import MAX_HEADER_BYTES, recv_frame, send_frame


class FrameProtocolTest(unittest.TestCase):
    def setUp(self):
        self.left, self.right = socket.socketpair()

    def tearDown(self):
        self.left.close()
        self.right.close()

    def test_round_trip(self):
        send_frame(self.left, {"kind": "color", "stamp": [1, 2]}, b"\xff\xd8jpeg")
        header, payload = recv_frame(self.right)
        self.assertEqual(header, {"kind": "color", "stamp": [1, 2]})
        self.assertEqual(payload, b"\xff\xd8jpeg")

    def test_empty_payload(self):
        send_frame(self.left, {"kind": "depth"}, b"")
        header, payload = recv_frame(self.right)
        self.assertEqual(header["kind"], "depth")
        self.assertEqual(payload, b"")

    def test_rejects_oversized_header(self):
        self.left.sendall(
            b"RBK1"
            + (MAX_HEADER_BYTES + 1).to_bytes(4, "big")
            + (0).to_bytes(4, "big")
        )
        with self.assertRaises(ValueError):
            recv_frame(self.right)

    def test_rejects_truncated_payload(self):
        self.left.sendall(
            b"RBK1" + (2).to_bytes(4, "big") + (100).to_bytes(4, "big") + b"{}"
        )
        self.left.shutdown(socket.SHUT_WR)
        with self.assertRaises(ValueError):
            recv_frame(self.right)


if __name__ == "__main__":
    unittest.main()
