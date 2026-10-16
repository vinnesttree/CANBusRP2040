# Copyright (c) 2026 CANBusRP2040 contributors
# SPDX-License-Identifier: Apache-2.0
"""Wire protocol; the vectors are shared with tests/lib/phoenix6/src/proto.c."""

import struct
import unittest

from phoenix6_bridge import protocol as p

HELLO_PACKET = bytes.fromhex("0b 01 02 01 01 b1 31 86 ca 37 45 00")
PING_PACKET = bytes.fromhex("02 02 01 03 fc a2 00")


class CrcTest(unittest.TestCase):
    def test_check_value(self):
        self.assertEqual(p.crc16(b"123456789"), 0x29B1)


class CobsTest(unittest.TestCase):
    def check(self, data: bytes, encoded: bytes):
        self.assertEqual(p.cobs_encode(data), encoded)
        self.assertEqual(p.cobs_decode(encoded), data)

    def test_short_vectors(self):
        # Reference vectors of the COBS article on Wikipedia
        self.check(b"", b"\x01")
        self.check(b"\x00", b"\x01\x01")
        self.check(b"\x00\x00", b"\x01\x01\x01")
        self.check(b"\x00\x11\x00", b"\x01\x02\x11\x01")
        self.check(b"\x11\x22\x00\x33", b"\x03\x11\x22\x02\x33")
        self.check(b"\x11\x22\x33\x44", b"\x05\x11\x22\x33\x44")
        self.check(b"\x11\x00\x00\x00", b"\x02\x11\x01\x01\x01")

    def test_long_vectors(self):
        b = bytes(range(1, 255))
        self.check(b, b"\xff" + b)
        self.check(b"\x00" + b, b"\x01\xff" + b)
        self.check(bytes(range(1, 256)), b"\xff" + b + b"\x02\xff")
        self.check(bytes(range(2, 256)) + b"\x00", b"\xff" + bytes(range(2, 256)) + b"\x01\x01")
        self.check(
            bytes(range(3, 256)) + b"\x00\x01", b"\xfe" + bytes(range(3, 256)) + b"\x02\x01"
        )

    def test_invalid(self):
        with self.assertRaises(p.ProtocolError):
            p.cobs_decode(b"\x02\x11\x00")
        with self.assertRaises(p.ProtocolError):
            p.cobs_decode(b"\x05\x11\x22")


class PacketTest(unittest.TestCase):
    def test_reference_packets(self):
        self.assertEqual(
            p.encode_packet(p.MsgType.HELLO, 0x0102, struct.pack("<BI", 1, 0xCA8631B1)),
            HELLO_PACKET,
        )
        self.assertEqual(p.encode_packet(p.MsgType.PING, 0), PING_PACKET)

    def test_limits(self):
        p.encode_packet(p.MsgType.LOG, 0, bytes(p.MAX_PACKET - 5))
        with self.assertRaises(p.ProtocolError):
            p.encode_packet(p.MsgType.LOG, 0, bytes(p.MAX_PACKET - 4))

    def test_decoder(self):
        d = p.FrameDecoder()
        pkts = list(d.feed(HELLO_PACKET))
        self.assertEqual(len(pkts), 1)
        self.assertEqual((pkts[0].type, pkts[0].seq), (p.MsgType.HELLO, 0x0102))
        self.assertEqual(pkts[0].body, bytes.fromhex("01 b1 31 86 ca"))

        corrupt = bytearray(HELLO_PACKET)
        corrupt[4] ^= 0x10
        self.assertEqual(list(d.feed(bytes(corrupt))), [])
        self.assertEqual(d.dropped, 1)
        self.assertEqual(len(list(d.feed(PING_PACKET))), 1)

        # Split across reads, empty frames ignored
        pkts = list(d.feed(b"\x00\x00" + PING_PACKET[:3])) + list(d.feed(PING_PACKET[3:]))
        self.assertEqual(len(pkts), 1)
        self.assertEqual(d.dropped, 1)

        # Largest packet, then an overlong frame
        big = p.encode_packet(p.MsgType.LOG, 7, bytes(i & 0xFF for i in range(p.MAX_PACKET - 5)))
        self.assertEqual(len(list(d.feed(big))), 1)
        self.assertEqual(list(d.feed(b"\x01" * 2000 + b"\x00")), [])
        self.assertEqual(d.dropped, 2)
        self.assertEqual(len(list(d.feed(PING_PACKET))), 1)


if __name__ == "__main__":
    unittest.main()
