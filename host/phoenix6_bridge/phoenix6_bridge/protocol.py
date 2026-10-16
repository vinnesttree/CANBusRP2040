# Copyright (c) 2026 CANBusRP2040 contributors
# SPDX-License-Identifier: Apache-2.0
"""Wire protocol of the Phoenix 6 bridge (see docs/phoenix6_bridge_protocol.md)."""

from __future__ import annotations

import binascii
import enum
import struct
from dataclasses import dataclass
from typing import Any, Iterator

PROTO_VERSION = 1
MAX_PACKET = 512

STATUS_OK = 0
BRIDGE_BAD_REQUEST = -20001
BRIDGE_NOT_ACCEPTED = -20002
BRIDGE_EXCEPTION = -20003

LOG_ERROR = 0
LOG_WARNING = 1
LOG_INFO = 2


class MsgType(enum.IntEnum):
    HELLO = 0x01
    PING = 0x02
    ENABLE = 0x03
    SET_CONTROL = 0x04
    CONFIG_SET = 0x05
    CONFIG_GET = 0x06
    FACTORY_DEFAULT = 0x07
    SUBSCRIBE = 0x08
    SET_POSITION = 0x09
    CLEAR_STICKY_FAULTS = 0x0A
    HELLO_REPLY = 0x81
    PONG = 0x82
    RESULT = 0x83
    SIGNALS = 0x84
    LOG = 0x85


class ProtocolError(Exception):
    """Malformed packet body."""


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE."""
    return binascii.crc_hqx(data, 0xFFFF)


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    block = bytearray()
    for i, b in enumerate(data):
        if b == 0:
            out.append(len(block) + 1)
            out += block
            block.clear()
        else:
            block.append(b)
            if len(block) == 254:
                out.append(255)
                out += block
                block.clear()
                if i + 1 == len(data):
                    # A full block at the end of the data needs no empty block
                    return bytes(out)
    out.append(len(block) + 1)
    out += block
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0:
            raise ProtocolError("zero byte in COBS data")
        i += 1
        end = i + code - 1
        if end > n:
            raise ProtocolError("truncated COBS block")
        out += data[i:end]
        i = end
        if code != 255 and i < n:
            out.append(0)
    return bytes(out)


def encode_packet(msg_type: int, seq: int, body: bytes = b"") -> bytes:
    payload = struct.pack("<BH", msg_type, seq & 0xFFFF) + body
    frame = payload + struct.pack("<H", crc16(payload))
    if len(frame) > MAX_PACKET:
        raise ProtocolError(f"packet too large ({len(frame)} bytes)")
    return cobs_encode(frame) + b"\x00"


@dataclass
class Packet:
    type: int
    seq: int
    body: bytes


class FrameDecoder:
    """Splits a byte stream into packets, dropping corrupted frames."""

    # Longest valid COBS encoding of a frame, without the delimiter
    MAX_ENCODED = MAX_PACKET + MAX_PACKET // 254 + 1

    def __init__(self) -> None:
        self._buf = bytearray()
        self._overflow = False
        self.dropped = 0

    def feed(self, data: bytes) -> Iterator[Packet]:
        for b in data:
            if b != 0:
                if len(self._buf) < self.MAX_ENCODED:
                    self._buf.append(b)
                else:
                    self._overflow = True
                continue
            raw = bytes(self._buf)
            overflow = self._overflow
            self._buf.clear()
            self._overflow = False
            if overflow:
                self.dropped += 1
                continue
            if not raw:
                continue
            try:
                frame = cobs_decode(raw)
            except ProtocolError:
                self.dropped += 1
                continue
            if len(frame) < 5 or len(frame) > MAX_PACKET:
                self.dropped += 1
                continue
            payload, crc = frame[:-2], struct.unpack("<H", frame[-2:])[0]
            if crc16(payload) != crc:
                self.dropped += 1
                continue
            msg_type, seq = struct.unpack("<BH", payload[:3])
            yield Packet(msg_type, seq, payload[3:])


class Reader:
    """Sequential reader for packet bodies."""

    def __init__(self, data: bytes) -> None:
        self._data = data
        self._pos = 0

    def _take(self, fmt: str) -> Any:
        size = struct.calcsize(fmt)
        if self._pos + size > len(self._data):
            raise ProtocolError("body too short")
        value = struct.unpack_from(fmt, self._data, self._pos)[0]
        self._pos += size
        return value

    def u8(self) -> int:
        return self._take("<B")

    def u16(self) -> int:
        return self._take("<H")

    def u32(self) -> int:
        return self._take("<I")

    def i32(self) -> int:
        return self._take("<i")

    def f32(self) -> float:
        return self._take("<f")

    def f64(self) -> float:
        return self._take("<d")

    def rest(self) -> bytes:
        data = self._data[self._pos:]
        self._pos = len(self._data)
        return data

    def done(self) -> None:
        if self._pos != len(self._data):
            raise ProtocolError("trailing bytes in body")


def encode_value(ptype: str, value: Any) -> bytes:
    if ptype == "float":
        return struct.pack("<d", float(value))
    if ptype == "bool":
        return struct.pack("<B", 1 if value else 0)
    if ptype in ("int", "enum"):
        return struct.pack("<i", int(value))
    raise ProtocolError(f"unknown value type {ptype}")


def decode_value(ptype: str, reader: Reader) -> Any:
    if ptype == "float":
        return reader.f64()
    if ptype == "bool":
        v = reader.u8()
        if v > 1:
            raise ProtocolError("bool out of range")
        return bool(v)
    if ptype in ("int", "enum"):
        return reader.i32()
    raise ProtocolError(f"unknown value type {ptype}")


def encode_control_params(schema: dict, control: dict, values: dict) -> bytes:
    """Encode the parameters of a control request (schema order)."""
    out = bytearray()
    by_name = {c["name"]: c for c in schema["controls"]}
    for p in control["params"]:
        if p["type"] == "control":
            out += encode_control_params(schema, by_name[p["control"]], values[p["name"]])
        else:
            value = values.get(p["name"], p.get("default"))
            if value is None:
                raise ProtocolError(f"missing parameter {p['name']}")
            out += encode_value(p["type"], value)
    return bytes(out)


def decode_control_params(schema: dict, control: dict, reader: Reader) -> dict:
    """Decode the parameters of a control request into a dict."""
    by_name = {c["name"]: c for c in schema["controls"]}
    values: dict[str, Any] = {}
    for p in control["params"]:
        if p["type"] == "control":
            values[p["name"]] = decode_control_params(schema, by_name[p["control"]], reader)
        else:
            values[p["name"]] = decode_value(p["type"], reader)
    return values
