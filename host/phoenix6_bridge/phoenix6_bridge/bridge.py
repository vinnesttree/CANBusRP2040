# Copyright (c) 2026 CANBusRP2040 contributors
# SPDX-License-Identifier: Apache-2.0
"""
Executes Phoenix 6 requests from a microcontroller with the official library.

The microcontroller (client) sends typed requests over a serial link (see
docs/phoenix6_bridge_protocol.md); this bridge runs them on Talon FX devices
with CTR Electronics' ``phoenix6`` package and streams status signals back.
"""

from __future__ import annotations

import logging
import struct
import time
from dataclasses import dataclass
from typing import Any, Callable, Protocol

from . import protocol as p
from .schema import SCHEMA_FORMAT, compare_with_installed, load_schema, schema_hash

log = logging.getLogger("phoenix6_bridge")

# Signals per SIGNALS packet: type/seq (3) + count (1) + CRC (2)
_SIGNAL_ENTRY = struct.Struct("<BHidf")
MAX_SIGNALS_PER_PACKET = (p.MAX_PACKET - 6) // _SIGNAL_ENTRY.size
MAX_SIGNAL_FREQ_HZ = 1000.0
MAX_LOG_TEXT = 200


class Transport(Protocol):
    def read(self) -> bytes:
        """Return received bytes, waiting at most a few milliseconds."""

    def write(self, data: bytes) -> None: ...


class SerialTransport:
    """
    Serial port (e.g. the microcontroller's USB CDC ACM port) via pyserial.

    The port disappears while the microcontroller resets; it is reopened when
    it comes back. Nothing is received in the meantime, so the bridge's link
    timeout ends the session and stops the motors.
    """

    REOPEN_INTERVAL = 0.5

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        import serial

        self._serial_mod = serial
        self._port = port
        self._baudrate = baudrate
        self._serial = None
        self._next_open = 0.0
        self._open()
        if self._serial is None:
            raise OSError(f"cannot open {port}")

    def _open(self) -> None:
        try:
            self._serial = self._serial_mod.Serial(
                self._port, baudrate=self._baudrate, timeout=0.005
            )
            log.info("opened %s", self._port)
        except (OSError, self._serial_mod.SerialException) as e:
            self._serial = None
            self._next_open = time.monotonic() + self.REOPEN_INTERVAL
            log.debug("cannot open %s: %s", self._port, e)

    def _lost(self, error: Exception) -> None:
        log.warning("%s disconnected (%s), waiting for it to come back", self._port, error)
        try:
            self._serial.close()
        except Exception:  # noqa: BLE001
            pass
        self._serial = None
        self._next_open = time.monotonic() + self.REOPEN_INTERVAL

    def read(self) -> bytes:
        if self._serial is None:
            if time.monotonic() >= self._next_open:
                self._open()
            if self._serial is None:
                time.sleep(0.01)
                return b""
        try:
            return self._serial.read(max(1, self._serial.in_waiting))
        except (OSError, self._serial_mod.SerialException) as e:
            self._lost(e)
            return b""

    def write(self, data: bytes) -> None:
        if self._serial is None:
            return
        try:
            self._serial.write(data)
        except (OSError, self._serial_mod.SerialException) as e:
            self._lost(e)

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None


@dataclass
class _Subscription:
    signal: Any
    period: float
    next_due: float


class Bridge:
    """Bridge between one client and the Talon FX devices on one CAN bus."""

    def __init__(
        self,
        transport: Transport,
        canbus: str = "",
        schema: dict | None = None,
        link_timeout: float = 0.5,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        import phoenix6
        from phoenix6 import configs, controls, hardware, unmanaged
        from phoenix6.controls import compound
        from phoenix6.signals import spn_enums

        self._phoenix6 = phoenix6
        self._configs = configs
        self._controls = controls
        self._compound = compound
        self._hardware = hardware
        self._unmanaged = unmanaged
        self._enums = spn_enums

        self.schema = schema if schema is not None else load_schema()
        if self.schema.get("format") != SCHEMA_FORMAT:
            raise RuntimeError(f"unsupported schema format {self.schema.get('format')}")
        self.hash = schema_hash(self.schema)
        self.version = self.schema["phoenix6_version"]

        self._transport = transport
        self._canbus = phoenix6.CANBus(canbus)
        self._link_timeout = link_timeout
        self._clock = clock
        self._decoder = p.FrameDecoder()
        self._seq = 0

        self._controls_by_id = {c["id"]: c for c in self.schema["controls"]}
        self._controls_by_name = {c["name"]: c for c in self.schema["controls"]}
        self._configs_by_id = {f["id"]: f for f in self.schema["configs"]}
        self._signals_by_id = {s["id"]: s for s in self.schema["signals"]}

        self._devices: dict[int, Any] = {}
        self._used: set[int] = set()
        self._subs: dict[tuple[int, int], _Subscription] = {}
        self.accepted = False
        self._last_rx = 0.0

    # Transport helpers

    def _send(self, msg_type: int, body: bytes = b"") -> None:
        self._transport.write(p.encode_packet(msg_type, self._seq, body))
        self._seq = (self._seq + 1) & 0xFFFF

    def _result(self, pkt: p.Packet, device: int, status: int, value: float = 0.0) -> None:
        body = struct.pack("<HBBid", pkt.seq, pkt.type, device & 0xFF, int(status), float(value))
        self._send(p.MsgType.RESULT, body)

    def _log(self, level: int, text: str) -> None:
        (log.error if level == p.LOG_ERROR else log.warning if level == p.LOG_WARNING else log.info)(
            text
        )
        data = text.encode("utf-8", "replace")[:MAX_LOG_TEXT]
        self._send(p.MsgType.LOG, bytes([level]) + data)

    # Devices

    def _device(self, device_id: int) -> Any:
        if not 0 <= device_id <= 62:
            raise p.ProtocolError(f"invalid device ID {device_id}")
        talon = self._devices.get(device_id)
        if talon is None:
            talon = self._hardware.TalonFX(device_id, self._canbus)
            self._devices[device_id] = talon
        return talon

    def stop_all(self) -> None:
        """Command neutral output on every device used and disable actuators."""
        neutral = self._controls.NeutralOut()
        for device_id in sorted(self._used):
            try:
                self._devices[device_id].set_control(neutral)
            except Exception:  # noqa: BLE001 - keep stopping the others
                log.exception("failed to stop device %d", device_id)
        self._unmanaged.feed_enable(0)

    def _end_session(self, reason: str) -> None:
        if self.accepted:
            log.warning("session ended: %s", reason)
        self.stop_all()
        self.accepted = False
        self._subs.clear()

    # Request handlers

    def _handle_hello(self, pkt: p.Packet) -> None:
        r = p.Reader(pkt.body)
        version = r.u8()
        client_hash = r.u32()
        ok = version == p.PROTO_VERSION and client_hash == self.hash

        # A new session: stop what the previous one did
        self._end_session("new session")
        self.accepted = ok

        ver = self.version.encode()
        body = struct.pack("<BIBB", p.PROTO_VERSION, self.hash, 1 if ok else 0, len(ver)) + ver
        self._send(p.MsgType.HELLO_REPLY, body)

        if ok:
            log.info("client connected")
        else:
            self._log(
                p.LOG_ERROR,
                f"rejected client: protocol {version} (bridge {p.PROTO_VERSION}), schema "
                f"0x{client_hash:08x} (bridge 0x{self.hash:08x}, Phoenix 6 {self.version}). "
                "Rebuild the firmware from the bridge's schema.json.",
            )

    def _enum(self, name: str, value: int) -> Any:
        return getattr(self._enums, name)(int(value))

    def _build_control(self, control: dict, values: dict) -> Any:
        cls = getattr(self._controls, control["name"], None) or getattr(
            self._compound, control["name"]
        )
        kwargs = {}
        for param in control["params"]:
            value = values[param["name"]]
            if param["type"] == "control":
                value = self._build_control(self._controls_by_name[param["control"]], value)
            elif param["type"] == "enum":
                value = self._enum(param["enum"], value)
            elif param["type"] == "int":
                value = int(value)
            kwargs[param["name"]] = value
        return cls(**kwargs)

    def _handle_set_control(self, pkt: p.Packet) -> None:
        r = p.Reader(pkt.body)
        device = r.u8()
        control = self._controls_by_id.get(r.u16())
        update_freq = r.f64()
        if control is None:
            raise p.ProtocolError("unknown control request")
        values = p.decode_control_params(self.schema, control, r)
        r.done()

        request = self._build_control(control, values)
        if hasattr(request, "update_freq_hz"):
            request.update_freq_hz = update_freq
        talon = self._device(device)
        self._used.add(device)
        status = talon.set_control(request)
        self._result(pkt, device, status.value)

    def _config_group(self, field: dict) -> Any:
        return getattr(self._configs, field["group_class"])()

    def _config_value(self, field: dict, value: float) -> Any:
        if field["type"] == "bool":
            return value != 0.0
        if field["type"] == "int":
            return int(value)
        if field["type"] == "enum":
            return self._enum(field["enum"], int(value))
        return float(value)

    @staticmethod
    def _as_double(value: Any) -> float:
        if hasattr(value, "value"):
            value = value.value
        return float(value)

    def _handle_config_set(self, pkt: p.Packet) -> None:
        r = p.Reader(pkt.body)
        device = r.u8()
        field = self._configs_by_id.get(r.u16())
        value = r.f64()
        timeout = r.u16() / 1000.0
        r.done()
        if field is None:
            raise p.ProtocolError("unknown configuration field")

        talon = self._device(device)
        group = self._config_group(field)
        # Read-modify-write so the other fields of the group keep their values
        status = talon.configurator.refresh(group, timeout)
        if status.is_ok():
            setattr(group, field["field"], self._config_value(field, value))
            status = talon.configurator.apply(group, timeout)
        self._result(pkt, device, status.value)

    def _handle_config_get(self, pkt: p.Packet) -> None:
        r = p.Reader(pkt.body)
        device = r.u8()
        field = self._configs_by_id.get(r.u16())
        timeout = r.u16() / 1000.0
        r.done()
        if field is None:
            raise p.ProtocolError("unknown configuration field")

        talon = self._device(device)
        group = self._config_group(field)
        status = talon.configurator.refresh(group, timeout)
        value = self._as_double(getattr(group, field["field"])) if status.is_ok() else 0.0
        self._result(pkt, device, status.value, value)

    def _handle_factory_default(self, pkt: p.Packet) -> None:
        r = p.Reader(pkt.body)
        device = r.u8()
        timeout = r.u16() / 1000.0
        r.done()
        status = self._device(device).configurator.apply(
            self._configs.TalonFXConfiguration(), timeout
        )
        self._result(pkt, device, status.value)

    def _handle_set_position(self, pkt: p.Packet) -> None:
        r = p.Reader(pkt.body)
        device = r.u8()
        value = r.f64()
        timeout = r.u16() / 1000.0
        r.done()
        status = self._device(device).set_position(value, timeout)
        self._result(pkt, device, status.value)

    def _handle_clear_sticky_faults(self, pkt: p.Packet) -> None:
        r = p.Reader(pkt.body)
        device = r.u8()
        timeout = r.u16() / 1000.0
        r.done()
        status = self._device(device).clear_sticky_faults(timeout)
        self._result(pkt, device, status.value)

    def _handle_subscribe(self, pkt: p.Packet) -> None:
        r = p.Reader(pkt.body)
        device = r.u8()
        signal_id = r.u16()
        freq = r.f32()
        r.done()
        desc = self._signals_by_id.get(signal_id)
        if desc is None or not 0.0 <= freq <= MAX_SIGNAL_FREQ_HZ:
            raise p.ProtocolError("invalid subscription")

        key = (device, signal_id)
        if freq == 0.0:
            # The signal's update frequency on the CAN bus is left unchanged
            self._subs.pop(key, None)
            self._result(pkt, device, p.STATUS_OK)
            return

        signal = getattr(self._device(device), f"get_{desc['name']}")(False)
        status = signal.set_update_frequency(freq, 0.1)
        self._subs[key] = _Subscription(signal, 1.0 / freq, self._clock())
        self._result(pkt, device, status.value)

    def _handle_enable(self, pkt: p.Packet) -> None:
        r = p.Reader(pkt.body)
        timeout_ms = r.u16()
        r.done()
        self._unmanaged.feed_enable(timeout_ms / 1000.0)

    _HANDLERS = {
        p.MsgType.SET_CONTROL: _handle_set_control,
        p.MsgType.CONFIG_SET: _handle_config_set,
        p.MsgType.CONFIG_GET: _handle_config_get,
        p.MsgType.FACTORY_DEFAULT: _handle_factory_default,
        p.MsgType.SET_POSITION: _handle_set_position,
        p.MsgType.CLEAR_STICKY_FAULTS: _handle_clear_sticky_faults,
        p.MsgType.SUBSCRIBE: _handle_subscribe,
    }

    def handle_packet(self, pkt: p.Packet) -> None:
        self._last_rx = self._clock()

        if pkt.type == p.MsgType.HELLO:
            try:
                self._handle_hello(pkt)
            except p.ProtocolError:
                log.warning("malformed HELLO")
            return

        if not self.accepted:
            # No PONG either, so the client's link times out and it sends HELLO
            if pkt.type in self._HANDLERS:
                device = pkt.body[0] if pkt.body else 0
                self._result(pkt, device, p.BRIDGE_NOT_ACCEPTED)
            return

        if pkt.type == p.MsgType.PING:
            self._send(p.MsgType.PONG)
            return

        if pkt.type == p.MsgType.ENABLE:
            try:
                self._handle_enable(pkt)
            except p.ProtocolError:
                log.warning("malformed ENABLE")
            return

        handler = self._HANDLERS.get(pkt.type)
        if handler is None:
            log.debug("ignoring message type 0x%02x", pkt.type)
            return

        device = pkt.body[0] if pkt.body else 0
        try:
            handler(self, pkt)
        except p.ProtocolError as e:
            self._log(p.LOG_WARNING, f"bad request 0x{pkt.type:02x}: {e}")
            self._result(pkt, device, p.BRIDGE_BAD_REQUEST)
        except Exception as e:  # noqa: BLE001 - report library errors to the client
            log.exception("request 0x%02x failed", pkt.type)
            self._log(p.LOG_ERROR, f"request 0x{pkt.type:02x} failed: {type(e).__name__}: {e}")
            self._result(pkt, device, p.BRIDGE_EXCEPTION)

    # Periodic work

    def _stream_signals(self, now: float) -> None:
        due = [(key, sub) for key, sub in self._subs.items() if now >= sub.next_due]
        if not due:
            return

        self._phoenix6.BaseStatusSignal.refresh_all(
            [sub.signal for _, sub in due], report_error=False
        )

        entries = []
        for (device, signal_id), sub in due:
            sig = sub.signal
            age_ms = max(0.0, sig.timestamp.get_latency() * 1000.0)
            entries.append(
                _SIGNAL_ENTRY.pack(device, signal_id, sig.status.value, sig.value_as_double, age_ms)
            )
            # Keep the phase, but do not try to catch up after a stall
            sub.next_due = max(sub.next_due + sub.period, now)

        for i in range(0, len(entries), MAX_SIGNALS_PER_PACKET):
            chunk = entries[i : i + MAX_SIGNALS_PER_PACKET]
            self._send(p.MsgType.SIGNALS, bytes([len(chunk)]) + b"".join(chunk))

    def poll(self) -> None:
        """Process received data and periodic work once."""
        data = self._transport.read()
        for pkt in self._decoder.feed(data):
            self.handle_packet(pkt)

        now = self._clock()
        if self.accepted and now - self._last_rx > self._link_timeout:
            self._end_session("link timeout")
        if self.accepted:
            self._stream_signals(now)

    def run(self, should_stop: Callable[[], bool] = lambda: False) -> None:
        try:
            while not should_stop():
                self.poll()
        finally:
            self._end_session("bridge stopped")


def check_installed(schema: dict) -> None:
    """Raise if ``schema`` does not match the installed phoenix6 package."""
    problems = compare_with_installed(schema)
    if problems:
        raise RuntimeError(
            "schema.json does not match the installed phoenix6 package:\n  "
            + "\n  ".join(problems)
            + "\nInstall phoenix6 "
            + schema["phoenix6_version"]
            + ", or regenerate schema.json and the firmware (scripts/gen_phoenix6.py)."
        )
