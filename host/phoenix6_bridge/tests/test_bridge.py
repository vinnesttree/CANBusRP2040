# Copyright (c) 2026 CANBusRP2040 contributors
# SPDX-License-Identifier: Apache-2.0
"""Bridge against simulated Talon FX devices of the Phoenix 6 library."""

import struct
import time
import unittest

from phoenix6_bridge import protocol as p
from phoenix6_bridge.bridge import Bridge
from phoenix6_bridge.schema import load_schema, schema_hash

SCHEMA = load_schema()
HASH = schema_hash(SCHEMA)
CONTROLS = {c["name"]: c for c in SCHEMA["controls"]}
CONFIGS = {f"{f['group']}.{f['field']}": f for f in SCHEMA["configs"]}
SIGNALS = {s["name"]: s for s in SCHEMA["signals"]}
ENUMS = SCHEMA["enums"]
BRIDGE_ERRORS = (p.BRIDGE_BAD_REQUEST, p.BRIDGE_NOT_ACCEPTED, p.BRIDGE_EXCEPTION)


class MemoryTransport:
    def __init__(self):
        self.to_bridge = bytearray()
        self.from_bridge = bytearray()

    def read(self) -> bytes:
        data = bytes(self.to_bridge)
        self.to_bridge.clear()
        return data

    def write(self, data: bytes) -> None:
        self.from_bridge += data


class FakeClock:
    def __init__(self):
        self.now = 1000.0
        self.real = True

    def __call__(self) -> float:
        return time.monotonic() if self.real else self.now


class Client:
    """Minimal client side of the protocol, driving the bridge synchronously."""

    def __init__(self, bridge: Bridge, transport: MemoryTransport):
        self.bridge = bridge
        self.transport = transport
        self.decoder = p.FrameDecoder()
        self.seq = 0
        self.received: list[p.Packet] = []

    def send(self, msg_type: int, body: bytes = b"") -> int:
        seq = self.seq
        self.transport.to_bridge += p.encode_packet(msg_type, seq, body)
        self.seq += 1
        return seq

    def pump(self, cycles: int = 1) -> None:
        for _ in range(cycles):
            self.bridge.poll()
        self.received += list(self.decoder.feed(bytes(self.transport.from_bridge)))
        self.transport.from_bridge.clear()

    def take(self, msg_type: int) -> list[p.Packet]:
        out = [pkt for pkt in self.received if pkt.type == msg_type]
        self.received = [pkt for pkt in self.received if pkt.type != msg_type]
        return out

    def hello(self, schema_hash: int = HASH) -> p.Packet:
        self.send(p.MsgType.HELLO, struct.pack("<BI", p.PROTO_VERSION, schema_hash))
        self.pump()
        replies = self.take(p.MsgType.HELLO_REPLY)
        assert len(replies) == 1
        return replies[0]

    def request(self, msg_type: int, body: bytes) -> tuple[int, int, float]:
        """Send a request and return (device, status, value) of its RESULT."""
        seq = self.send(msg_type, body)
        self.pump()
        for pkt in self.take(p.MsgType.RESULT):
            r = p.Reader(pkt.body)
            rseq, rtype, device, status, value = r.u16(), r.u8(), r.u8(), r.i32(), r.f64()
            if rseq == seq:
                assert rtype == msg_type
                return device, status, value
        raise AssertionError(f"no RESULT for request 0x{msg_type:02x}")

    def set_control(self, device: int, name: str, freq: float = 100.0, **values) -> int:
        control = CONTROLS[name]
        body = struct.pack("<BHd", device, control["id"], freq)
        body += p.encode_control_params(SCHEMA, control, _defaults(control, values))
        return self.request(p.MsgType.SET_CONTROL, body)[1]

    def config_set(self, device: int, name: str, value: float, timeout_ms: int = 100) -> int:
        body = struct.pack("<BHdH", device, CONFIGS[name]["id"], value, timeout_ms)
        return self.request(p.MsgType.CONFIG_SET, body)[1]

    def config_get(self, device: int, name: str, timeout_ms: int = 100) -> tuple[int, float]:
        body = struct.pack("<BHH", device, CONFIGS[name]["id"], timeout_ms)
        _, status, value = self.request(p.MsgType.CONFIG_GET, body)
        return status, value

    def subscribe(self, device: int, name: str, freq: float) -> int:
        body = struct.pack("<BHf", device, SIGNALS[name]["id"], freq)
        return self.request(p.MsgType.SUBSCRIBE, body)[1]

    def signals(self) -> dict[tuple[int, int], tuple[int, float, float]]:
        out = {}
        for pkt in self.take(p.MsgType.SIGNALS):
            r = p.Reader(pkt.body)
            for _ in range(r.u8()):
                device, signal, status, value, age = r.u8(), r.u16(), r.i32(), r.f64(), r.f32()
                out[(device, signal)] = (status, value, age)
            r.done()
        return out


def _defaults(control: dict, values: dict) -> dict:
    """Parameter values: given ones, else the Phoenix 6 default, else zero."""
    out = {}
    for param in control["params"]:
        if param["type"] == "control":
            out[param["name"]] = _defaults(CONTROLS[param["control"]], values.get(param["name"], {}))
        else:
            out[param["name"]] = values.get(param["name"], param.get("default", 0))
    return out


def wait_for(predicate, timeout: float = 2.0, client: Client | None = None):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if client is not None:
            client.pump()
        result = predicate()
        if result:
            return result
        time.sleep(0.01)
    return predicate()


class BridgeTest(unittest.TestCase):
    def setUp(self):
        self.transport = MemoryTransport()
        self.clock = FakeClock()
        self.bridge = Bridge(self.transport, canbus="sim", schema=SCHEMA, clock=self.clock)
        self.client = Client(self.bridge, self.transport)

    def tearDown(self):
        self.bridge.stop_all()

    def connect(self):
        reply = self.client.hello()
        r = p.Reader(reply.body)
        self.assertEqual(r.u8(), p.PROTO_VERSION)
        self.assertEqual(r.u32(), HASH)
        self.assertEqual(r.u8(), 1)
        length = r.u8()
        self.assertEqual(r.rest().decode(), SCHEMA["phoenix6_version"])
        self.assertEqual(length, len(SCHEMA["phoenix6_version"]))
        self.assertTrue(self.bridge.accepted)

    def test_rejects_other_schema(self):
        reply = self.client.hello(HASH ^ 1)
        r = p.Reader(reply.body)
        r.u8()
        self.assertEqual(r.u32(), HASH)
        self.assertEqual(r.u8(), 0)
        self.assertFalse(self.bridge.accepted)
        self.assertEqual(len(self.client.take(p.MsgType.LOG)), 1)

        # Requests are refused and PING is not answered, so the client's
        # link times out and it repeats HELLO
        self.assertEqual(self.client.set_control(1, "NeutralOut"), p.BRIDGE_NOT_ACCEPTED)
        self.client.send(p.MsgType.PING)
        self.client.pump()
        self.assertEqual(self.client.take(p.MsgType.PONG), [])

    def test_ping(self):
        self.connect()
        self.client.send(p.MsgType.PING)
        self.client.pump()
        self.assertEqual(len(self.client.take(p.MsgType.PONG)), 1)

    def test_velocity_voltage(self):
        self.connect()
        self.client.send(p.MsgType.ENABLE, struct.pack("<H", 100))
        self.assertEqual(self.client.set_control(1, "VelocityVoltage", velocity=10.0, slot=1), 0)

        talon = self.bridge._devices[1]
        mode = talon.get_control_mode()
        self.assertTrue(
            wait_for(lambda: mode.refresh().value.name.startswith("VELOCITY_VOLTAGE")), mode.value
        )
        reference = talon.get_closed_loop_reference()
        self.assertTrue(wait_for(lambda: reference.refresh().value == 10.0), reference.value)
        self.assertEqual(talon.get_closed_loop_slot().refresh().value, 1)

    def test_every_control_request(self):
        """Every request of the schema can be built and sent with default values."""
        self.connect()
        for name in CONTROLS:
            with self.subTest(control=name):
                status = self.client.set_control(2, name)
                self.assertNotIn(status, BRIDGE_ERRORS, name)
                self.assertEqual(self.client.take(p.MsgType.LOG), [])

    def test_enum_and_nested_parameters(self):
        self.connect()
        aligned = ENUMS["MotorAlignmentValue"]["OPPOSED"]
        self.assertNotIn(
            self.client.set_control(3, "Follower", leader_id=1, motor_alignment=aligned),
            BRIDGE_ERRORS,
        )
        request = self.bridge._build_control(
            CONTROLS["Diff_VelocityVoltage_Position"],
            _defaults(
                CONTROLS["Diff_VelocityVoltage_Position"],
                {"average_request": {"velocity": 5.0}, "differential_request": {"position": 0.25}},
            ),
        )
        self.assertEqual(request.average_request.velocity, 5.0)
        self.assertEqual(request.differential_request.position, 0.25)
        self.assertTrue(request.average_request.enable_foc)

    def test_configs(self):
        self.connect()
        self.assertEqual(self.client.config_set(4, "slot0.k_p", 1.5), 0)
        self.assertEqual(self.client.config_set(4, "slot0.k_i", 0.25), 0)
        self.assertEqual(self.client.config_get(4, "slot0.k_p"), (0, 1.5))
        self.assertEqual(self.client.config_get(4, "slot0.k_i"), (0, 0.25))

        brake = ENUMS["NeutralModeValue"]["BRAKE"]
        self.assertEqual(self.client.config_set(4, "motor_output.neutral_mode", brake), 0)
        self.assertEqual(self.client.config_get(4, "motor_output.neutral_mode"), (0, brake))

        body = struct.pack("<BH", 4, 100)
        self.assertEqual(self.client.request(p.MsgType.FACTORY_DEFAULT, body)[1], 0)
        self.assertEqual(self.client.config_get(4, "slot0.k_p"), (0, 0.0))

    def test_every_config_field(self):
        self.connect()
        for name, field in CONFIGS.items():
            with self.subTest(config=name):
                status, value = self.client.config_get(5, name)
                self.assertEqual(status, 0, name)
                # The device stores single precision values
                self.assertAlmostEqual(value, float(field["default"]), places=6, msg=name)

    def test_set_position_and_signals(self):
        self.connect()
        body = struct.pack("<BdH", 6, 3.0, 100)
        self.assertEqual(self.client.request(p.MsgType.SET_POSITION, body)[1], 0)
        self.assertNotIn(self.client.subscribe(6, "position", 50.0), BRIDGE_ERRORS)
        self.assertNotIn(self.client.subscribe(6, "control_mode", 50.0), BRIDGE_ERRORS)

        key = (6, SIGNALS["position"]["id"])
        got = wait_for(lambda: self.client.signals().get(key), client=self.client)
        self.assertIsNotNone(got)
        status, value, age = got
        self.assertEqual(status, 0)
        self.assertAlmostEqual(value, 3.0, places=3)
        self.assertGreaterEqual(age, 0.0)

        # Unsubscribe stops the stream
        self.assertEqual(self.client.subscribe(6, "position", 0.0), 0)
        self.client.pump()
        self.client.take(p.MsgType.SIGNALS)
        time.sleep(0.1)
        self.client.pump()
        self.assertNotIn(key, self.client.signals())

    def test_every_signal(self):
        self.connect()
        for name in SIGNALS:
            with self.subTest(signal=name):
                self.assertNotIn(self.client.subscribe(7, name, 10.0), BRIDGE_ERRORS, name)
        self.client.take(p.MsgType.SIGNALS)
        time.sleep(0.15)
        self.client.pump()
        received = {signal for (_, signal) in self.client.signals()}
        self.assertEqual(received, {s["id"] for s in SIGNALS.values()})

    def test_bad_requests(self):
        self.connect()
        # Truncated body, unknown control, invalid device, unknown config
        _, status, _ = self.client.request(p.MsgType.SET_CONTROL, b"\x01\x2c")
        self.assertEqual(status, p.BRIDGE_BAD_REQUEST)
        self.assertEqual(
            self.client.request(p.MsgType.SET_CONTROL, struct.pack("<BHd", 1, 999, 100.0))[1],
            p.BRIDGE_BAD_REQUEST,
        )
        self.assertEqual(self.client.set_control(63, "NeutralOut"), p.BRIDGE_BAD_REQUEST)
        self.assertEqual(
            self.client.request(p.MsgType.CONFIG_GET, struct.pack("<BHH", 1, 999, 10))[1],
            p.BRIDGE_BAD_REQUEST,
        )
        self.assertEqual(
            self.client.request(p.MsgType.SUBSCRIBE, struct.pack("<BHf", 1, 1, 5000.0))[1],
            p.BRIDGE_BAD_REQUEST,
        )
        self.assertTrue(self.bridge.accepted)

    def test_link_timeout_stops_motors(self):
        from phoenix6 import unmanaged

        self.connect()
        self.client.send(p.MsgType.ENABLE, struct.pack("<H", 1000))
        self.client.pump()
        self.assertTrue(unmanaged.get_enable_state())
        self.assertEqual(self.client.set_control(8, "DutyCycleOut", output=0.5), 0)
        self.client.subscribe(8, "duty_cycle", 50.0)

        # Nothing received for longer than the link timeout
        self.clock.now = time.monotonic()
        self.clock.real = False
        self.client.pump()
        self.assertTrue(self.bridge.accepted)
        self.clock.now += 0.6
        self.client.pump()

        self.assertFalse(self.bridge.accepted)
        self.assertFalse(unmanaged.get_enable_state())
        talon = self.bridge._devices[8]
        self.assertTrue(
            wait_for(lambda: talon.get_control_mode().refresh().value.name == "NEUTRAL_OUT")
        )
        # Subscriptions belong to the session
        self.client.take(p.MsgType.SIGNALS)
        self.clock.now += 0.1
        self.client.pump()
        self.assertEqual(self.client.signals(), {})

    def test_new_hello_starts_new_session(self):
        self.connect()
        self.client.subscribe(9, "velocity", 50.0)
        self.assertEqual(self.client.set_control(9, "DutyCycleOut", output=0.3), 0)
        self.connect()
        talon = self.bridge._devices[9]
        self.assertTrue(
            wait_for(lambda: talon.get_control_mode().refresh().value.name == "NEUTRAL_OUT")
        )
        self.client.take(p.MsgType.SIGNALS)
        time.sleep(0.05)
        self.client.pump()
        self.assertEqual(self.client.signals(), {})


if __name__ == "__main__":
    unittest.main()
