# Phoenix 6 bridge protocol

The RP2040 (client) and the host bridge (`host/phoenix6_bridge`) exchange
binary packets over a serial link (USB CDC ACM or a UART). The host executes
every request with CTR Electronics' official Phoenix 6 library.

All multi-byte values are little-endian. `f64` is an IEEE 754 double, `f32` an
IEEE 754 single.

## Framing

```
packet  = COBS(payload || crc16) || 0x00
payload = type:u8  seq:u16  body
crc16   = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection) of payload
```

The decoded `payload || crc16` is at most 512 bytes. Packets with a bad CRC or
an oversized body are dropped. `seq` is incremented by the sender for every
packet; `RESULT` echoes the `seq` and type of the request it answers.

## Schema

IDs of control requests, configuration fields and signals, the parameter
order of every control request and all enum values come from `schema.json`,
which is generated from the installed `phoenix6` package. The firmware is
built from the same file (`include/canbus_rp2040/phoenix6_generated.h`).
`schema_hash` is the CRC-32 (IEEE) of the canonical JSON encoding of the wire
relevant parts of the schema (see `schema.py`). The host rejects a client
whose hash differs from its own, and refuses to start if `schema.json` does not
match the installed `phoenix6` package.

Values are encoded by type:

| Schema type | Encoding |
|---|---|
| `float` | f64 |
| `bool` | u8 (0 or 1) |
| `int` | i32 |
| `enum` | i32 (Phoenix 6 enum value) |
| `control` | the parameters of the nested request, in schema order |

## Client to host

| Type | Name | Body | Reply |
|---|---|---|---|
| 0x01 | HELLO | proto_version:u8 (1), schema_hash:u32 | HELLO_REPLY |
| 0x02 | PING | – | PONG |
| 0x03 | ENABLE | timeout_ms:u16 | – (`unmanaged.feed_enable(timeout)`, 0 disables) |
| 0x04 | SET_CONTROL | device:u8, control_id:u16, update_freq_hz:f64, params | RESULT |
| 0x05 | CONFIG_SET | device:u8, field_id:u16, value:f64, timeout_ms:u16 | RESULT |
| 0x06 | CONFIG_GET | device:u8, field_id:u16, timeout_ms:u16 | RESULT (value) |
| 0x07 | FACTORY_DEFAULT | device:u8, timeout_ms:u16 | RESULT |
| 0x08 | SUBSCRIBE | device:u8, signal_id:u16, freq_hz:f32 (0: unsubscribe) | RESULT |
| 0x09 | SET_POSITION | device:u8, value:f64, timeout_ms:u16 | RESULT |
| 0x0A | CLEAR_STICKY_FAULTS | device:u8, timeout_ms:u16 | RESULT |

`device` is the CAN ID of the Talon FX. `CONFIG_SET` reads the configuration
group from the device, changes one field and writes the group back, so other
fields of the group keep their values. Configuration values are carried as
f64; booleans are 0/1 and enums their Phoenix 6 value.

## Host to client

| Type | Name | Body |
|---|---|---|
| 0x81 | HELLO_REPLY | proto_version:u8, schema_hash:u32, accepted:u8, version_len:u8, phoenix6_version |
| 0x82 | PONG | – |
| 0x83 | RESULT | request_seq:u16, request_type:u8, device:u8, status:i32, value:f64 |
| 0x84 | SIGNALS | count:u8, then count × (device:u8, signal_id:u16, status:i32, value:f64, age_ms:f32) |
| 0x85 | LOG | level:u8 (0 error, 1 warning, 2 info), text |

`status` is a Phoenix 6 `StatusCode` value (0 = OK) or one of the bridge codes:

| Code | Meaning |
|---|---|
| -20001 | Malformed request (unknown ID, wrong length, value out of range) |
| -20002 | Session not accepted (no HELLO or schema mismatch) |
| -20003 | Exception raised by the Phoenix 6 library |

`SIGNALS` carry the latest value of each subscribed signal at the subscribed
rate. `age_ms` is the age of the value when the packet was sent.

## Link supervision

The client sends PING every 100 ms, independent of other traffic, and the host
answers each with PONG; ENABLE and other requests do not replace it, since
ENABLE gets no answer. The client repeats HELLO until it is accepted. Each side
treats the link as lost after 500 ms without a valid packet. The host does not
answer PING outside an accepted session, so a client whose session the host
ended times out and starts a new one with HELLO. On link loss the host commands `NeutralOut` on every
Talon FX it has used and calls `feed_enable(0)`, so actuators stop even before
the last enable timeout expires. A new HELLO starts a new session and clears
all subscriptions.
