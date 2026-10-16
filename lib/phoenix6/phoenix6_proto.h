/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wire protocol of the Phoenix 6 bridge (docs/phoenix6_bridge_protocol.md).
 */

#ifndef CANBUS_RP2040_LIB_PHOENIX6_PROTO_H_
#define CANBUS_RP2040_LIB_PHOENIX6_PROTO_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <canbus_rp2040/phoenix6.h>

#define PHOENIX6_PROTO_VERSION 1

/* Maximum size of payload + CRC */
#define PHOENIX6_MAX_PACKET  512
/* Maximum size of an encoded packet including the delimiter */
#define PHOENIX6_MAX_ENCODED (PHOENIX6_MAX_PACKET + PHOENIX6_MAX_PACKET / 254 + 2)

enum phoenix6_msg {
	PHOENIX6_MSG_HELLO = 0x01,
	PHOENIX6_MSG_PING = 0x02,
	PHOENIX6_MSG_ENABLE = 0x03,
	PHOENIX6_MSG_SET_CONTROL = 0x04,
	PHOENIX6_MSG_CONFIG_SET = 0x05,
	PHOENIX6_MSG_CONFIG_GET = 0x06,
	PHOENIX6_MSG_FACTORY_DEFAULT = 0x07,
	PHOENIX6_MSG_SUBSCRIBE = 0x08,
	PHOENIX6_MSG_SET_POSITION = 0x09,
	PHOENIX6_MSG_CLEAR_STICKY_FAULTS = 0x0A,
	PHOENIX6_MSG_HELLO_REPLY = 0x81,
	PHOENIX6_MSG_PONG = 0x82,
	PHOENIX6_MSG_RESULT = 0x83,
	PHOENIX6_MSG_SIGNALS = 0x84,
	PHOENIX6_MSG_LOG = 0x85,
};

/* Size of one entry of a SIGNALS message */
#define PHOENIX6_SIGNAL_ENTRY_SIZE (1 + 2 + 4 + 8 + 4)

struct phoenix6_writer {
	uint8_t *buf;
	size_t size;
	size_t len;
	bool overflow;
};

struct phoenix6_reader {
	const uint8_t *buf;
	size_t len;
	size_t pos;
	bool error;
};

void phoenix6_writer_init(struct phoenix6_writer *w, uint8_t *buf, size_t size);
void phoenix6_put_u8(struct phoenix6_writer *w, uint8_t v);
void phoenix6_put_u16(struct phoenix6_writer *w, uint16_t v);
void phoenix6_put_u32(struct phoenix6_writer *w, uint32_t v);
void phoenix6_put_i32(struct phoenix6_writer *w, int32_t v);
void phoenix6_put_f32(struct phoenix6_writer *w, float v);
void phoenix6_put_f64(struct phoenix6_writer *w, double v);

void phoenix6_reader_init(struct phoenix6_reader *r, const uint8_t *buf, size_t len);
uint8_t phoenix6_get_u8(struct phoenix6_reader *r);
uint16_t phoenix6_get_u16(struct phoenix6_reader *r);
uint32_t phoenix6_get_u32(struct phoenix6_reader *r);
int32_t phoenix6_get_i32(struct phoenix6_reader *r);
float phoenix6_get_f32(struct phoenix6_reader *r);
double phoenix6_get_f64(struct phoenix6_reader *r);

/* CRC-16/CCITT-FALSE */
uint16_t phoenix6_crc16(const uint8_t *data, size_t len);

/* Returns the encoded length; out must hold len + len / 254 + 1 bytes */
size_t phoenix6_cobs_encode(const uint8_t *in, size_t len, uint8_t *out);

/* Returns the decoded length or -EINVAL */
int phoenix6_cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_size);

/* Encode a complete packet including the 0x00 delimiter. Returns its length or -EMSGSIZE. */
int phoenix6_packet_encode(uint8_t type, uint16_t seq, const uint8_t *body, size_t body_len,
			   uint8_t *out, size_t out_size);

struct phoenix6_packet {
	uint8_t type;
	uint16_t seq;
	const uint8_t *body;
	size_t body_len;
};

struct phoenix6_frame_decoder {
	uint8_t raw[PHOENIX6_MAX_ENCODED];
	size_t raw_len;
	bool overflow;
	uint8_t frame[PHOENIX6_MAX_PACKET];
	uint32_t dropped;
};

void phoenix6_frame_decoder_init(struct phoenix6_frame_decoder *d);

/*
 * Feed one received byte. Returns true when a valid packet was completed; it
 * stays valid until the next call.
 */
bool phoenix6_frame_decoder_feed(struct phoenix6_frame_decoder *d, uint8_t byte,
				 struct phoenix6_packet *pkt);

/* Encode the parameters of a control request (schema order). */
int phoenix6_encode_control_params(struct phoenix6_writer *w,
				   const struct phoenix6_control_desc *desc, const uint8_t *params);

#endif /* CANBUS_RP2040_LIB_PHOENIX6_PROTO_H_ */
