/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#include "phoenix6_proto.h"

void phoenix6_writer_init(struct phoenix6_writer *w, uint8_t *buf, size_t size)
{
	w->buf = buf;
	w->size = size;
	w->len = 0;
	w->overflow = false;
}

static uint8_t *writer_reserve(struct phoenix6_writer *w, size_t n)
{
	uint8_t *p;

	if (w->overflow || w->len + n > w->size) {
		w->overflow = true;
		return NULL;
	}

	p = &w->buf[w->len];
	w->len += n;

	return p;
}

void phoenix6_put_u8(struct phoenix6_writer *w, uint8_t v)
{
	uint8_t *p = writer_reserve(w, 1);

	if (p != NULL) {
		*p = v;
	}
}

void phoenix6_put_u16(struct phoenix6_writer *w, uint16_t v)
{
	uint8_t *p = writer_reserve(w, 2);

	if (p != NULL) {
		sys_put_le16(v, p);
	}
}

void phoenix6_put_u32(struct phoenix6_writer *w, uint32_t v)
{
	uint8_t *p = writer_reserve(w, 4);

	if (p != NULL) {
		sys_put_le32(v, p);
	}
}

void phoenix6_put_i32(struct phoenix6_writer *w, int32_t v)
{
	phoenix6_put_u32(w, (uint32_t)v);
}

void phoenix6_put_f32(struct phoenix6_writer *w, float v)
{
	uint32_t bits;

	memcpy(&bits, &v, sizeof(bits));
	phoenix6_put_u32(w, bits);
}

void phoenix6_put_f64(struct phoenix6_writer *w, double v)
{
	uint8_t *p = writer_reserve(w, 8);
	uint64_t bits;

	memcpy(&bits, &v, sizeof(bits));
	if (p != NULL) {
		sys_put_le64(bits, p);
	}
}

void phoenix6_reader_init(struct phoenix6_reader *r, const uint8_t *buf, size_t len)
{
	r->buf = buf;
	r->len = len;
	r->pos = 0;
	r->error = false;
}

static const uint8_t *reader_take(struct phoenix6_reader *r, size_t n)
{
	const uint8_t *p;

	if (r->error || r->pos + n > r->len) {
		r->error = true;
		return NULL;
	}

	p = &r->buf[r->pos];
	r->pos += n;

	return p;
}

uint8_t phoenix6_get_u8(struct phoenix6_reader *r)
{
	const uint8_t *p = reader_take(r, 1);

	return (p != NULL) ? *p : 0U;
}

uint16_t phoenix6_get_u16(struct phoenix6_reader *r)
{
	const uint8_t *p = reader_take(r, 2);

	return (p != NULL) ? sys_get_le16(p) : 0U;
}

uint32_t phoenix6_get_u32(struct phoenix6_reader *r)
{
	const uint8_t *p = reader_take(r, 4);

	return (p != NULL) ? sys_get_le32(p) : 0U;
}

int32_t phoenix6_get_i32(struct phoenix6_reader *r)
{
	return (int32_t)phoenix6_get_u32(r);
}

float phoenix6_get_f32(struct phoenix6_reader *r)
{
	uint32_t bits = phoenix6_get_u32(r);
	float v;

	memcpy(&v, &bits, sizeof(v));

	return v;
}

double phoenix6_get_f64(struct phoenix6_reader *r)
{
	const uint8_t *p = reader_take(r, 8);
	uint64_t bits = (p != NULL) ? sys_get_le64(p) : 0U;
	double v;

	memcpy(&v, &bits, sizeof(v));

	return v;
}

uint16_t phoenix6_crc16(const uint8_t *data, size_t len)
{
	/* Non-reflected polynomial 0x1021 with seed 0xFFFF: CRC-16/CCITT-FALSE */
	return crc16_itu_t(0xFFFF, data, len);
}

size_t phoenix6_cobs_encode(const uint8_t *in, size_t len, uint8_t *out)
{
	size_t code_pos = 0;
	size_t out_pos = 1;
	uint8_t code = 1;

	for (size_t i = 0; i < len; i++) {
		if (in[i] == 0U) {
			out[code_pos] = code;
			code_pos = out_pos++;
			code = 1;
		} else {
			out[out_pos++] = in[i];
			code++;
			/* A full block at the end of the data needs no empty block */
			if (code == 0xFF && i + 1U < len) {
				out[code_pos] = code;
				code_pos = out_pos++;
				code = 1;
			}
		}
	}

	out[code_pos] = code;

	return out_pos;
}

int phoenix6_cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_size)
{
	size_t in_pos = 0;
	size_t out_pos = 0;

	while (in_pos < len) {
		uint8_t code = in[in_pos++];

		if (code == 0U || in_pos + code - 1U > len) {
			return -EINVAL;
		}

		for (uint8_t i = 1; i < code; i++) {
			if (out_pos >= out_size) {
				return -EINVAL;
			}
			out[out_pos++] = in[in_pos++];
		}

		if (code != 0xFF && in_pos < len) {
			if (out_pos >= out_size) {
				return -EINVAL;
			}
			out[out_pos++] = 0U;
		}
	}

	return (int)out_pos;
}

int phoenix6_packet_encode(uint8_t type, uint16_t seq, const uint8_t *body, size_t body_len,
			   uint8_t *out, size_t out_size)
{
	uint8_t frame[PHOENIX6_MAX_PACKET];
	size_t frame_len = 3U + body_len + 2U;
	size_t encoded;

	if (frame_len > sizeof(frame) || out_size < frame_len + frame_len / 254U + 2U) {
		return -EMSGSIZE;
	}

	frame[0] = type;
	sys_put_le16(seq, &frame[1]);
	if (body_len > 0U) {
		memcpy(&frame[3], body, body_len);
	}
	sys_put_le16(phoenix6_crc16(frame, 3U + body_len), &frame[3U + body_len]);

	encoded = phoenix6_cobs_encode(frame, frame_len, out);
	out[encoded++] = 0U;

	return (int)encoded;
}

void phoenix6_frame_decoder_init(struct phoenix6_frame_decoder *d)
{
	d->raw_len = 0;
	d->overflow = false;
	d->dropped = 0;
}

bool phoenix6_frame_decoder_feed(struct phoenix6_frame_decoder *d, uint8_t byte,
				 struct phoenix6_packet *pkt)
{
	int len;

	if (byte != 0U) {
		if (d->raw_len < sizeof(d->raw)) {
			d->raw[d->raw_len++] = byte;
		} else {
			d->overflow = true;
		}
		return false;
	}

	if (d->raw_len == 0U) {
		return false;
	}

	if (d->overflow) {
		len = -EINVAL;
	} else {
		len = phoenix6_cobs_decode(d->raw, d->raw_len, d->frame, sizeof(d->frame));
	}

	d->raw_len = 0;
	d->overflow = false;

	if (len < 5 || sys_get_le16(&d->frame[len - 2]) != phoenix6_crc16(d->frame, len - 2)) {
		d->dropped++;
		return false;
	}

	pkt->type = d->frame[0];
	pkt->seq = sys_get_le16(&d->frame[1]);
	pkt->body = &d->frame[3];
	pkt->body_len = (size_t)len - 5U;

	return true;
}

int phoenix6_encode_control_params(struct phoenix6_writer *w,
				   const struct phoenix6_control_desc *desc, const uint8_t *params)
{
	for (uint8_t i = 0; i < desc->param_count; i++) {
		const struct phoenix6_param_desc *p = &desc->params[i];
		const uint8_t *field = params + p->offset;

		switch (p->type) {
		case PHOENIX6_TYPE_FLOAT: {
			double v;

			memcpy(&v, field, sizeof(v));
			phoenix6_put_f64(w, v);
			break;
		}
		case PHOENIX6_TYPE_BOOL: {
			bool v;

			memcpy(&v, field, sizeof(v));
			phoenix6_put_u8(w, v ? 1U : 0U);
			break;
		}
		case PHOENIX6_TYPE_INT:
		case PHOENIX6_TYPE_ENUM: {
			int32_t v;

			memcpy(&v, field, sizeof(v));
			phoenix6_put_i32(w, v);
			break;
		}
		case PHOENIX6_TYPE_CONTROL: {
			const struct phoenix6_control_desc *sub =
				phoenix6_control_desc_get(p->control);
			int err;

			if (sub == NULL) {
				return -EINVAL;
			}

			err = phoenix6_encode_control_params(w, sub, field);
			if (err != 0) {
				return err;
			}
			break;
		}
		default:
			return -EINVAL;
		}
	}

	return w->overflow ? -EMSGSIZE : 0;
}
