/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wire protocol of the Phoenix 6 bridge: framing and value encoding.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "phoenix6_proto.h"

ZTEST(phoenix6_proto, test_crc16_check_value)
{
	/* Catalogue check value of CRC-16/CCITT-FALSE */
	zassert_equal(phoenix6_crc16((const uint8_t *)"123456789", 9), 0x29B1);
}

struct cobs_vector {
	const uint8_t *in;
	size_t in_len;
	const uint8_t *out;
	size_t out_len;
};

#define VEC(i, o)                                                                                  \
	{                                                                                          \
		(const uint8_t *)(i), sizeof(i) - 1, (const uint8_t *)(o), sizeof(o) - 1           \
	}

static void check_cobs(const uint8_t *in, size_t in_len, const uint8_t *expected,
		       size_t expected_len)
{
	uint8_t enc[300];
	uint8_t dec[300];
	size_t n;
	int m;

	n = phoenix6_cobs_encode(in, in_len, enc);
	zassert_equal(n, expected_len, "encoded length %zu, expected %zu", n, expected_len);
	zassert_mem_equal(enc, expected, expected_len);
	zassert_true(n <= in_len + in_len / 254U + 1U);

	m = phoenix6_cobs_decode(enc, n, dec, sizeof(dec));
	zassert_equal(m, (int)in_len);
	zassert_mem_equal(dec, in, in_len);
}

ZTEST(phoenix6_proto, test_cobs_short_vectors)
{
	/* Reference vectors of the COBS article on Wikipedia */
	static const struct cobs_vector vectors[] = {
		VEC("", "\x01"),
		VEC("\x00", "\x01\x01"),
		VEC("\x00\x00", "\x01\x01\x01"),
		VEC("\x00\x11\x00", "\x01\x02\x11\x01"),
		VEC("\x11\x22\x00\x33", "\x03\x11\x22\x02\x33"),
		VEC("\x11\x22\x33\x44", "\x05\x11\x22\x33\x44"),
		VEC("\x11\x00\x00\x00", "\x02\x11\x01\x01\x01"),
	};

	ARRAY_FOR_EACH_PTR(vectors, v) {
		check_cobs(v->in, v->in_len, v->out, v->out_len);
	}
}

ZTEST(phoenix6_proto, test_cobs_long_vectors)
{
	uint8_t in[255];
	uint8_t out[258];

	/* 01 02 ... FE -> FF 01 02 ... FE */
	for (int i = 0; i < 254; i++) {
		in[i] = i + 1;
	}
	out[0] = 0xFF;
	memcpy(&out[1], in, 254);
	check_cobs(in, 254, out, 255);

	/* 00 01 02 ... FE -> 01 FF 01 02 ... FE */
	in[0] = 0x00;
	for (int i = 1; i < 255; i++) {
		in[i] = i;
	}
	out[0] = 0x01;
	out[1] = 0xFF;
	memcpy(&out[2], &in[1], 254);
	check_cobs(in, 255, out, 256);

	/* 01 02 ... FE FF -> FF 01 02 ... FE 02 FF */
	for (int i = 0; i < 255; i++) {
		in[i] = i + 1;
	}
	out[0] = 0xFF;
	memcpy(&out[1], in, 254);
	out[255] = 0x02;
	out[256] = 0xFF;
	check_cobs(in, 255, out, 257);

	/* 02 03 ... FF 00 -> FF 02 03 ... FF 01 01 */
	for (int i = 0; i < 254; i++) {
		in[i] = i + 2;
	}
	in[254] = 0x00;
	out[0] = 0xFF;
	memcpy(&out[1], in, 254);
	out[255] = 0x01;
	out[256] = 0x01;
	check_cobs(in, 255, out, 257);

	/* 03 04 ... FF 00 01 -> FE 03 04 ... FF 02 01 */
	for (int i = 0; i < 253; i++) {
		in[i] = i + 3;
	}
	in[253] = 0x00;
	in[254] = 0x01;
	out[0] = 0xFE;
	memcpy(&out[1], in, 253);
	out[254] = 0x02;
	out[255] = 0x01;
	check_cobs(in, 255, out, 256);
}

ZTEST(phoenix6_proto, test_cobs_decode_invalid)
{
	uint8_t out[16];

	/* Zero code byte */
	zassert_equal(phoenix6_cobs_decode((const uint8_t *)"\x02\x11\x00", 3, out, sizeof(out)),
		      -EINVAL);
	/* Block runs past the end */
	zassert_equal(phoenix6_cobs_decode((const uint8_t *)"\x05\x11\x22", 3, out, sizeof(out)),
		      -EINVAL);
	/* Output too small */
	zassert_equal(phoenix6_cobs_decode((const uint8_t *)"\x05\x11\x22\x33\x44", 5, out, 3),
		      -EINVAL);
}

/* Reference packets, identical to the Python implementation's test vectors */
static const uint8_t hello_packet[] = {0x0b, 0x01, 0x02, 0x01, 0x01, 0xb1,
				       0x31, 0x86, 0xca, 0x37, 0x45, 0x00};
static const uint8_t ping_packet[] = {0x02, 0x02, 0x01, 0x03, 0xfc, 0xa2, 0x00};

ZTEST(phoenix6_proto, test_packet_encode_reference)
{
	uint8_t body[5];
	uint8_t out[PHOENIX6_MAX_ENCODED];
	struct phoenix6_writer w;
	int n;

	phoenix6_writer_init(&w, body, sizeof(body));
	phoenix6_put_u8(&w, 1);
	phoenix6_put_u32(&w, 0xca8631b1);
	zassert_false(w.overflow);

	n = phoenix6_packet_encode(PHOENIX6_MSG_HELLO, 0x0102, body, w.len, out, sizeof(out));
	zassert_equal(n, sizeof(hello_packet));
	zassert_mem_equal(out, hello_packet, sizeof(hello_packet));

	n = phoenix6_packet_encode(PHOENIX6_MSG_PING, 0, NULL, 0, out, sizeof(out));
	zassert_equal(n, sizeof(ping_packet));
	zassert_mem_equal(out, ping_packet, sizeof(ping_packet));
}

ZTEST(phoenix6_proto, test_packet_encode_limits)
{
	static uint8_t body[PHOENIX6_MAX_PACKET];
	static uint8_t out[PHOENIX6_MAX_ENCODED];

	/* Largest body: 512 - 3 (type, seq) - 2 (CRC) */
	zassert_true(phoenix6_packet_encode(PHOENIX6_MSG_LOG, 0, body, PHOENIX6_MAX_PACKET - 5, out,
					    sizeof(out)) > 0);
	zassert_equal(phoenix6_packet_encode(PHOENIX6_MSG_LOG, 0, body, PHOENIX6_MAX_PACKET - 4,
					     out, sizeof(out)),
		      -EMSGSIZE);
	zassert_equal(phoenix6_packet_encode(PHOENIX6_MSG_LOG, 0, body, 10, out, 12), -EMSGSIZE);
}

static int feed_all(struct phoenix6_frame_decoder *d, const uint8_t *data, size_t len,
		    struct phoenix6_packet *pkt)
{
	int packets = 0;

	for (size_t i = 0; i < len; i++) {
		if (phoenix6_frame_decoder_feed(d, data[i], pkt)) {
			packets++;
		}
	}

	return packets;
}

ZTEST(phoenix6_proto, test_frame_decoder)
{
	static struct phoenix6_frame_decoder d;
	static uint8_t big[PHOENIX6_MAX_PACKET];
	static uint8_t enc[PHOENIX6_MAX_ENCODED];
	struct phoenix6_packet pkt;
	uint8_t corrupt[sizeof(hello_packet)];
	int n;

	phoenix6_frame_decoder_init(&d);

	zassert_equal(feed_all(&d, hello_packet, sizeof(hello_packet), &pkt), 1);
	zassert_equal(pkt.type, PHOENIX6_MSG_HELLO);
	zassert_equal(pkt.seq, 0x0102);
	zassert_equal(pkt.body_len, 5);
	zassert_mem_equal(pkt.body, "\x01\xb1\x31\x86\xca", 5);

	/* Corrupted packet is dropped, the next one is received */
	memcpy(corrupt, hello_packet, sizeof(corrupt));
	corrupt[4] ^= 0x10;
	zassert_equal(feed_all(&d, corrupt, sizeof(corrupt), &pkt), 0);
	zassert_equal(d.dropped, 1);
	zassert_equal(feed_all(&d, ping_packet, sizeof(ping_packet), &pkt), 1);
	zassert_equal(pkt.type, PHOENIX6_MSG_PING);
	zassert_equal(pkt.body_len, 0);

	/* Garbage without delimiter before a packet is discarded with the packet */
	zassert_equal(feed_all(&d, (const uint8_t *)"\x05\x01", 2, &pkt), 0);
	zassert_equal(feed_all(&d, ping_packet, sizeof(ping_packet), &pkt), 0);
	zassert_equal(d.dropped, 2);

	/* Empty frames between delimiters are ignored */
	zassert_equal(feed_all(&d, (const uint8_t *)"\x00\x00", 2, &pkt), 0);
	zassert_equal(d.dropped, 2);

	/* Largest packet */
	for (size_t i = 0; i < sizeof(big); i++) {
		big[i] = (uint8_t)i;
	}
	n = phoenix6_packet_encode(PHOENIX6_MSG_LOG, 7, big, PHOENIX6_MAX_PACKET - 5, enc,
				   sizeof(enc));
	zassert_true(n > 0);
	zassert_equal(feed_all(&d, enc, n, &pkt), 1);
	zassert_equal(pkt.body_len, PHOENIX6_MAX_PACKET - 5);
	zassert_mem_equal(pkt.body, big, PHOENIX6_MAX_PACKET - 5);

	/* Overlong frame is dropped */
	memset(enc, 0x01, sizeof(enc));
	zassert_equal(feed_all(&d, enc, sizeof(enc), &pkt), 0);
	zassert_equal(feed_all(&d, enc, sizeof(enc), &pkt), 0);
	zassert_true(d.overflow);
	zassert_equal(feed_all(&d, (const uint8_t *)"\x00", 1, &pkt), 0);
	zassert_equal(d.dropped, 3);
	zassert_equal(feed_all(&d, ping_packet, sizeof(ping_packet), &pkt), 1);
}

ZTEST(phoenix6_proto, test_writer_reader)
{
	uint8_t buf[27];
	struct phoenix6_writer w;
	struct phoenix6_reader r;

	phoenix6_writer_init(&w, buf, sizeof(buf));
	phoenix6_put_u8(&w, 0xA5);
	phoenix6_put_u16(&w, 0x1234);
	phoenix6_put_u32(&w, 0xDEADBEEF);
	phoenix6_put_i32(&w, -42);
	phoenix6_put_f32(&w, 1.5f);
	phoenix6_put_f64(&w, -2.25);
	zassert_false(w.overflow);
	zassert_equal(w.len, 23);
	zassert_mem_equal(buf, "\xA5\x34\x12\xEF\xBE\xAD\xDE\xD6\xFF\xFF\xFF", 11);
	/* 1.5f = 0x3FC00000, -2.25 = 0xC002000000000000 */
	zassert_mem_equal(&buf[11], "\x00\x00\xC0\x3F", 4);
	zassert_mem_equal(&buf[15], "\x00\x00\x00\x00\x00\x00\x02\xC0", 8);

	phoenix6_put_f64(&w, 1.0);
	zassert_true(w.overflow);
	zassert_equal(w.len, 23);

	phoenix6_reader_init(&r, buf, 23);
	zassert_equal(phoenix6_get_u8(&r), 0xA5);
	zassert_equal(phoenix6_get_u16(&r), 0x1234);
	zassert_equal(phoenix6_get_u32(&r), 0xDEADBEEF);
	zassert_equal(phoenix6_get_i32(&r), -42);
	zassert_equal(phoenix6_get_f32(&r), 1.5f);
	zassert_equal(phoenix6_get_f64(&r), -2.25);
	zassert_false(r.error);
	zassert_equal(phoenix6_get_u8(&r), 0);
	zassert_true(r.error);
}

ZTEST(phoenix6_proto, test_encode_control_params)
{
	struct phoenix6_control req = PHOENIX6_VELOCITY_VOLTAGE(10.0);
	uint8_t buf[128];
	struct phoenix6_writer w;
	struct phoenix6_reader r;

	req.velocity_voltage.slot = 1;
	req.velocity_voltage.feed_forward = 0.5;
	req.velocity_voltage.enable_foc = false;

	phoenix6_writer_init(&w, buf, sizeof(buf));
	zassert_ok(phoenix6_encode_control_params(&w, phoenix6_control_desc_get(req.id),
						  (const uint8_t *)&req +
							  PHOENIX6_CONTROL_PARAMS_OFFSET));

	/* Phoenix 6 parameter order */
	phoenix6_reader_init(&r, buf, w.len);
	zassert_equal(phoenix6_get_f64(&r), 10.0); /* velocity */
	zassert_equal(phoenix6_get_f64(&r), 0.0);  /* acceleration */
	zassert_equal(phoenix6_get_u8(&r), 0);     /* enable_foc */
	zassert_equal(phoenix6_get_f64(&r), 0.5);  /* feed_forward */
	zassert_equal(phoenix6_get_i32(&r), 1);    /* slot */
	for (int i = 0; i < 6; i++) {
		/* override_brake_dur_neutral ... use_timesync */
		zassert_equal(phoenix6_get_u8(&r), 0);
	}
	zassert_false(r.error);
	zassert_equal(r.pos, r.len);

	/* Too small buffer */
	phoenix6_writer_init(&w, buf, 20);
	zassert_equal(phoenix6_encode_control_params(&w, phoenix6_control_desc_get(req.id),
						     (const uint8_t *)&req +
							     PHOENIX6_CONTROL_PARAMS_OFFSET),
		      -EMSGSIZE);
}

ZTEST(phoenix6_proto, test_encode_nested_control)
{
	struct phoenix6_control req = PHOENIX6_DIFF_VELOCITY_VOLTAGE_POSITION(
		PHOENIX6_VELOCITY_VOLTAGE_PARAMS(5.0), PHOENIX6_POSITION_VOLTAGE_PARAMS(0.25));
	const struct phoenix6_control_desc *vv =
		phoenix6_control_desc_get(PHOENIX6_CONTROL_ID_VELOCITY_VOLTAGE);
	const struct phoenix6_control_desc *pv =
		phoenix6_control_desc_get(PHOENIX6_CONTROL_ID_POSITION_VOLTAGE);
	uint8_t nested[256];
	uint8_t flat[256];
	struct phoenix6_writer w;
	struct phoenix6_writer wf;
	struct phoenix6_control avg = PHOENIX6_VELOCITY_VOLTAGE(5.0);
	struct phoenix6_control diff = PHOENIX6_POSITION_VOLTAGE(0.25);

	phoenix6_writer_init(&w, nested, sizeof(nested));
	zassert_ok(phoenix6_encode_control_params(&w, phoenix6_control_desc_get(req.id),
						  (const uint8_t *)&req +
							  PHOENIX6_CONTROL_PARAMS_OFFSET));

	/* Nested requests are encoded as their parameters, in order */
	phoenix6_writer_init(&wf, flat, sizeof(flat));
	zassert_ok(phoenix6_encode_control_params(
		&wf, vv, (const uint8_t *)&avg + PHOENIX6_CONTROL_PARAMS_OFFSET));
	zassert_ok(phoenix6_encode_control_params(
		&wf, pv, (const uint8_t *)&diff + PHOENIX6_CONTROL_PARAMS_OFFSET));

	zassert_equal(w.len, wf.len);
	zassert_mem_equal(nested, flat, w.len);
}

ZTEST_SUITE(phoenix6_proto, NULL, NULL, NULL, NULL, NULL);
