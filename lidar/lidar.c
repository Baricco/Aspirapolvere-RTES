/* lidar.c - Protocollo seriale del lidar 3irobotix LDS3: vedi lidar.h e docs/protocollo.md. */
#include "lidar.h"

#include <string.h>

uint16_t lidar_sum(const uint8_t *b, size_t n)
{
	uint16_t s = 0;
	for (size_t i = 0; i < n; i++)
		s += b[i];
	return s;
}

uint16_t lidar_crc16(const uint8_t *b, size_t n)
{
	uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < n; i++) {
		crc ^= b[i];
		for (int k = 0; k < 8; k++)
			crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
	}
	return crc;
}

/* Il checksum che il pacchetto in b (len byte, senza checksum) deve avere, secondo il campo indirizzo. */
static uint16_t expected_checksum(const uint8_t *b, size_t len)
{
	return b[3] == 1 ? lidar_sum(b, len) : lidar_crc16(b, len);
}

size_t lidar_build_packet(uint8_t address, uint8_t type, uint8_t cmd, const void *param, size_t n,
                     uint8_t *out, size_t out_size)
{
	size_t len = 8 + n;
	if (n > LIDAR_MAX_PARAMS || out_size < len + 2)
		return 0;
	out[0] = LIDAR_HEADER;
	out[1] = 0;
	out[2] = (uint8_t)len;
	out[3] = address;
	out[4] = type;
	out[5] = cmd;
	out[6] = (uint8_t)(n >> 8);
	out[7] = (uint8_t)n;
	if (n)
		memcpy(out + 8, param, n);
	uint16_t ck = expected_checksum(out, len);
	out[len] = (uint8_t)(ck >> 8);
	out[len + 1] = (uint8_t)ck;
	return len + 2;
}

const char *lidar_cmd_name(uint8_t cmd)
{
	switch (cmd) {
	case LIDAR_SCAN:       return "scansione";
	case LIDAR_HEALTH:          return "salute";
	case LIDAR_VERSION:        return "versione";
	case LIDAR_SCAN_NEW: return "scansione_nuova";
	case LIDAR_SPEED:        return "velocita";
	default:                    return NULL;
	}
}

void lidar_deframer_init(lidar_deframer_t *d)
{
	memset(d, 0, sizeof(*d));
}

/* Come in gd32.c: si scarta il primo byte e si rianalizzano gli altri. */
static void frame_error(lidar_deframer_t *d)
{
	uint8_t tmp[LIDAR_MAX_PACKET];
	size_t rest = d->pending_n - d->pending_i;
	size_t k = d->n - 1;

	memcpy(tmp, d->buf + 1, k);
	memcpy(tmp + k, d->pending + d->pending_i, rest);
	memcpy(d->pending, tmp, k + rest);
	d->pending_n = k + rest;
	d->pending_i = 0;
	d->n = 0;
	d->discarded_bytes++;
}

static int byte(lidar_deframer_t *d, uint8_t b, lidar_packet_t *p)
{
	if (d->n == 0) {
		if (b == LIDAR_HEADER)
			d->buf[d->n++] = b;
		else
			d->discarded_bytes++;
		return 0;
	}

	d->buf[d->n++] = b;

	if (d->n == 2) {
		if (b != 0) {
			d->bad_lengths++;
			frame_error(d);
		}
		return 0;
	}
	if (d->n == 3) {
		if (b < LIDAR_LEN_MIN || b > LIDAR_LEN_MAX) {
			d->bad_lengths++;
			frame_error(d);
		}
		return 0;
	}

	size_t len = d->buf[2];
	if (d->n < len + 2)
		return 0;

	uint16_t received = (uint16_t)(d->buf[len] << 8 | d->buf[len + 1]);
	if (expected_checksum(d->buf, len) != received) {
		d->bad_checksums++;
		frame_error(d);
		return 0;
	}
	p->address = d->buf[3];
	p->type = d->buf[4];
	p->cmd = d->buf[5];
	p->n_declared = len >= 8 ? (uint16_t)(d->buf[6] << 8 | d->buf[7]) : 0;
	p->n_param = (uint16_t)(len > 8 ? len - 8 : 0);
	memcpy(p->param, d->buf + 8, p->n_param);
	d->n = 0;
	d->packets_ok++;
	return 1;
}

size_t lidar_deframer_feed(lidar_deframer_t *d, const uint8_t *b, size_t n, lidar_packet_cb cb, void *ctx)
{
	lidar_packet_t p;
	size_t found = 0, i = 0;

	for (;;) {
		uint8_t c;
		if (d->pending_i < d->pending_n)
			c = d->pending[d->pending_i++];
		else if (i < n)
			c = b[i++];
		else
			break;
		if (d->pending_i == d->pending_n)
			d->pending_i = d->pending_n = 0;
		if (byte(d, c, &p)) {
			found++;
			if (cb)
				cb(&p, ctx);
		}
	}
	return found;
}
