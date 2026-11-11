/* gd32.c - Protocollo seriale SoC <-> GD32: vedi gd32.h e docs/protocollo.md. */
#include "gd32.h"

#include <string.h>

uint16_t gd32_checksum(const uint8_t *b, size_t n)
{
	uint16_t ck = 0;
	size_t i;
	for (i = 0; i + 1 < n; i += 2)
		ck += (uint16_t)(b[i] << 8 | b[i + 1]);
	if (n % 2)
		ck ^= b[n - 1];
	return ck;
}

size_t gd32_build_frame(uint8_t cmd, const void *data, size_t n, uint8_t *out, size_t out_size)
{
	if (n > GD32_MAX_DATA || out_size < n + 6)
		return 0;
	out[0] = GD32_HEADER_1;
	out[1] = GD32_HEADER_2;
	out[2] = (uint8_t)(n + 3);
	out[3] = cmd;
	if (n)
		memcpy(out + 4, data, n);
	uint16_t ck = gd32_checksum(out + 3, n + 1);
	out[4 + n] = (uint8_t)(ck >> 8);
	out[5 + n] = (uint8_t)ck;
	return n + 6;
}

const char *gd32_cmd_name(uint8_t cmd)
{
	switch (cmd) {
	case GD32_ERROR_CODE:        return "codice_errore";
	case GD32_SLEEP:                return "sleep";
	case GD32_WAKEUP_ACK:           return "wakeup_ack";
	case GD32_HEARTBEAT:            return "heartbeat";
	case GD32_VERSION:             return "versione";
	case GD32_IMU_ZERO:             return "imu_zero";
	case GD32_RESET_ERRORS:         return "reset_errori";
	case GD32_SOC_WATCHDOG:        return "sorveglia_soc";
	case GD32_KEEPALIVE:            return "keepalive";
	case GD32_SENSORS:              return "sensori";
	case GD32_IMU_RAW:           return "imu_grezza";
	case GD32_MACHINE_FUNCTIONS:    return "funzioni_macchina";
	case GD32_CLIFF_BACKOFF:    return "cliff_retromarcia";
	case GD32_CONTROL_TYPE:       return "tipo_controllo";
	case GD32_VELOCITY:             return "velocita";
	case GD32_WHEEL_SPEED:       return "velocita_ruote";
	case GD32_SUCTION:          return "aspirazione";
	case GD32_SIDE_BRUSH:    return "spazzola_laterale";
	case GD32_MAIN_BRUSH:    return "spazzola_centrale";
	case GD32_WATER_PUMP:          return "pompa_acqua";
	case GD32_LIDAR_MOTOR:         return "motore_lidar";
	case GD32_CLIFF_IR:             return "cliff_ir";
	case GD32_CLIFF_IR_DIRECTION:   return "cliff_ir_direzione";
	case GD32_SELF_CONTROL_PARAMS:   return "parametri_autonomi";
	case GD32_SENSOR_MASK:     return "maschera_sensori";
	case GD32_BUMPER_SMOOTH:       return "bumper_morbido";
	case GD32_SUCTION_MODE:     return "aspirazione_modo";
	case GD32_SUCTION_PWM:      return "aspirazione_pwm";
	case GD32_SUCTION_PWM_BATTERY:  return "aspirazione_pwm_bat";
	case GD32_BUTTON_LED:            return "led_tasto";
	case GD32_LIDAR_POWER:  return "alimentazione_lidar";
	case GD32_SOC_POWER:    return "alimentazione_soc";
	case GD32_SOC_RESTART:          return "riavvia_soc";
	case GD32_CHARGER_POWER:           return "caricatore";
	case GD32_SENSOR_POWER: return "alimentazione_sensori";
	case GD32_IMU_CALIBRATE:          return "imu_calibra";
	case GD32_IMU_CALIBRATION_STATE:    return "imu_stato_calibra";
	case GD32_MAGNETOMETER_CALIBRATE: return "magnetometro_calibra";
	case GD32_MAGNETOMETER_STATE:   return "magnetometro_stato";
	case GD32_DEBUG:                return "debug";
	default:                        return NULL;
	}
}

void gd32_deframer_init(gd32_deframer_t *d)
{
	memset(d, 0, sizeof(*d));
}

/*
 * Il frame in buf non è valido: si scarta il primo byte (l'FA) e tutti gli altri si rimettono in coda per
 * essere rianalizzati, davanti a quelli che erano già in attesa.
 *
 * Perché la coda basta: i byte nuovi si leggono solo a coda vuota, quindi in ogni momento
 * n + (pending_n - pending_i) <= GD32_MAX_FRAME. Un errore sposta n-1 byte da buf alla coda e la somma scende.
 */
static void frame_error(gd32_deframer_t *d)
{
	uint8_t tmp[GD32_MAX_FRAME];
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

/* Aggiunge un byte al frame in costruzione. Ritorna 1 se ha completato un frame valido (in *f). */
static int byte(gd32_deframer_t *d, uint8_t b, gd32_frame_t *f)
{
	if (d->n == 0) {
		if (b == GD32_HEADER_1)
			d->buf[d->n++] = b;
		else
			d->discarded_bytes++;
		return 0;
	}

	d->buf[d->n++] = b;

	if (d->n == 2) {
		if (b != GD32_HEADER_2)
			frame_error(d);      /* FA non seguito da FB: si scarta l'FA e si rianalizza b */
		return 0;
	}
	if (d->n == 3) {
		if (b < 3) {
			d->bad_lengths++;
			frame_error(d);
		}
		return 0;
	}

	size_t len = d->buf[2];
	if (d->n < 3 + len)
		return 0;

	/* frame completo: buf[3] = cmd, poi i dati, poi 2 byte di checksum */
	size_t body_len = len - 2;
	uint16_t expected = (uint16_t)(d->buf[3 + body_len] << 8 | d->buf[4 + body_len]);
	if (gd32_checksum(d->buf + 3, body_len) != expected) {
		d->bad_checksums++;
		frame_error(d);
		return 0;
	}
	f->cmd = d->buf[3];
	f->data_len = (uint8_t)(body_len - 1);
	memcpy(f->data, d->buf + 4, f->data_len);
	d->n = 0;
	d->frames_ok++;
	return 1;
}

size_t gd32_deframer_feed(gd32_deframer_t *d, const uint8_t *b, size_t n, gd32_frame_cb cb, void *ctx)
{
	gd32_frame_t f;
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
		if (byte(d, c, &f)) {
			found++;
			if (cb)
				cb(&f, ctx);
		}
	}
	return found;
}

/* Campi little-endian dentro i dati */
static uint16_t u16(const uint8_t *b) { return (uint16_t)(b[0] | b[1] << 8); }
static int32_t i32(const uint8_t *b) { return (int32_t)((uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24); }

int gd32_read_sensors(const gd32_frame_t *f, gd32_sensors_t *o)
{
	const uint8_t *d = f->data;
	if (f->cmd != GD32_SENSORS || f->data_len < 96)
		return -1;
	memset(o, 0, sizeof(*o));
	o->status = d[0];
	o->bumper_raw = d[1];
	o->bumper_left = (d[1] >> 2 | d[1] >> 3) & 1;
	o->bumper_right = (d[1] >> 1 | d[1]) & 1;
	o->byte2 = d[2];
	for (int i = 0; i < 4; i++)
		o->cliff[i] = (d[3] >> i) & 1;
	o->dustbin = (d[4] >> 2) & 1;
	o->byte4_other = d[4] & 3;
	o->byte5 = d[5];
	o->ir_dock_right = d[6] >> 4;
	o->ir_dock_left = d[6] & 0x0f;
	o->charging = d[7] & 1;
	o->on_base = (d[7] | d[7] >> 1) & 1;
	o->voltage_dv = d[8];
	o->temperature_c = (int8_t)d[9];
	o->encoder_left = i32(d + 16);
	o->encoder_right = i32(d + 24);
	o->v_measured = i32(d + 32);
	o->w_measured = i32(d + 36);
	for (int i = 0; i < 3; i++) {
		o->angular_velocity[i] = (int16_t)u16(d + 40 + 4 * i);
		o->angle[i] = (int16_t)u16(d + 42 + 4 * i);
		o->gravity[i] = (int16_t)u16(d + 52 + 2 * i);
		o->magnetic[i] = (int16_t)u16(d + 90 + 2 * i);
	}
	o->power_button_ms = u16(d + 58) * 10;
	o->spot_button_ms = u16(d + 60) * 10;
	o->home_button_ms = u16(d + 62) * 10;
	o->control = d[64];
	o->led = d[65];
	o->byte66 = d[66] & 0x0f;
	o->byte67 = (int8_t)d[67];
	o->byte68 = d[68];
	o->fan_rpm = d[69] * 100;
	o->water = d[70];
	o->raw72 = u16(d + 72);
	for (int i = 0; i < 6; i++)
		o->currents[i] = u16(d + 74 + 2 * i);
	o->byte88 = (int16_t)u16(d + 88);
	return 0;
}
