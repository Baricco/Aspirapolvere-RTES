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

size_t gd32_componi(uint8_t cmd, const void *dati, size_t n, uint8_t *out, size_t out_dim)
{
	if (n > GD32_MAX_DATI || out_dim < n + 6)
		return 0;
	out[0] = GD32_INTESTAZIONE_1;
	out[1] = GD32_INTESTAZIONE_2;
	out[2] = (uint8_t)(n + 3);
	out[3] = cmd;
	if (n)
		memcpy(out + 4, dati, n);
	uint16_t ck = gd32_checksum(out + 3, n + 1);
	out[4 + n] = (uint8_t)(ck >> 8);
	out[5 + n] = (uint8_t)ck;
	return n + 6;
}

const char *gd32_nome_cmd(uint8_t cmd)
{
	switch (cmd) {
	case GD32_CODICE_ERRORE:        return "codice_errore";
	case GD32_SLEEP:                return "sleep";
	case GD32_WAKEUP_ACK:           return "wakeup_ack";
	case GD32_HEARTBEAT:            return "heartbeat";
	case GD32_VERSIONE:             return "versione";
	case GD32_IMU_ZERO:             return "imu_zero";
	case GD32_RESET_ERRORI:         return "reset_errori";
	case GD32_SORVEGLIA_SOC:        return "sorveglia_soc";
	case GD32_KEEPALIVE:            return "keepalive";
	case GD32_SENSORI:              return "sensori";
	case GD32_IMU_GREZZA:           return "imu_grezza";
	case GD32_FUNZIONI_MACCHINA:    return "funzioni_macchina";
	case GD32_CLIFF_RETROMARCIA:    return "cliff_retromarcia";
	case GD32_TIPO_CONTROLLO:       return "tipo_controllo";
	case GD32_VELOCITA:             return "velocita";
	case GD32_VELOCITA_RUOTE:       return "velocita_ruote";
	case GD32_ASPIRAZIONE:          return "aspirazione";
	case GD32_SPAZZOLA_LATERALE:    return "spazzola_laterale";
	case GD32_SPAZZOLA_CENTRALE:    return "spazzola_centrale";
	case GD32_POMPA_ACQUA:          return "pompa_acqua";
	case GD32_MOTORE_LIDAR:         return "motore_lidar";
	case GD32_CLIFF_IR:             return "cliff_ir";
	case GD32_CLIFF_IR_DIREZIONE:   return "cliff_ir_direzione";
	case GD32_PARAMETRI_AUTONOMI:   return "parametri_autonomi";
	case GD32_MASCHERA_SENSORI:     return "maschera_sensori";
	case GD32_BUMPER_MORBIDO:       return "bumper_morbido";
	case GD32_ASPIRAZIONE_MODO:     return "aspirazione_modo";
	case GD32_ASPIRAZIONE_PWM:      return "aspirazione_pwm";
	case GD32_ASPIRAZIONE_PWM_BAT:  return "aspirazione_pwm_bat";
	case GD32_LED_TASTO:            return "led_tasto";
	case GD32_ALIMENTAZIONE_LIDAR:  return "alimentazione_lidar";
	case GD32_ALIMENTAZIONE_SOC:    return "alimentazione_soc";
	case GD32_RIAVVIA_SOC:          return "riavvia_soc";
	case GD32_CARICATORE:           return "caricatore";
	case GD32_ALIMENTAZIONE_SENSORI: return "alimentazione_sensori";
	case GD32_IMU_CALIBRA:          return "imu_calibra";
	case GD32_IMU_STATO_CALIBRA:    return "imu_stato_calibra";
	case GD32_MAGNETOMETRO_CALIBRA: return "magnetometro_calibra";
	case GD32_MAGNETOMETRO_STATO:   return "magnetometro_stato";
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
 * n + (attesa_n - attesa_i) <= GD32_MAX_FRAME. Un errore sposta n-1 byte da buf alla coda e la somma scende.
 */
static void errore(gd32_deframer_t *d)
{
	uint8_t tmp[GD32_MAX_FRAME];
	size_t resto = d->attesa_n - d->attesa_i;
	size_t k = d->n - 1;

	memcpy(tmp, d->buf + 1, k);
	memcpy(tmp + k, d->attesa + d->attesa_i, resto);
	memcpy(d->attesa, tmp, k + resto);
	d->attesa_n = k + resto;
	d->attesa_i = 0;
	d->n = 0;
	d->byte_scartati++;
}

/* Aggiunge un byte al frame in costruzione. Ritorna 1 se ha completato un frame valido (in *f). */
static int byte(gd32_deframer_t *d, uint8_t b, gd32_frame_t *f)
{
	if (d->n == 0) {
		if (b == GD32_INTESTAZIONE_1)
			d->buf[d->n++] = b;
		else
			d->byte_scartati++;
		return 0;
	}

	d->buf[d->n++] = b;

	if (d->n == 2) {
		if (b != GD32_INTESTAZIONE_2)
			errore(d);      /* FA non seguito da FB: si scarta l'FA e si rianalizza b */
		return 0;
	}
	if (d->n == 3) {
		if (b < 3) {
			d->len_errati++;
			errore(d);
		}
		return 0;
	}

	size_t len = d->buf[2];
	if (d->n < 3 + len)
		return 0;

	/* frame completo: buf[3] = cmd, poi i dati, poi 2 byte di checksum */
	size_t n_corpo = len - 2;
	uint16_t atteso = (uint16_t)(d->buf[3 + n_corpo] << 8 | d->buf[4 + n_corpo]);
	if (gd32_checksum(d->buf + 3, n_corpo) != atteso) {
		d->checksum_errati++;
		errore(d);
		return 0;
	}
	f->cmd = d->buf[3];
	f->n_dati = (uint8_t)(n_corpo - 1);
	memcpy(f->dati, d->buf + 4, f->n_dati);
	d->n = 0;
	d->frame_ok++;
	return 1;
}

size_t gd32_deframer_scrivi(gd32_deframer_t *d, const uint8_t *b, size_t n, gd32_frame_cb cb, void *ctx)
{
	gd32_frame_t f;
	size_t trovati = 0, i = 0;

	for (;;) {
		uint8_t c;
		if (d->attesa_i < d->attesa_n)
			c = d->attesa[d->attesa_i++];
		else if (i < n)
			c = b[i++];
		else
			break;
		if (d->attesa_i == d->attesa_n)
			d->attesa_i = d->attesa_n = 0;
		if (byte(d, c, &f)) {
			trovati++;
			if (cb)
				cb(&f, ctx);
		}
	}
	return trovati;
}

/* Campi little-endian dentro i dati */
static uint16_t u16(const uint8_t *b) { return (uint16_t)(b[0] | b[1] << 8); }
static int32_t i32(const uint8_t *b) { return (int32_t)((uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24); }

int gd32_leggi_sensori(const gd32_frame_t *f, gd32_sensori_t *o)
{
	const uint8_t *d = f->dati;
	if (f->cmd != GD32_SENSORI || f->n_dati < 96)
		return -1;
	memset(o, 0, sizeof(*o));
	o->stato = d[0];
	o->bumper_bit = d[1];
	o->bumper_sx = (d[1] >> 2 | d[1] >> 3) & 1;
	o->bumper_dx = (d[1] >> 1 | d[1]) & 1;
	o->byte2 = d[2];
	for (int i = 0; i < 4; i++)
		o->cliff[i] = (d[3] >> i) & 1;
	o->cassetto = (d[4] >> 2) & 1;
	o->byte4_altri = d[4] & 3;
	o->byte5 = d[5];
	o->ir_base_a = d[6] >> 4;
	o->ir_base_b = d[6] & 0x0f;
	o->in_carica = d[7] & 1;
	o->sulla_base = (d[7] | d[7] >> 1) & 1;
	o->tensione_dv = d[8];
	o->temperatura_c = (int8_t)d[9];
	o->encoder_sx = i32(d + 16);
	o->encoder_dx = i32(d + 24);
	o->vel_sx = i32(d + 32);
	o->vel_dx = i32(d + 36);
	for (int i = 0; i < 3; i++) {
		o->vel_angolare[i] = (int16_t)u16(d + 40 + 4 * i);
		o->angolo[i] = (int16_t)u16(d + 42 + 4 * i);
		o->gravita[i] = (int16_t)u16(d + 52 + 2 * i);
		o->magnetico[i] = (int16_t)u16(d + 90 + 2 * i);
	}
	o->tasto_accensione_ms = u16(d + 58) * 10;
	o->tasto_punto_ms = u16(d + 60) * 10;
	o->tasto_casa_ms = u16(d + 62) * 10;
	o->controllo = d[64];
	o->led = d[65];
	o->byte66 = d[66] & 0x0f;
	o->byte67 = (int8_t)d[67];
	o->byte68 = d[68];
	o->byte69_x100 = d[69] * 100;
	o->acqua = d[70];
	o->grezzo72 = u16(d + 72);
	for (int i = 0; i < 6; i++)
		o->correnti[i] = u16(d + 74 + 2 * i);
	o->byte88 = (int16_t)u16(d + 88);
	return 0;
}
