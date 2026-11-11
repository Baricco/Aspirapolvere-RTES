/* Test della descrizione dei messaggi (gd32/description.h): i campi devono coincidere con il decodificatore
 * gd32_read_sensors, non sovrapporsi e stare dentro i 96 byte. */
#include <string.h>

#include "test.h"
#include "../gd32/description.h"
#include "../gd32/gd32.h"

static const field_t *campo(const char *name)
{
	for (int i = 0; i < N_SENSOR_FIELDS; i++)
		if (!strcmp(SENSOR_FIELDS[i].name, name))
			return &SENSOR_FIELDS[i];
	return NULL;
}

static void dentro_e_senza_sovrapposizioni(void)
{
	uint8_t usato[96][8];
	memset(usato, 0, sizeof usato);
	int ok = 1;
	for (int i = 0; i < N_SENSOR_FIELDS; i++) {
		const field_t *c = &SENSOR_FIELDS[i];
		CONTROLLA(c->byte + c->n_bytes <= 96);
		CONTROLLA(c->reliability == 'V' || c->reliability == 'C' || c->reliability == '?');
		for (int b = c->byte; b < c->byte + c->n_bytes && b < 96; b++)
			for (int k = 0; k < 8; k++) {
				int mio = c->bit < 0 || (b == c->byte && k >= c->bit && k < c->bit + c->n_bits);
				if (!mio)
					continue;
				if (usato[b][k]) {
					printf("  sovrapposti al byte %d bit %d: %s\n", b, k, c->name);
					ok = 0;
				}
				usato[b][k] = 1;
			}
	}
	CONTROLLA(ok);
}

static void uguale_al_decodificatore(void)
{
	gd32_frame_t f = { .cmd = GD32_SENSORS, .data_len = 96 };
	for (int i = 0; i < 96; i++)
		f.data[i] = (uint8_t)(i * 37 + 11);
	gd32_sensors_t s;
	CONTROLLA(gd32_read_sensors(&f, &s) == 0);
	const uint8_t *d = f.data;
	CONTROLLA(description_raw_value(campo("encoder_sx"), d) == s.encoder_left);
	CONTROLLA(description_raw_value(campo("encoder_dx"), d) == s.encoder_right);
	CONTROLLA(description_raw_value(campo("v_misurata"), d) == s.v_measured);
	CONTROLLA(description_raw_value(campo("w_misurata"), d) == s.w_measured);
	CONTROLLA(description_raw_value(campo("tensione"), d) == s.voltage_dv);
	CONTROLLA(description_raw_value(campo("temperatura"), d) == s.temperature_c);
	CONTROLLA(description_raw_value(campo("vel_imbardata"), d) == s.angular_velocity[0]);
	CONTROLLA(description_raw_value(campo("angolo_imbardata"), d) == s.angle[0]);
	CONTROLLA(description_raw_value(campo("angolo_rollio"), d) == s.angle[2]);
	CONTROLLA(description_raw_value(campo("gravita_z"), d) == s.gravity[2]);
	CONTROLLA(description_raw_value(campo("tasto_casa"), d) * 10 == s.home_button_ms);
	CONTROLLA(description_raw_value(campo("led"), d) == s.led);
	CONTROLLA(description_raw_value(campo("corrente_ruota_dx"), d) == s.currents[1]);
	CONTROLLA(description_raw_value(campo("corrente_aspirazione"), d) == s.currents[4]);
	CONTROLLA(description_raw_value(campo("eco_laterale"), d) == s.byte67);
	CONTROLLA(description_raw_value(campo("ventola"), d) * 100 == s.fan_rpm);
	CONTROLLA(description_raw_value(campo("copia_vel_beccheggio"), d) == s.magnetic[1]);
	/* campi a bit */
	CONTROLLA(description_raw_value(campo("cassetto"), d) == s.dustbin);
	CONTROLLA(description_raw_value(campo("in_carica"), d) == s.charging);
	CONTROLLA(description_raw_value(campo("ir_base_a"), d) == s.ir_dock_right);
	CONTROLLA(description_raw_value(campo("ir_base_b"), d) == s.ir_dock_left);
	CONTROLLA((description_raw_value(campo("bumper_sx"), d) != 0) == s.bumper_left);
	CONTROLLA((description_raw_value(campo("bumper_dx"), d) != 0) == s.bumper_right);
	CONTROLLA(description_raw_value(campo("cliff_sx_laterale"), d) == s.cliff[0]);
	CONTROLLA(description_raw_value(campo("cliff_dx_laterale"), d) == s.cliff[3]);
	CONTROLLA(description_raw_value(campo("sensori_accesi"), d) == ((s.control >> 5) & 1));
}

static void messaggi(void)
{
	CONTROLLA(description_find_message(0x15, 0) && description_find_message(0x15, 0)->n_fields == N_SENSOR_FIELDS);
	const message_t *v = description_find_message(0x66, 1);
	CONTROLLA(v && v->n_fields == 2);
	uint8_t d[8] = { 0x78, 0x06, 0, 0, 0x90, 0x01, 0, 0 };     /* 1656, 400 */
	CONTROLLA(description_raw_value(&v->fields[0], d) == 1656 && description_raw_value(&v->fields[1], d) == 400);
	CONTROLLA(description_find_message(0x66, 0) == NULL);
	for (int i = 0; i < N_MESSAGES; i++)
		CONTROLLA(MESSAGES[i].name && MESSAGES[i].explanation);
}

int main(void)
{
	ESEGUI(dentro_e_senza_sovrapposizioni);
	ESEGUI(uguale_al_decodificatore);
	ESEGUI(messaggi);
	return TEST_FINE();
}
