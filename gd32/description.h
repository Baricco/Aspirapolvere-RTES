/*
 * description.h - Descrizione leggibile dei messaggi fra SoC e GD32: ogni campo del frame dei sensori 0x15 e dei
 * comandi, con posizione, tipo, scala, unità, affidabilità e spiegazione.
 *
 * È la fonte unica per l'ispettore della pagina del simulatore (che la riceve da GET /campi) e per chi legge i dati
 * a mano. test/test_descrizione.c controlla che le posizioni coincidano con gd32_read_sensors.
 * Affidabilità: 'V' verificato sul robot, 'C' letto nel codice di everest-server / AuxCtrl, '?' ipotesi.
 */
#ifndef GD32_DESCRIPTION_H
#define GD32_DESCRIPTION_H

#include <stdint.h>

/* Gruppi (per i colori dell'ispettore) */
enum {
	FIELD_STATUS = 0, FIELD_CONTACTS, FIELD_BASE, FIELD_ODOMETRY, FIELD_IMU, FIELD_BUTTONS, FIELD_MOTORS, FIELD_UNKNOWN,
};

typedef struct {
	const char *name;
	uint8_t byte, n_bytes;        /* posizione nei dati (dopo il codice cmd) e lunghezza */
	int8_t bit;                  /* -1 = campo numerico; altrimenti primo bit dentro il byte */
	uint8_t n_bits;               /* per i campi a bit */
	const char *type;            /* u8 i8 u16 i16 i32 bit */
	double scale;                /* valore = grezzo x scala */
	const char *unit;
	char reliability;           /* 'V' 'C' '?' */
	uint8_t group;
	const char *explanation;
} field_t;

typedef struct {
	uint8_t cmd;
	const char *name;
	uint8_t to_gd32;          /* 1 = dal SoC al GD32, 0 = dal GD32 al SoC */
	const char *explanation;
	const field_t *fields;
	uint8_t n_fields;
} message_t;

extern const field_t SENSOR_FIELDS[];
extern const int N_SENSOR_FIELDS;

extern const message_t MESSAGES[];
extern const int N_MESSAGES;

/* Descrizione di un messaggio (verso: 1 dal SoC, 0 dal GD32), o NULL. */
const message_t *description_find_message(uint8_t cmd, int to_gd32);

/* Valore grezzo di un campo letto dai dati (per i campi a bit: i bit, già spostati). */
int64_t description_raw_value(const field_t *c, const uint8_t *data);

#endif /* GD32_DESCRIPTION_H */
