/*
 * descrizione.h - Descrizione leggibile dei messaggi fra SoC e GD32: ogni campo del frame dei sensori 0x15 e dei
 * comandi, con posizione, tipo, scala, unità, affidabilità e spiegazione.
 *
 * È la fonte unica per l'ispettore della pagina del simulatore (che la riceve da GET /campi) e per chi legge i dati
 * a mano. test/test_descrizione.c controlla che le posizioni coincidano con gd32_leggi_sensori.
 * Affidabilità: 'V' verificato sul robot, 'C' letto nel codice di everest-server / AuxCtrl, '?' ipotesi.
 */
#ifndef GD32_DESCRIZIONE_H
#define GD32_DESCRIZIONE_H

#include <stdint.h>

/* Gruppi (per i colori dell'ispettore) */
enum {
	CAMPO_STATO = 0, CAMPO_URTI, CAMPO_BASE, CAMPO_ODOMETRIA, CAMPO_IMU, CAMPO_TASTI, CAMPO_MOTORI, CAMPO_IGNOTO,
};

typedef struct {
	const char *nome;
	uint8_t byte, n_byte;        /* posizione nei dati (dopo il codice cmd) e lunghezza */
	int8_t bit;                  /* -1 = campo numerico; altrimenti primo bit dentro il byte */
	uint8_t n_bit;               /* per i campi a bit */
	const char *tipo;            /* u8 i8 u16 i16 i32 bit */
	double scala;                /* valore = grezzo x scala */
	const char *unita;
	char affidabilita;           /* 'V' 'C' '?' */
	uint8_t gruppo;
	const char *spiegazione;
} campo_t;

typedef struct {
	uint8_t cmd;
	const char *nome;
	uint8_t verso_gd32;          /* 1 = dal SoC al GD32, 0 = dal GD32 al SoC */
	const char *spiegazione;
	const campo_t *campi;
	uint8_t n_campi;
} messaggio_t;

extern const campo_t CAMPI_SENSORI[];
extern const int N_CAMPI_SENSORI;

extern const messaggio_t MESSAGGI[];
extern const int N_MESSAGGI;

/* Descrizione di un messaggio (verso: 1 dal SoC, 0 dal GD32), o NULL. */
const messaggio_t *descrizione_messaggio(uint8_t cmd, int verso_gd32);

/* Valore grezzo di un campo letto dai dati (per i campi a bit: i bit, già spostati). */
int64_t descrizione_grezzo(const campo_t *c, const uint8_t *dati);

#endif /* GD32_DESCRIZIONE_H */
