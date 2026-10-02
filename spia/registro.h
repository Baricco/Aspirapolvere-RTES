/*
 * registro.h - Formato del file in cui la spia registra tutto il traffico dei canali (seriali, socket...).
 *
 * Si registrano i blocchi di byte COSÌ COME ARRIVANO, non i frame: se un decodificatore avesse un errore,
 * i dati grezzi restano intatti e si possono rianalizzare. Il file è binario e si scrive in fretta:
 *
 *   intestazione:  "SPIA1\n"
 *                  uint8 n_canali
 *                  per ogni canale: uint8 id | uint8 protocollo | uint8 lungh_nome | nome | uint8 lungh_disp | disp
 *   record:        int64 tempo_ns | uint8 canale | uint8 direzione | uint16 n | n byte
 *
 * Tutti i numeri sono little-endian (come il SoC e il PC). tempo_ns è CLOCK_MONOTONIC del robot.
 * La direzione è vista dal SoC: VERSO_SOC = dati dalla periferica (GD32, lidar) ai programmi,
 * DAL_SOC = dati dai programmi alla periferica. Per i canali socket (spia socket): VERSO_SOC = dal server, cioè
 * chi è in ascolto (AuxCtrl), al client (RobotApp); DAL_SOC = dal client al server.
 */
#ifndef REGISTRO_H
#define REGISTRO_H

#include <stdint.h>
#include <stdio.h>

#define REG_MAGIA       "SPIA1\n"
#define REG_MAX_CANALI  8
#define REG_MAX_NOME    32
#define REG_MAX_BLOCCO  65535

enum { REG_VERSO_SOC = 0, REG_DAL_SOC = 1 };

/* Come interpretare i byte di un canale (per i decodificatori). */
enum { REG_PROTO_GREZZO = 0, REG_PROTO_GD32 = 1, REG_PROTO_LIDAR = 2, REG_PROTO_EVEREST = 3 };

typedef struct {
	uint8_t id;
	uint8_t protocollo;
	char nome[REG_MAX_NOME];
	char dispositivo[REG_MAX_NOME * 2];
} reg_canale_t;

typedef struct {
	uint8_t n_canali;
	reg_canale_t canali[REG_MAX_CANALI];
} reg_intestazione_t;

typedef struct {
	int64_t tempo_ns;
	uint8_t canale;
	uint8_t direzione;
	uint16_t n;
	uint8_t dati[REG_MAX_BLOCCO];
} reg_record_t;

/* Scrittura. Ritornano 0 oppure -1 (errore di scrittura, vedi errno). */
int reg_scrivi_intestazione(FILE *f, const reg_intestazione_t *h);
int reg_scrivi_record(FILE *f, int64_t tempo_ns, uint8_t canale, uint8_t direzione, const void *dati, uint16_t n);

/* Lettura. Ritornano 1 = letto, 0 = fine del file, -1 = file rovinato o troncato a metà record. */
int reg_leggi_intestazione(FILE *f, reg_intestazione_t *h);
int reg_leggi_record(FILE *f, reg_record_t *r);

const char *reg_nome_protocollo(uint8_t p);
int reg_protocollo_da_nome(const char *s);   /* -1 se sconosciuto */

#endif /* REGISTRO_H */
