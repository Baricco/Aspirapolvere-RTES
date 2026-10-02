/*
 * everest.h - Messaggi tra AuxCtrl e RobotApp sul socket Unix /tmp/AuxEverest_Sock (classi CMsgServer,
 * CommonClient, CMsgHead in AuxCtrl). Ogni messaggio è un'intestazione da 32 byte seguita dai dati:
 *
 *   +0  uint32 magia 0x5867a9b0
 *   +4  int32  comando              (1° parametro di CMsgServer::Send)
 *   +8  int32  lunghezza dei dati   (Send controlla che sia coerente con il puntatore ai dati;
 *                                    ReceiveLocalData la confronta con un massimo di ~4 MB)
 *   +12 int32  ?                    (4° parametro di Send)
 *   +16 uint64 tempo                (5° parametro di Send)
 *   +24 8 byte                      (il puntatore ai dati nella memoria del mittente: non ha significato)
 *
 * Tutto little-endian. Fonte: disassemblato di CMsgServer::Send e CommonClient::SendLocalData/ReceiveLocalData.
 */
#ifndef EVEREST_H
#define EVEREST_H

#include <stddef.h>
#include <stdint.h>

#define EVEREST_MAGIA      0x5867a9b0u
#define EVEREST_TESTA      32
#define EVEREST_MAX_LUNG   (4 * 1024 * 1024)
#define EVEREST_MAX_DATI   16384          /* oltre, i dati si contano ma non si conservano */

typedef struct {
	int32_t comando;
	int32_t lunghezza;
	int32_t campo12;
	uint64_t tempo;
	uint32_t n_dati;                     /* byte conservati in dati[] (<= EVEREST_MAX_DATI) */
	uint8_t dati[EVEREST_MAX_DATI];
} everest_msg_t;

typedef struct {
	uint8_t testa[EVEREST_TESTA];
	size_t n_testa;
	everest_msg_t msg;
	size_t ricevuti;                     /* byte di dati ricevuti del messaggio in corso */
	uint64_t messaggi_ok, testate_errate, byte_scartati;
} everest_deframer_t;

typedef void (*everest_msg_cb)(const everest_msg_t *m, void *ctx);

void everest_deframer_init(everest_deframer_t *d);
size_t everest_deframer_scrivi(everest_deframer_t *d, const uint8_t *b, size_t n, everest_msg_cb cb, void *ctx);

#endif /* EVEREST_H */
