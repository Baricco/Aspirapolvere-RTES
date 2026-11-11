/*
 * lidar.h - Protocollo seriale del lidar 3irobotix LDS3 (firmware "LDS3_NOR", maggio 2018).
 *
 * Il lidar manda i dati al SoC su /dev/ttyS1 a 115200 baud (getLidarData in AuxCtrl). Il suo motore NON si
 * comanda da qui: lo alimenta e lo regola il GD32 (comandi 0x97 e 0x71, vedi gd32.h).
 * All'accensione il lidar stampa un testo in cinese (codifica GBK) con il nome del produttore e la versione;
 * poi manda solo pacchetti binari:
 *
 *     AA | len_hi len_lo | indirizzo | tipo | cmd | n_hi n_lo | parametri (n byte) | ck_hi ck_lo
 *
 *   - len = numero di byte del pacchetto SENZA i 2 del checksum (= 8 + n). AuxCtrl accetta solo len_hi = 0 e
 *     len_lo tra 6 e 100 (CLidarPacketReceiver::processStateHeader2 e processStateLength);
 *   - tutti i campi multi-byte sono big-endian (CLidarPacket::bufToUByte2);
 *   - il checksum copre i byte da AA all'ultimo parametro (len byte) e dipende dal campo indirizzo:
 *     indirizzo 1 -> somma dei byte modulo 2^16; altrimenti CRC-16/MODBUS (CLidarPacket::verifyCheckSum,
 *     calcCheckSumSum, calcCheckSumCRC: le tabelle del CRC in AuxCtrl sono quelle Modbus standard).
 *   - tipo: 0x61 nel pacchetto di versione catturato.
 * Le fonti sono in docs/protocollo.md.
 *
 * Come gd32.h: niente allocazioni, niente Linux, nessuno stato globale.
 */
#ifndef LIDAR_H
#define LIDAR_H

#include <stddef.h>
#include <stdint.h>

#define LIDAR_HEADER 0xAA
#define LIDAR_LEN_MIN      6
#define LIDAR_LEN_MAX      100
#define LIDAR_MAX_PACKET (LIDAR_LEN_MAX + 2)
#define LIDAR_MAX_PARAMS (LIDAR_LEN_MAX - 8)

/* Codici cmd (C3iroboticsLidar::analysisPacket in AuxCtrl). */
enum {
	LIDAR_SCAN        = 0xA9,  /* analysisToothScan: un settore di misure, formato vecchio */
	LIDAR_HEALTH           = 0xAB,  /* analysisHealthInfo */
	LIDAR_VERSION         = 0xAC,  /* analysisLidarVersion: testo ASCII, es. "...LDS3_NOR_1805041536" */
	LIDAR_SCAN_NEW  = 0xAD,  /* analysisNewToothScan: un settore di misure, formato nuovo */
	LIDAR_SPEED         = 0xAE,  /* analysisLidarSpeed: velocità di rotazione */
};

typedef struct {
	uint8_t address;
	uint8_t type;
	uint8_t cmd;
	uint16_t n_param;        /* byte di parametri presenti davvero: len - 8 */
	uint16_t n_declared;   /* il campo n del pacchetto; AuxCtrl non controlla che sia uguale a n_param */
	uint8_t param[LIDAR_MAX_PARAMS];
} lidar_packet_t;

/* Somma dei byte modulo 2^16. */
uint16_t lidar_sum(const uint8_t *b, size_t n);

/* CRC-16/MODBUS: polinomio 0xA001 riflesso, valore iniziale 0xFFFF. */
uint16_t lidar_crc16(const uint8_t *b, size_t n);

/*
 * Costruisce un pacchetto completo. Ritorna la sua lunghezza (n + 10) oppure 0 se i parametri non ci stanno.
 * Serve ai test e al simulatore: il robot vero non manda pacchetti al lidar.
 */
size_t lidar_build_packet(uint8_t address, uint8_t type, uint8_t cmd, const void *param, size_t n,
                     uint8_t *out, size_t out_size);

const char *lidar_cmd_name(uint8_t cmd);

/* Deframer con la stessa logica di gd32_deframer_t: blocchi qualsiasi, risincronizzazione dopo un errore. */
typedef struct {
	uint8_t buf[LIDAR_MAX_PACKET];
	size_t n;
	uint8_t pending[LIDAR_MAX_PACKET];
	size_t pending_n, pending_i;

	uint64_t packets_ok;
	uint64_t bad_checksums;
	uint64_t bad_lengths;           /* len_hi diverso da 0 o len_lo fuori da 6..100 */
	uint64_t discarded_bytes;        /* anche il testo di avvio finisce qui */
} lidar_deframer_t;

typedef void (*lidar_packet_cb)(const lidar_packet_t *p, void *ctx);

void lidar_deframer_init(lidar_deframer_t *d);
size_t lidar_deframer_feed(lidar_deframer_t *d, const uint8_t *b, size_t n, lidar_packet_cb cb, void *ctx);

#endif /* LIDAR_H */
