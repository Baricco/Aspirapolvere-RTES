/*
 * gd32.h - Protocollo seriale tra il SoC e il microcontrollore GD32F103 del robot.
 *
 * Il GD32 comanda motori, spazzole, aspirazione e motore del lidar, e legge sensori, tasti e batteria.
 * Parla con il SoC su /dev/ttyS3 a 115200 baud, 8N1. Ogni messaggio è un frame:
 *
 *     FA FB | len | cmd | dati (len-3 byte) | ck_hi ck_lo
 *
 *   - len conta cmd + dati + 2 byte di checksum: va da 3 a 255, quindi i dati sono al massimo 252 byte;
 *   - il checksum si calcola su cmd + dati (vedi gd32_checksum) e si trasmette big-endian;
 *   - i campi multi-byte dentro i dati sono little-endian.
 * La descrizione completa, con le fonti, è in docs/protocollo.md.
 *
 * Il modulo non alloca memoria, non usa Linux e non ha stato globale: si potrebbe usare così com'è
 * anche su un microcontrollore. Lettura e scrittura della seriale sono compito di chi lo usa.
 *
 * Uso tipico in ricezione:
 *
 *     gd32_deframer_t d;
 *     gd32_deframer_init(&d);
 *     ...
 *     n = read(fd, buf, sizeof buf);               // blocchi di dimensione qualsiasi
 *     gd32_deframer_scrivi(&d, buf, n, gestisci, ctx);   // chiama gestisci() per ogni frame valido
 */
#ifndef GD32_H
#define GD32_H

#include <stddef.h>
#include <stdint.h>

#define GD32_INTESTAZIONE_1 0xFA
#define GD32_INTESTAZIONE_2 0xFB
#define GD32_MAX_DATI       252                       /* len massimo 255 = cmd + 252 + 2 */
#define GD32_MAX_FRAME      (2 + 1 + 1 + GD32_MAX_DATI + 2)

/*
 * Codici cmd. Lo stesso codice si usa per la richiesta (SoC -> GD32) e per la risposta (GD32 -> SoC).
 * Nomi dalle funzioni CSerialMessagePacket::packet* di AuxCtrl, oppure dal significato osservato.
 */
enum {
	GD32_SLEEP               = 0x04,  /* packetStm32Sleep; il GD32 conferma con 0x04 e smette di mandare i sensori */
	GD32_WAKEUP_ACK          = 0x05,  /* GD32 -> SoC al risveglio (tasto premuto); il SoC conferma con 0x05 */
	GD32_HEARTBEAT           = 0x06,  /* packetHeartBeat: SoC -> GD32 ogni 200 ms */
	GD32_VERSIONE            = 0x07,  /* richiesta / risposta con la versione del firmware (ASCII) */
	GD32_IMU_ZERO            = 0x08,  /* packetSetIMUZero; risposta 01 */
	GD32_RESET_ERRORI        = 0x0a,  /* packetResetErrorCode */
	GD32_SOC_ATTIVO          = 0x0c,  /* 01 = SoC attivo, 00 = prima della sospensione; il GD32 risponde 0x0c 01 */
	GD32_KEEPALIVE           = 0x0d,  /* keepalive di AuxCtrl, ogni 3 s: basta da solo a evitare il reset del GD32 */
	GD32_SENSORI             = 0x15,  /* GD32 -> SoC: stato dei sensori, 96 byte */
	GD32_TIPO_CONTROLLO      = 0x65,  /* packetMotorControlType, 1 byte */
	GD32_VELOCITA            = 0x66,  /* packetMotorVelocity: int32 v, int32 omega (tick/s) ogni 20 ms */
	GD32_VELOCITA_RUOTE      = 0x67,  /* packetMotorSpeed: int32 sinistra, int32 destra */
	GD32_ASPIRAZIONE         = 0x68,  /* packetBlowerSpeed, 2 byte */
	GD32_SPAZZOLA_LATERALE   = 0x69,  /* packetBrushSpeed, int8 */
	GD32_SPAZZOLA_CENTRALE   = 0x6a,  /* packetRollingSpeed, int8 */
	GD32_MOTORE_LIDAR        = 0x71,  /* controlLidarPwm: int32 potenza 0..100 */
	GD32_CLIFF_IR            = 0x78,  /* packetCliffIRControl, bool */
	GD32_CLIFF_IR_DIREZIONE  = 0x79,  /* packetCliffIRDirection, bool */
	GD32_LED_TASTO           = 0x8d,  /* packetButtonLEDState, 1 byte */
	GD32_ALIMENTAZIONE_LIDAR = 0x97,  /* packetLidarPower, bool */
	GD32_ALIMENTAZIONE_SOC   = 0x99,  /* packetR16Power, bool */
	GD32_RIAVVIA_SOC         = 0x9a,  /* packetRestartR16System */
	GD32_CARICATORE          = 0x9b,  /* packetChargerPower, bool */
	GD32_IMU_CALIBRA         = 0xa1,  /* packetIMUCalibaration */
	GD32_IMU_STATO_CALIBRA   = 0xa2,  /* packetIMUFactoryCalibrateState */
	GD32_MAGNETOMETRO_CALIBRA = 0xa3,
	GD32_MAGNETOMETRO_STATO  = 0xa4,
};

/* Un frame decodificato: solo cmd e dati, l'intestazione e il checksum sono già stati verificati. */
typedef struct {
	uint8_t cmd;
	uint8_t n_dati;
	uint8_t dati[GD32_MAX_DATI];
} gd32_frame_t;

/*
 * Checksum di cmd + dati (CRobotPacket::calcCheckSum in AuxCtrl): somma modulo 2^16 delle parole
 * big-endian da 16 bit; se i byte sono dispari, l'ultimo non si somma ma si mette in XOR.
 */
uint16_t gd32_checksum(const uint8_t *b, size_t n);

/*
 * Costruisce in `out` il frame completo con il codice `cmd` e `n` byte di dati.
 * Ritorna la lunghezza del frame (n + 6), oppure 0 se n > GD32_MAX_DATI o se out_dim non basta.
 */
size_t gd32_componi(uint8_t cmd, const void *dati, size_t n, uint8_t *out, size_t out_dim);

/* Nome leggibile di un codice cmd, oppure NULL se non è noto. */
const char *gd32_nome_cmd(uint8_t cmd);

/*
 * Deframer: riceve i byte così come arrivano dalla seriale, in blocchi di dimensione qualsiasi (anche
 * spezzati a metà frame) e restituisce i frame completi con checksum giusto.
 *
 * Se un frame è rovinato (checksum sbagliato, len < 3, byte persi), scarta solo il primo byte FA e rianalizza
 * i byte successivi: se lì dentro inizia un frame buono, lo ritrova. Così un frame troncato non fa perdere
 * anche quello che lo segue.
 */
typedef struct {
	uint8_t buf[GD32_MAX_FRAME];   /* il frame in costruzione */
	size_t n;                      /* byte in buf */
	/* byte da rianalizzare dopo un errore (perché un frame basta: vedi errore() in gd32.c) */
	uint8_t attesa[GD32_MAX_FRAME];
	size_t attesa_n, attesa_i;

	/* Contatori, per controllare la qualità della linea */
	uint64_t frame_ok;
	uint64_t checksum_errati;      /* frame con intestazione e lunghezza plausibili ma checksum sbagliato */
	uint64_t len_errati;           /* len < 3 */
	uint64_t byte_scartati;        /* byte che non fanno parte di nessun frame valido */
} gd32_deframer_t;

typedef void (*gd32_frame_cb)(const gd32_frame_t *f, void *ctx);

void gd32_deframer_init(gd32_deframer_t *d);

/* Analizza n byte. Per ogni frame valido trovato chiama cb(frame, ctx). Ritorna il numero di frame trovati. */
size_t gd32_deframer_scrivi(gd32_deframer_t *d, const uint8_t *b, size_t n, gd32_frame_cb cb, void *ctx);

/*
 * Campi noti del frame dei sensori (0x15). Per ora solo quelli ricavati dal codice di AuxCtrl
 * (updateBatterySensor); gli altri sono in docs/protocollo.md come ipotesi.
 */
typedef struct {
	int in_carica;      /* dati[7] bit 0 */
	int sulla_base;     /* dati[7] bit 1 (AuxCtrl considera "sulla base" anche quando bit 0 = 1) */
	int tensione_dv;    /* dati[8]: tensione della batteria in decimi di volt */
} gd32_batteria_t;

/* Ritorna 0 se f è un frame dei sensori abbastanza lungo, -1 altrimenti. */
int gd32_leggi_batteria(const gd32_frame_t *f, gd32_batteria_t *out);

#endif /* GD32_H */
