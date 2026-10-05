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
 * Unità di 0x66 / 0x67 e dei byte 32 / 36 dei sensori [misura del 5/10, robot a terra; docs/fase3.md]:
 *   v = tick dell'encoder al secondo di ogni ruota; ω = metà della differenza fra le ruote, in tick/s (ruota
 *   sinistra v - ω, destra v + ω). Gli encoder (byte 16 e 24) contano gli stessi tick.
 * Il robot fa ~4580 tick per metro, NON i 16561 che everest ricava da CRobotParams (raggio 34,75 mm, 3616 tick per
 * giro): con le scale di everest (16561,27 e 1999,77) il robot va ~3,6 volte più veloce del voluto. Misure:
 *   - rotazione: 286 unità per 3 s -> 86,1° per l'IMU, che a sua volta segna il 3,2 % in meno del lidar (40,7° contro
 *     42°, 201,6° contro 208°): 0,517 rad/s, cioè 553 unità per rad/s;
 *   - traslazione con il lidar: 1656 unità per 2 s -> 0,68 e 0,81 m (4100-4900 unità per m/s);
 *   - velocità massima del GD32: 1772 unità (~0,39 m/s).
 * Seconda fonte: kaiaai/VacuumTiger (proscenic-m6pro.toml) usa 4400-4460 e 523, il 4-5 % in meno.
 */
#define GD32_SCALA_W      553.0                                /* unità di ω per rad/s [misura] */
#define GD32_CARREGGIATA  0.2415                               /* m fra le ruote [firmware: CRobotParams] */
#define GD32_SCALA_V      (2 * GD32_SCALA_W / GD32_CARREGGIATA) /* unità di v per m/s = tick al metro (~4580) */

/*
 * Codici cmd. Lo stesso codice si usa per la richiesta (SoC -> GD32) e per la risposta (GD32 -> SoC).
 * Nomi dalle funzioni CSerialMessagePacket::packet* di everest-server e AuxCtrl (fase 2, blocco A),
 * oppure dal significato osservato. Formato dei dati e fonti in docs/protocollo.md.
 */
enum {
	GD32_CODICE_ERRORE       = 0x01,  /* GD32 -> SoC: codice di errore (everest lo salva; mai visto finora) */
	GD32_SLEEP               = 0x04,  /* packetStm32Sleep; il GD32 conferma con 0x04 e smette di mandare i sensori */
	GD32_WAKEUP_ACK          = 0x05,  /* packetWakeupAck; GD32 -> SoC al risveglio (tasto premuto), il SoC conferma */
	GD32_HEARTBEAT           = 0x06,  /* packetHeartBeat: SoC -> GD32 ogni 200 ms */
	GD32_VERSIONE            = 0x07,  /* packetRequireSystemVersion; risposta con la versione del firmware (ASCII) */
	GD32_IMU_ZERO            = 0x08,  /* packetSetIMUZero; risposta 01 */
	GD32_RESET_ERRORI        = 0x0a,  /* packetResetErrorCode */
	GD32_SORVEGLIA_SOC       = 0x0c,  /* packetTinaHaltedDetectOnce(bool): 01 all'avvio, 00 prima della sospensione */
	GD32_KEEPALIVE           = 0x0d,  /* keepalive di AuxCtrl, ogni 3 s: basta da solo a evitare il reset del GD32 */
	GD32_SENSORI             = 0x15,  /* GD32 -> SoC: stato dei sensori, 96 byte (gd32_sensori_t) */
	GD32_IMU_GREZZA          = 0x17,  /* packetRequireIMURawData: 1 byte utile (01); everest ne manda 4 */
	GD32_FUNZIONI_MACCHINA   = 0x18,  /* packetGetMachineFunctionList, senza dati */
	GD32_CLIFF_RETROMARCIA   = 0x19,  /* packetCliffBack(bool): retromarcia automatica del GD32 sugli anti-caduta */
	GD32_TIPO_CONTROLLO      = 0x65,  /* packetMotorControlType, 1 byte */
	GD32_VELOCITA            = 0x66,  /* packetMotorVelocity: int32 v, int32 omega (GD32_SCALA_V/W) ogni 20 ms */
	GD32_VELOCITA_RUOTE      = 0x67,  /* packetMotorSpeed: int32 sinistra, int32 destra (stessa scala di v) */
	GD32_ASPIRAZIONE         = 0x68,  /* packetBlowerSpeed: int16 = velocità / 10 (everest al massimo 30000) */
	GD32_SPAZZOLA_LATERALE   = 0x69,  /* packetBrushSpeed, int8 */
	GD32_SPAZZOLA_CENTRALE   = 0x6a,  /* packetRollingSpeed, int8 */
	GD32_POMPA_ACQUA         = 0x6b,  /* packetWaterPumpSpeed, int8: everest manda 0. NON USARE (niente serbatoio) */
	GD32_MOTORE_LIDAR        = 0x71,  /* controlLidarPwm: int32 potenza 0..100 */
	GD32_CLIFF_IR            = 0x78,  /* packetCliffIRControl, bool */
	GD32_CLIFF_IR_DIREZIONE  = 0x79,  /* packetCliffIRDirection, bool */
	GD32_PARAMETRI_AUTONOMI  = 0x7a,  /* packetSelfControlParams: 24 byte (struttura da capire) */
	GD32_MASCHERA_SENSORI    = 0x7c,  /* packetMaskSensorMotion: 3 byte (all'avvio 00 ff ff) */
	GD32_BUMPER_MORBIDO      = 0x80,  /* packetBumperSmooth(bool) */
	GD32_ASPIRAZIONE_MODO    = 0x84,  /* packetBlowerControlMode: int32 */
	GD32_ASPIRAZIONE_PWM     = 0x85,  /* packetBlowerPwmOnly: uint8 */
	GD32_ASPIRAZIONE_PWM_BAT = 0x86,  /* packetBlowerPwmBattery: uint8 (PWM compensato con la tensione) */
	GD32_LED_TASTO           = 0x8d,  /* packetButtonLEDState, 1 byte */
	GD32_ALIMENTAZIONE_LIDAR = 0x97,  /* packetLidarPower, bool */
	GD32_ALIMENTAZIONE_SOC   = 0x99,  /* packetR16Power, bool. NON USARE */
	GD32_RIAVVIA_SOC         = 0x9a,  /* packetRestartR16System. NON USARE */
	GD32_CARICATORE          = 0x9b,  /* packetChargerPower (solo AuxCtrl), bool */
	GD32_ALIMENTAZIONE_SENSORI = 0x9d, /* packetSensorPower(bool) */
	GD32_IMU_CALIBRA         = 0xa1,  /* packetIMUFactoryCalibrate / packetIMUCalibaration */
	GD32_IMU_STATO_CALIBRA   = 0xa2,  /* packetIMUFactoryCalibrateState */
	GD32_MAGNETOMETRO_CALIBRA = 0xa3, /* packetGeoMagnetismCalibrate */
	GD32_MAGNETOMETRO_STATO  = 0xa4,  /* packetGeoMagnetismCalibrateState */
	GD32_DEBUG               = 0xab,  /* GD32 -> SoC: messaggio di debug (handleDebug1) */
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
 * Frame dei sensori 0x15 (96 byte), decodificato.
 *
 * Fonte principale: CSerialMessageHandle::handleSensorMessage e update*Sensor di everest-server, che copia i
 * byte 0..85 del frame in una struttura con gli stessi offset, IGNORA i byte 86..87 e sposta i byte 88..95.
 * Tra parentesi: [V] verificato sul robot, [C] letto nel codice (tipo e scala certi, significato dal contesto),
 * [?] significato da confermare (blocchi C e D della fase 2). Tabella completa in docs/protocollo.md.
 */
typedef struct {
	uint8_t stato;              /* byte 0 [C]: everest lo confronta con 1 (01 a riposo) */
	int bumper_sx, bumper_dx;   /* byte 1 [V]: sx = bit2|bit3, dx = bit1|bit0 (everest fa questo OR) */
	uint8_t bumper_bit;         /* byte 1 grezzo: bit3 e bit0 [?] secondo contatto di ogni lato */
	uint8_t byte2;              /* byte 2 [?]: everest usa i bit 1 e 0; non cambiano sollevando il robot */
	int cliff[4];               /* byte 3 [V]: da sinistra a destra = bit 0 sx laterale, 1 sx anteriore, 2 dx anteriore,
	                               3 dx laterale; 1 = nessun pavimento (prove con la mano, fase 2 blocco D) */
	int cassetto;               /* byte 4 bit 2 [V]: cassetto della polvere presente */
	uint8_t byte4_altri;        /* byte 4 bit 1 e 0 [?] (serbatoio / mop: everest ha detectWaterboxLost, detectMoppingLost) */
	uint8_t byte5;              /* byte 5 [?]: everest usa i bit 1 e 0 */
	uint8_t ir_base_a, ir_base_b; /* byte 6 [V]: due ricevitori, a = nibble alto = DESTRO, b = nibble basso = SINISTRO;
	                                 ogni bit 1/2/4/8 = un fascio della base (fase 3, prova ir1) */
	int in_carica;              /* byte 7 bit 0 [V] */
	int sulla_base;             /* byte 7 bit 0 | bit 1 [V] (bit 1 = contatto con la base, bit 0 = in carica) */
	int tensione_dv;            /* byte 8 [V]: tensione della batteria in decimi di volt */
	int temperatura_c;          /* byte 9 [C] int8: temperatura della batteria in °C (31 a riposo) */
	int32_t encoder_sx, encoder_dx; /* byte 16 e 24 [V]: contatori assoluti (3616 tick per giro) */
	int32_t vel_sx, vel_dx;     /* byte 32 e 36 [V]: NON sono le due ruote, ma v e ω misurati dal GD32 sugli encoder in
	                               ~70 ms (nomi storici): v = vel_sx / GD32_SCALA_V m/s, ω = vel_dx / GD32_SCALA_W rad/s;
	                               valgono solo in modalità 02 (quella del driver) */
	/* IMU [V] (fase 2, blocco D): indice 0 = imbardata (+ antiorario visto dall'alto), 1 = beccheggio (+ naso in giù),
	   2 = rollio (+ fianco sinistro in giù) */
	int16_t vel_angolare[3];    /* byte 40, 44, 48: velocità angolare in 0,01 gradi/s */
	int16_t angolo[3];          /* byte 42, 46, 50: angolo integrato dal GD32 in 0,01 gradi (l'imbardata deriva) */
	int16_t gravita[3];         /* byte 52, 54, 56 [V]: direzione "su" in centesimi di g, assi X indietro, Y destra, Z su */
	int tasto_accensione_ms;    /* byte 58 [V]: durata della pressione (uint16 x 10 ms) */
	int tasto_punto_ms;         /* byte 60 [C]: tasto "punto", assente su questo modello */
	int tasto_casa_ms;          /* byte 62 [V] */
	uint8_t controllo;          /* byte 64 [V]: bit 5 = sensori alimentati (0x9d), bit 7 = lidar alimentato (0x97);
	                               bit 0, 1, 6 [?] (0x42 di base) */
	uint8_t led;                /* byte 65 [V]: stato del LED (copia dell'ultimo 0x8d) */
	uint8_t byte66;             /* byte 66 [C]: everest usa solo i 4 bit bassi */
	int8_t byte67;              /* byte 67 [V] int8: eco del comando della spazzola laterale */
	uint8_t byte68;             /* byte 68 [V]: eco della spazzola centrale (100 durante lo sblocco) */
	int byte69_x100;            /* byte 69 x 100 [V]: giri/min della ventola dell'aspirazione */
	uint8_t acqua;              /* byte 70 [?]: livello del serbatoio dell'acqua (VacuumTiger), qui assente */
	uint16_t grezzo72;          /* byte 72 [C] uint16 */
	uint16_t correnti[6];       /* byte 74..84 [V] uint16, mA: 0 ruota sx, 1 ruota dx, 2 spazzola laterale, 3 spazzola
	                               centrale (~58 a riposo), 4 aspirazione (~14 a riposo); 5 (byte 84) [?] sempre 2-3 */
	int16_t byte88;             /* byte 88 [C]: everest guarda solo se > 0 (costante 0x1903) */
	int16_t magnetico[3];       /* byte 90, 92, 94 [V]: copia delle velocità angolari (stessi valori), non un magnetometro */
} gd32_sensori_t;

/* Ritorna 0 se f è un frame dei sensori di 96 byte (decodificato in *out), -1 altrimenti. */
int gd32_leggi_sensori(const gd32_frame_t *f, gd32_sensori_t *out);

/* Bit del byte 3 (anti-caduta) quando il robot è sollevato: tutti i sensori vedono "vuoto". */
#define GD32_CLIFF_TUTTI_VUOTO 0x0f

/* Byte 64 bit 5: sensori alimentati (0 dopo 0x9d 00; everest ignora anti-caduta e bumper quando è 0). */
#define GD32_CONTROLLO_SENSORI_ACCESI 0x20

#endif /* GD32_H */
