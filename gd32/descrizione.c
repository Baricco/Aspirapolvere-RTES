/* descrizione.c - Descrizione dei messaggi SoC <-> GD32: vedi descrizione.h. Fonti: gd32.h, docs/protocollo.md,
 * docs/fase2.md, docs/fase3.md. */
#include "descrizione.h"

#include <stddef.h>

#define N(x) ((int)(sizeof(x) / sizeof((x)[0])))

/* Frame dei sensori 0x15: 96 byte ogni 20 ms. */
const campo_t CAMPI_SENSORI[] = {
	{ "stato", 0, 1, -1, 0, "u8", 1, "", 'C', CAMPO_STATO, "everest lo confronta con 1 (vale 1 a riposo)" },
	{ "bumper_dx", 1, 1, 0, 2, "bit", 1, "", 'V', CAMPO_URTI, "bumper destro: bit 0 e 1 (1 = premuto). Con i sensori spenti si legge premuto" },
	{ "bumper_sx", 1, 1, 2, 2, "bit", 1, "", 'V', CAMPO_URTI, "bumper sinistro: bit 2 e 3 (1 = premuto)" },
	{ "byte2", 2, 1, -1, 0, "u8", 1, "", '?', CAMPO_IGNOTO, "everest usa i bit 1 e 0; non cambiano sollevando il robot" },
	{ "cliff_sx_laterale", 3, 1, 0, 1, "bit", 1, "", 'V', CAMPO_URTI, "anti-caduta sinistro laterale: 1 = nessun pavimento" },
	{ "cliff_sx_anteriore", 3, 1, 1, 1, "bit", 1, "", 'V', CAMPO_URTI, "anti-caduta sinistro anteriore: 1 = nessun pavimento" },
	{ "cliff_dx_anteriore", 3, 1, 2, 1, "bit", 1, "", 'V', CAMPO_URTI, "anti-caduta destro anteriore: 1 = nessun pavimento" },
	{ "cliff_dx_laterale", 3, 1, 3, 1, "bit", 1, "", 'V', CAMPO_URTI, "anti-caduta destro laterale: 1 = nessun pavimento. Sulla base leggono tutti 1" },
	{ "byte4_bit0_1", 4, 1, 0, 2, "bit", 1, "", '?', CAMPO_IGNOTO, "serbatoio / panno (everest: detectWaterboxLost, detectMoppingLost): qui non montati" },
	{ "cassetto", 4, 1, 2, 1, "bit", 1, "", 'V', CAMPO_BASE, "cassetto della polvere presente" },
	{ "byte5", 5, 1, -1, 0, "u8", 1, "", '?', CAMPO_IGNOTO, "everest usa i bit 1 e 0" },
	{ "ir_base_b", 6, 1, 0, 4, "bit", 1, "", 'V', CAMPO_BASE, "ricevitore IR della base (davanti): ogni bit 1/2/4/8 è un fascio della base" },
	{ "ir_base_a", 6, 1, 4, 4, "bit", 1, "", 'V', CAMPO_BASE, "secondo ricevitore IR della base" },
	{ "in_carica", 7, 1, 0, 1, "bit", 1, "", 'V', CAMPO_BASE, "1 = in carica" },
	{ "contatto_base", 7, 1, 1, 1, "bit", 1, "", 'V', CAMPO_BASE, "1 = contatti di carica (dietro) sulla base" },
	{ "tensione", 8, 1, -1, 0, "u8", 0.1, "V", 'V', CAMPO_BASE, "tensione della batteria in decimi di volt" },
	{ "temperatura", 9, 1, -1, 0, "i8", 1, "°C", 'C', CAMPO_BASE, "temperatura della batteria" },
	{ "encoder_sx", 16, 4, -1, 0, "i32", 1, "tick", 'V', CAMPO_ODOMETRIA, "contatore della ruota sinistra: 3616 tick per giro, 60,4 µm per tick; in avanti aumenta" },
	{ "encoder_dx", 24, 4, -1, 0, "i32", 1, "tick", 'V', CAMPO_ODOMETRIA, "contatore della ruota destra" },
	{ "v_misurata", 32, 4, -1, 0, "i32", 1.0 / 16561.27, "m/s", 'V', CAMPO_ODOMETRIA, "velocità lineare misurata dal GD32 sugli encoder in ~70 ms (solo in modalità 02)" },
	{ "w_misurata", 36, 4, -1, 0, "i32", 1.0 / 1999.77, "rad/s", 'V', CAMPO_ODOMETRIA, "velocità angolare misurata (solo in modalità 02)" },
	{ "vel_imbardata", 40, 2, -1, 0, "i16", 0.01, "°/s", 'V', CAMPO_IMU, "IMU: velocità angolare intorno all'asse verticale (+ = antiorario)" },
	{ "angolo_imbardata", 42, 2, -1, 0, "i16", 0.01, "°", 'V', CAMPO_IMU, "IMU: angolo integrato dal GD32 (deriva ~0,3°/h); 0x08 lo azzera" },
	{ "vel_beccheggio", 44, 2, -1, 0, "i16", 0.01, "°/s", 'V', CAMPO_IMU, "IMU: velocità angolare di beccheggio" },
	{ "angolo_beccheggio", 46, 2, -1, 0, "i16", 0.01, "°", 'V', CAMPO_IMU, "IMU: beccheggio (+ = naso in giù)" },
	{ "vel_rollio", 48, 2, -1, 0, "i16", 0.01, "°/s", 'V', CAMPO_IMU, "IMU: velocità angolare di rollio" },
	{ "angolo_rollio", 50, 2, -1, 0, "i16", 0.01, "°", 'V', CAMPO_IMU, "IMU: rollio (+ = fianco sinistro in giù)" },
	{ "gravita_x", 52, 2, -1, 0, "i16", 0.01, "g", 'V', CAMPO_IMU, "direzione \"su\": asse x (indietro)" },
	{ "gravita_y", 54, 2, -1, 0, "i16", 0.01, "g", 'V', CAMPO_IMU, "direzione \"su\": asse y (destra)" },
	{ "gravita_z", 56, 2, -1, 0, "i16", 0.01, "g", 'V', CAMPO_IMU, "direzione \"su\": asse z (100 a robot piano)" },
	{ "tasto_accensione", 58, 2, -1, 0, "u16", 10, "ms", 'V', CAMPO_TASTI, "durata della pressione del tasto di accensione" },
	{ "tasto_punto", 60, 2, -1, 0, "u16", 10, "ms", 'C', CAMPO_TASTI, "tasto \"punto\": non c'è su questo modello" },
	{ "tasto_casa", 62, 2, -1, 0, "u16", 10, "ms", 'V', CAMPO_TASTI, "durata della pressione del tasto casa" },
	{ "controllo_bit0_1", 64, 1, 0, 2, "bit", 1, "", '?', CAMPO_STATO, "bit 0 e 1 del byte di controllo (0x42 di base)" },
	{ "sensori_accesi", 64, 1, 5, 1, "bit", 1, "", 'V', CAMPO_STATO, "1 = sensori alimentati (0x9d 01). A 0 bumper e anti-caduta non valgono" },
	{ "controllo_bit6", 64, 1, 6, 1, "bit", 1, "", '?', CAMPO_STATO, "sempre 1 finora" },
	{ "lidar_alimentato", 64, 1, 7, 1, "bit", 1, "", 'V', CAMPO_STATO, "1 = lidar alimentato (0x97 01); misurato il 2026-10-05 sulla registrazione c1" },
	{ "led", 65, 1, -1, 0, "u8", 1, "", 'V', CAMPO_TASTI, "modo del LED del tasto (eco dell'ultimo 0x8d)" },
	{ "byte66", 66, 1, -1, 0, "u8", 1, "", 'C', CAMPO_IGNOTO, "everest usa i 4 bit bassi" },
	{ "eco_laterale", 67, 1, -1, 0, "i8", 1, "", 'V', CAMPO_MOTORI, "eco del comando della spazzola laterale" },
	{ "eco_centrale", 68, 1, -1, 0, "u8", 1, "", 'V', CAMPO_MOTORI, "eco della spazzola centrale (100 durante lo sblocco)" },
	{ "ventola", 69, 1, -1, 0, "u8", 100, "giri/min", 'V', CAMPO_MOTORI, "giri della ventola dell'aspirazione" },
	{ "acqua", 70, 1, -1, 0, "u8", 1, "", '?', CAMPO_IGNOTO, "livello del serbatoio dell'acqua: non montato" },
	{ "byte72", 72, 2, -1, 0, "u16", 1, "", 'C', CAMPO_IGNOTO, "everest lo legge come u16" },
	{ "corrente_ruota_sx", 74, 2, -1, 0, "u16", 1, "mA", 'V', CAMPO_MOTORI, "corrente del motore della ruota sinistra" },
	{ "corrente_ruota_dx", 76, 2, -1, 0, "u16", 1, "mA", 'V', CAMPO_MOTORI, "corrente del motore della ruota destra" },
	{ "corrente_laterale", 78, 2, -1, 0, "u16", 1, "mA", 'V', CAMPO_MOTORI, "corrente della spazzola laterale" },
	{ "corrente_centrale", 80, 2, -1, 0, "u16", 1, "mA", 'V', CAMPO_MOTORI, "corrente della spazzola centrale (~58 a riposo)" },
	{ "corrente_aspirazione", 82, 2, -1, 0, "u16", 1, "mA", 'V', CAMPO_MOTORI, "corrente dell'aspirazione (~14 a riposo)" },
	{ "byte84", 84, 2, -1, 0, "u16", 1, "", '?', CAMPO_IGNOTO, "2-3 sempre" },
	{ "byte88", 88, 2, -1, 0, "i16", 1, "", 'C', CAMPO_IGNOTO, "costante 0x1903 = 6403; everest guarda solo se è > 0" },
	{ "copia_vel_imbardata", 90, 2, -1, 0, "i16", 0.01, "°/s", 'V', CAMPO_IMU, "copia del byte 40" },
	{ "copia_vel_beccheggio", 92, 2, -1, 0, "i16", 0.01, "°/s", 'V', CAMPO_IMU, "copia del byte 44" },
	{ "copia_vel_rollio", 94, 2, -1, 0, "i16", 0.01, "°/s", 'V', CAMPO_IMU, "copia del byte 48" },
};
const int N_CAMPI_SENSORI = N(CAMPI_SENSORI);

static const campo_t C_UNO[] = { { "valore", 0, 1, -1, 0, "u8", 1, "", 'V', CAMPO_STATO, "" } };
static const campo_t C_BOOL[] = { { "acceso", 0, 1, -1, 0, "u8", 1, "", 'V', CAMPO_STATO, "1 = acceso, 0 = spento" } };
static const campo_t C_VEL[] = {
	{ "v", 0, 4, -1, 0, "i32", 1.0 / 16561.27, "m/s", 'V', CAMPO_ODOMETRIA, "velocità lineare: tick/s di ogni ruota" },
	{ "w", 4, 4, -1, 0, "i32", 1.0 / 1999.77, "rad/s", 'V', CAMPO_ODOMETRIA, "velocità angolare (+ = antiorario)" },
};
static const campo_t C_RUOTE[] = {
	{ "sinistra", 0, 4, -1, 0, "i32", 1.0 / 16561.27, "m/s", 'V', CAMPO_ODOMETRIA, "ruota sinistra" },
	{ "destra", 4, 4, -1, 0, "i32", 1.0 / 16561.27, "m/s", 'V', CAMPO_ODOMETRIA, "ruota destra" },
};
static const campo_t C_MODO[] = { { "modo", 0, 1, -1, 0, "u8", 1, "", 'V', CAMPO_MOTORI, "00 motori spenti, 01 per ruota (0x67), 02 v/omega (0x66)" } };
static const campo_t C_SPAZZOLA[] = { { "velocita", 0, 1, -1, 0, "i8", 1, "", 'V', CAMPO_MOTORI, "positivo = pulizia; negativo = sblocco per ~4 s" } };
static const campo_t C_ASP[] = { { "giri", 0, 2, -1, 0, "i16", 10, "giri/min", 'V', CAMPO_MOTORI, "velocità regolata dal GD32" } };
static const campo_t C_PWM[] = { { "pwm", 0, 1, -1, 0, "u8", 1, "%", 'V', CAMPO_MOTORI, "potenza diretta della ventola" } };
static const campo_t C_POT[] = { { "potenza", 0, 4, -1, 0, "i32", 1, "%", 'V', CAMPO_MOTORI, "potenza del motore del lidar (68 = ~6 giri/s)" } };
static const campo_t C_LED[] = { { "modo", 0, 1, -1, 0, "u8", 1, "", 'V', CAMPO_TASTI, "0 spento, 1 azzurro, 4 giallo che respira, 6 giallo lento, 8 rosso veloce..." } };
static const campo_t C_MASCHERA[] = { { "maschera", 0, 3, -1, 0, "u8", 1, "", 'C', CAMPO_STATO, "quali sensori fermano il moto (all'avvio 00 ff ff)" } };

const messaggio_t MESSAGGI[] = {
	{ 0x15, "sensori", 0, "frame dei sensori, ogni 20 ms", CAMPI_SENSORI, N(CAMPI_SENSORI) },
	{ 0x06, "heartbeat", 1, "battito del SoC ogni 200 ms: senza, dopo 1,9 s il GD32 toglie potenza ai motori", NULL, 0 },
	{ 0x06, "heartbeat", 0, "battito del GD32, ogni 2 s", NULL, 0 },
	{ 0x0d, "keepalive", 1, "ogni 3 s: senza, dopo ~190 s il GD32 resetta tutto il robot", NULL, 0 },
	{ 0x07, "versione", 1, "richiesta della versione del firmware", NULL, 0 },
	{ 0x07, "versione", 0, "versione del firmware (ASCII, es. 2.0.1_19082727)", NULL, 0 },
	{ 0x08, "imu_zero", 1, "azzera l'angolo dell'IMU", NULL, 0 },
	{ 0x08, "imu_zero", 0, "conferma dell'azzeramento (dopo ~200 ms)", C_UNO, 1 },
	{ 0x0c, "sorveglia_soc", 1, "01 all'avvio, 00 prima della sospensione", C_UNO, 1 },
	{ 0x0c, "sorveglia_soc", 0, "conferma", C_UNO, 1 },
	{ 0x04, "sospensione", 1, "mette il GD32 in sospensione", NULL, 0 },
	{ 0x05, "risveglio", 0, "tasto premuto durante la sospensione: chiede conferma ogni 100 ms", NULL, 0 },
	{ 0x05, "risveglio", 1, "conferma del risveglio", NULL, 0 },
	{ 0x17, "imu_grezza", 1, "richiesta dati IMU grezzi (all'avvio)", C_UNO, 1 },
	{ 0x18, "funzioni_macchina", 1, "elenco delle funzioni (all'avvio)", NULL, 0 },
	{ 0x18, "funzioni_macchina", 0, "risposta: 04 00 00 00 3f 00 00 00", NULL, 0 },
	{ 0x19, "cliff_retromarcia", 1, "retromarcia automatica del GD32 sugli anti-caduta", C_BOOL, 1 },
	{ 0x65, "tipo_controllo", 1, "modalità dei motori", C_MODO, 1 },
	{ 0x66, "velocita", 1, "velocità v e omega, ogni 20 ms; il GD32 la applica dopo ~29 ms", C_VEL, 2 },
	{ 0x67, "velocita_ruote", 1, "velocità delle due ruote (solo in modalità 01)", C_RUOTE, 2 },
	{ 0x68, "aspirazione", 1, "aspirazione regolata", C_ASP, 1 },
	{ 0x69, "spazzola_laterale", 1, "spazzola laterale", C_SPAZZOLA, 1 },
	{ 0x6a, "spazzola_centrale", 1, "spazzola centrale", C_SPAZZOLA, 1 },
	{ 0x6b, "pompa_acqua", 1, "pompa dell'acqua: NON c'è il serbatoio, vietata", C_UNO, 1 },
	{ 0x71, "motore_lidar", 1, "potenza del motore del lidar", C_POT, 1 },
	{ 0x78, "cliff_ir", 1, "emettitori IR degli anti-caduta", C_BOOL, 1 },
	{ 0x7c, "maschera_sensori", 1, "maschera dei sensori del moto", C_MASCHERA, 1 },
	{ 0x85, "aspirazione_pwm", 1, "aspirazione a PWM diretto", C_PWM, 1 },
	{ 0x86, "aspirazione_pwm_bat", 1, "aspirazione a PWM compensato con la tensione", C_PWM, 1 },
	{ 0x8d, "led_tasto", 1, "LED del tasto", C_LED, 1 },
	{ 0x97, "alimentazione_lidar", 1, "alimentazione del lidar", C_BOOL, 1 },
	{ 0x9d, "alimentazione_sensori", 1, "sensori: 00 sulla base, 01 fuori", C_BOOL, 1 },
	{ 0xa2, "imu_stato_calibra", 1, "stato della calibrazione dell'IMU", NULL, 0 },
	{ 0xa2, "imu_stato_calibra", 0, "risposta", C_UNO, 1 },
};
const int N_MESSAGGI = N(MESSAGGI);

const messaggio_t *descrizione_messaggio(uint8_t cmd, int verso_gd32)
{
	for (int i = 0; i < N_MESSAGGI; i++)
		if (MESSAGGI[i].cmd == cmd && MESSAGGI[i].verso_gd32 == verso_gd32)
			return &MESSAGGI[i];
	return NULL;
}

int64_t descrizione_grezzo(const campo_t *c, const uint8_t *d)
{
	const uint8_t *b = d + c->byte;
	if (c->bit >= 0)
		return (b[0] >> c->bit) & ((1 << c->n_bit) - 1);
	switch (c->n_byte) {
	case 1:
		return c->tipo[0] == 'i' ? (int8_t)b[0] : b[0];
	case 2: {
		uint16_t u = (uint16_t)(b[0] | b[1] << 8);
		return c->tipo[0] == 'i' ? (int16_t)u : u;
	}
	case 4:
		return (int32_t)((uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24);
	default: {
		int64_t v = 0;
		for (int i = c->n_byte - 1; i >= 0; i--)
			v = v << 8 | b[i];
		return v;
	}
	}
}
