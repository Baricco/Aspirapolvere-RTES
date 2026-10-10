/* parametri.c - Parametri tarati del simulatore: vedi parametri.h. */
#include "parametri.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Default = i valori di sim/parametri.txt al momento della taratura (2026-10-05). */
sim_parametri_t par = {
	.gd32_periodo_ms = 20.00056,
	.arrivo_n = 13,
	.arrivo_prob = { 0, .01, .05, .10, .25, .50, .75, .90, .95, .99, .999, .9999, 1 },
	.arrivo_ms = { 0, .293, .324, .329, .334, .342, .352, .361, .365, .375, .407, .8, 5.0 },
	.ruote_ritardo_ms = 29,
	.rampa_v_su = 16584, .rampa_v_giu = 16761, .rampa_w_su = 2214, .rampa_w_giu = 2223,
	.tau_v_terra = 0.10, .tau_w_terra = 0.073,
	.ruote_finestra_ms = 70,
	.ruote_timeout_hb = 2.04, .ruote_inerzia = 1879,
	.corrente_ruota_base = 64, .corrente_ruota_picchi = 290, .corrente_ruota_avvio = 772,
	.imu_deriva_gradi_s = -0.000083, .imu_scala = 0.9723, .imu_rumore_velocita = 0.5, .imu_beccheggio = -55, .imu_rollio = 38,
	.corrente_riposo_laterale = 2, .corrente_riposo_centrale = 58, .corrente_riposo_aspirazione = 14, .byte84 = 2,
	.temperatura_c = 33,
	.corrente_centrale_accesa = 202, .corrente_centrale_sblocco = 617,
	.aspirazione_giri_per_pwm = 219, .aspirazione_corrente_max = 1878, .aspirazione_giri_max = 18200,
	.aspirazione_esponente = 2.27,
	.lidar_a = 0.0856, .lidar_b = 0.167,
	.lidar_tau_su = 0.18, .lidar_tau_giu = 0.24, .lidar_tau_spento = 0.54,
	.lidar_misure_s = 2037,
	.lidar_rumore_mm = { 7.2, 6.6, 6.8, 8.8, 12.3, 16.3, 36.8 },
	.lidar_buchi = 0.0229, .lidar_intensita = 131, .lidar_intensita_dev = 35, .lidar_portata = 8.37,
	.batteria_utile_ah = 2.45, .batteria_totale_ah = 3.2,
};

void parametri_frame_default(void)
{
	static const char *hex = "0100000f04000000a21f0000000000009037000000000000cd3800000000000000000000000000000000000000000b000000dbff000000006400000000000000e20c00000000000000000000000002003b000d00020000000319000000000000";
	for (int i = 0; i < 96; i++) {
		unsigned b;
		sscanf(hex + 2 * i, "%2x", &b);
		par.frame_riposo[i] = (uint8_t)b;
	}
}

static int numeri(const char *s, double *out, int max)
{
	int n = 0;
	char *fine;
	while (n < max) {
		double v = strtod(s, &fine);
		if (fine == s)
			break;
		out[n++] = v;
		s = fine;
	}
	return n;
}

int parametri_leggi(const char *percorso)
{
	FILE *f = fopen(percorso, "r");
	if (!f)
		return -1;
	char riga[1024];
	while (fgets(riga, sizeof riga, f)) {
		char *c = strchr(riga, '#');
		if (c)
			*c = 0;
		char *uguale = strchr(riga, '=');
		if (!uguale)
			continue;
		*uguale = 0;
		char chiave[64];
		if (sscanf(riga, "%63s", chiave) != 1)
			continue;
		const char *v = uguale + 1;
		double x[PAR_MAX_QUANTILI];
		int n = numeri(v, x, PAR_MAX_QUANTILI);
#define UNO(nome, campo) else if (!strcmp(chiave, nome) && n >= 1) par.campo = x[0]
#define DUE(nome, c1, c2) else if (!strcmp(chiave, nome) && n >= 2) { par.c1 = x[0]; par.c2 = x[1]; }
		if (!strcmp(chiave, "arrivo_probabilita") && n >= 2) {
			for (int i = 0; i < n; i++)
				par.arrivo_prob[i] = x[i];
			par.arrivo_n = n;
		} else if (!strcmp(chiave, "arrivo_ritardo_ms") && n >= 2) {
			for (int i = 0; i < n && i < par.arrivo_n; i++)
				par.arrivo_ms[i] = x[i];
		} else if (!strcmp(chiave, "frame_riposo")) {
			char hex[200];
			if (sscanf(v, "%199s", hex) == 1 && strlen(hex) == 192)
				for (int i = 0; i < 96; i++) {
					unsigned b;
					sscanf(hex + 2 * i, "%2x", &b);
					par.frame_riposo[i] = (uint8_t)b;
				}
		} else if (!strcmp(chiave, "lidar_rumore_mm") && n >= 7) {
			for (int i = 0; i < 7; i++)
				par.lidar_rumore_mm[i] = x[i];
		}
		UNO("gd32_periodo_ms", gd32_periodo_ms);
		UNO("ruote_ritardo_ms", ruote_ritardo_ms);
		DUE("ruote_rampa_v", rampa_v_su, rampa_v_giu)
		DUE("ruote_rampa_w", rampa_w_su, rampa_w_giu)
		DUE("ruote_tau_terra_s", tau_v_terra, tau_w_terra)
		UNO("ruote_finestra_misura_ms", ruote_finestra_ms);
		UNO("ruote_timeout_heartbeat_s", ruote_timeout_hb);
		UNO("ruote_inerzia", ruote_inerzia);
		UNO("ruote_corrente_base_ma", corrente_ruota_base);
		UNO("ruote_corrente_picchi_ma", corrente_ruota_picchi);
		UNO("ruote_corrente_avvio_ma", corrente_ruota_avvio);
		UNO("imu_deriva_gradi_s", imu_deriva_gradi_s);
		UNO("imu_rumore_velocita", imu_rumore_velocita);
		UNO("imu_scala", imu_scala);
		UNO("imu_beccheggio", imu_beccheggio);
		UNO("imu_rollio", imu_rollio);
		UNO("corrente_riposo_laterale_ma", corrente_riposo_laterale);
		UNO("corrente_riposo_centrale_ma", corrente_riposo_centrale);
		UNO("corrente_riposo_aspirazione_ma", corrente_riposo_aspirazione);
		UNO("byte84_riposo", byte84);
		UNO("temperatura_c", temperatura_c);
		UNO("centrale_corrente_accesa_ma", corrente_centrale_accesa);
		UNO("centrale_corrente_sblocco_ma", corrente_centrale_sblocco);
		UNO("aspirazione_giri_per_pwm", aspirazione_giri_per_pwm);
		DUE("aspirazione_corrente_max", aspirazione_corrente_max, aspirazione_giri_max)
		UNO("aspirazione_esponente", aspirazione_esponente);
		DUE("lidar_giri_per_potenza", lidar_a, lidar_b)
		UNO("lidar_tau_salita_s", lidar_tau_su);
		UNO("lidar_tau_discesa_s", lidar_tau_giu);
		UNO("lidar_tau_spento_s", lidar_tau_spento);
		UNO("lidar_misure_al_secondo", lidar_misure_s);
		UNO("lidar_buchi_isolati", lidar_buchi);
		DUE("lidar_intensita", lidar_intensita, lidar_intensita_dev)
		UNO("lidar_portata_m", lidar_portata);
		UNO("batteria_capacita_utile_ah", batteria_utile_ah);
		UNO("batteria_capacita_totale_ah", batteria_totale_ah);
#undef UNO
#undef DUE
	}
	fclose(f);
	return 0;
}

double parametri_ritardo_arrivo(double u)
{
	int n = par.arrivo_n;
	if (n < 2)
		return 0;
	for (int i = 1; i < n; i++)
		if (u <= par.arrivo_prob[i]) {
			double p0 = par.arrivo_prob[i - 1], p1 = par.arrivo_prob[i];
			double f = p1 > p0 ? (u - p0) / (p1 - p0) : 0;
			return (par.arrivo_ms[i - 1] + f * (par.arrivo_ms[i] - par.arrivo_ms[i - 1])) / 1000.0;
		}
	return par.arrivo_ms[n - 1] / 1000.0;
}
