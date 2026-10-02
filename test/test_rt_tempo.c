/* Test delle conversioni e delle funzioni di tempo (rt/rt_tempo.h). */
#include "test.h"
#include "../rt/rt_tempo.h"

static void conversioni_andata_ritorno(void)
{
	int64_t valori[] = { 0, 1, 999999999, RT_S(1), RT_S(1) + 1, RT_MS(1500), RT_S(100000) + 7 };
	for (unsigned i = 0; i < sizeof(valori) / sizeof(valori[0]); i++) {
		struct timespec ts = rt_ns_a_timespec(valori[i]);
		CONTROLLA(ts.tv_nsec >= 0 && ts.tv_nsec < RT_NS_PER_S);
		CONTROLLA(rt_timespec_a_ns(&ts) == valori[i]);
	}
}

static void negativi_normalizzati(void)
{
	/* -1 ns deve diventare { -1 s, 999999999 ns }, non { 0 s, -1 ns } */
	struct timespec ts = rt_ns_a_timespec(-1);
	CONTROLLA(ts.tv_sec == -1);
	CONTROLLA(ts.tv_nsec == 999999999);
	CONTROLLA(rt_timespec_a_ns(&ts) == -1);
}

static void macro_unita(void)
{
	CONTROLLA(RT_US(3) == 3000);
	CONTROLLA(RT_MS(20) == 20000000);
	CONTROLLA(RT_S(2) == 2000000000LL);
}

static void orologio_monotono(void)
{
	int64_t a = rt_adesso();
	int64_t b = rt_adesso();
	CONTROLLA(b >= a);
}

static void dormi_fino_a_non_torna_prima(void)
{
	int64_t obiettivo = rt_adesso() + RT_MS(5);
	rt_dormi_fino_a(obiettivo);
	CONTROLLA(rt_adesso() >= obiettivo);
	/* un istante già passato deve tornare subito */
	int64_t prima = rt_adesso();
	rt_dormi_fino_a(prima - RT_MS(1));
	CONTROLLA(rt_adesso() - prima < RT_MS(1));
}

int main(void)
{
	ESEGUI(conversioni_andata_ritorno);
	ESEGUI(negativi_normalizzati);
	ESEGUI(macro_unita);
	ESEGUI(orologio_monotono);
	ESEGUI(dormi_fino_a_non_torna_prima);
	return TEST_FINE();
}
