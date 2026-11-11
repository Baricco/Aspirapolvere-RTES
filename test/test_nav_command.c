/* Test dell'impacchettamento del comando dell'app per la navigazione in 64 bit (tasks/vacuum/navigation.h,
 * docs/app.md sezione 9.4): numero 16 bit, tipo 4, v in mm/s 16, omega in mrad/s 16, spazzole 1, aspirazione 1. */
#include <math.h>

#include "test.h"
#include "../tasks/vacuum/navigation.h"

static void posizione_dei_campi(void)
{
	navigation_command_t c = { .num = 0xABCD };
	CONTROLLA(pack_navigation_command(&c) == 0xABCDull);

	c = (navigation_command_t){ .type = NAV_COMMAND_REMOTE_RELEASE };
	CONTROLLA(pack_navigation_command(&c) == (uint64_t)NAV_COMMAND_REMOTE_RELEASE << 16);

	c = (navigation_command_t){ .v = 0.001 };
	CONTROLLA(pack_navigation_command(&c) == 1ull << 20);

	c = (navigation_command_t){ .omega = 0.001 };
	CONTROLLA(pack_navigation_command(&c) == 1ull << 36);

	c = (navigation_command_t){ .brushes = true };
	CONTROLLA(pack_navigation_command(&c) == 1ull << 52);

	c = (navigation_command_t){ .suction = true };
	CONTROLLA(pack_navigation_command(&c) == 1ull << 53);

	/* tutto insieme sta in 54 bit */
	c = (navigation_command_t){ .num = 0xFFFF, .type = 15, .v = -0.001, .omega = -0.001, .brushes = true, .suction = true };
	CONTROLLA(pack_navigation_command(&c) == (1ull << 54) - 1);
}

static void andata_e_ritorno(void)
{
	navigation_command_t casi[] = {
		{ .num = 1, .type = NAV_COMMAND_START },
		{ .num = 2, .type = NAV_COMMAND_PAUSE },
		{ .num = 3, .type = NAV_COMMAND_RESUME },
		{ .num = 4, .type = NAV_COMMAND_HOME },
		{ .num = 5, .type = NAV_COMMAND_REMOTE_TAKE },
		{ .num = 6, .type = NAV_COMMAND_REMOTE_MOVE, .v = 0.35, .omega = -1.2 },
		{ .num = 7, .type = NAV_COMMAND_REMOTE_MOVE, .v = -0.35, .omega = 1.2 },
		{ .num = 8, .type = NAV_COMMAND_REMOTE_MOVE, .v = 0.123, .omega = 0.001 },
		{ .num = 9, .type = NAV_COMMAND_REMOTE_CLEANING, .brushes = true, .suction = false },
		{ .num = 10, .type = NAV_COMMAND_REMOTE_CLEANING, .brushes = false, .suction = true },
		{ .num = 65535, .type = NAV_COMMAND_REMOTE_KEEPALIVE },
		{ .num = 300, .type = NAV_COMMAND_REMOTE_RELEASE },
	};
	for (unsigned i = 0; i < sizeof(casi) / sizeof(casi[0]); i++) {
		navigation_command_t r = unpack_navigation_command(pack_navigation_command(&casi[i]));
		CONTROLLA(r.num == casi[i].num);
		CONTROLLA(r.type == casi[i].type);
		CONTROLLA(fabs(r.v - casi[i].v) < 1e-9);
		CONTROLLA(fabs(r.omega - casi[i].omega) < 1e-9);
		CONTROLLA(r.brushes == casi[i].brushes);
		CONTROLLA(r.suction == casi[i].suction);
	}
}

static void velocita_arrotondate_e_limitate(void)
{
	/* arrotondate al millimetro al secondo */
	navigation_command_t c = { .v = 0.1004, .omega = -0.1006 };
	navigation_command_t r = unpack_navigation_command(pack_navigation_command(&c));
	CONTROLLA(fabs(r.v - 0.100) < 1e-9);
	CONTROLLA(fabs(r.omega + 0.101) < 1e-9);

	/* valori enormi o non numeri non escono dai loro 16 bit: gli altri campi restano giusti */
	double strani[] = { 1e9, -1e9, INFINITY, -INFINITY, NAN };
	for (unsigned i = 0; i < sizeof(strani) / sizeof(strani[0]); i++) {
		c = (navigation_command_t){ .num = 77, .type = NAV_COMMAND_REMOTE_MOVE, .v = strani[i], .omega = strani[i], .suction = true };
		r = unpack_navigation_command(pack_navigation_command(&c));
		CONTROLLA(r.num == 77);
		CONTROLLA(r.type == NAV_COMMAND_REMOTE_MOVE);
		CONTROLLA(fabs(r.v) <= 32.767 + 1e-9);
		CONTROLLA(fabs(r.omega) <= 32.767 + 1e-9);
		CONTROLLA(!r.brushes && r.suction);
	}
}

static void tipi_in_quattro_bit(void)
{
	/* il tipo occupa 4 bit: l'enum non deve superare 16 valori */
	CONTROLLA(NAV_COMMAND_REMOTE_RELEASE <= NAV_COMMAND_TYPE_MASK);
	CONTROLLA(NAV_COMMAND_NONE == 0);
	/* il comando azzerato è "nessun comando" numero 0 */
	navigation_command_t r = unpack_navigation_command(0);
	CONTROLLA(r.num == 0 && r.type == NAV_COMMAND_NONE && r.v == 0 && r.omega == 0);
}

int main(void)
{
	ESEGUI(posizione_dei_campi);
	ESEGUI(andata_e_ritorno);
	ESEGUI(velocita_arrotondate_e_limitate);
	ESEGUI(tipi_in_quattro_bit);
	return TEST_FINE();
}
