/* lidar_scan.c - Giri completi del lidar e regolazione della velocità: vedi lidar_scan.h. */
#include <math.h>

#include "lidar_scan.h"

#include <string.h>

void lidar_assembler_init(lidar_assembler_t *a, int offset_cdeg)
{
	memset(a, 0, sizeof(*a));
	a->last_start = -1;
	a->offset_cdeg = offset_cdeg;
}

static uint16_t u16be(const uint8_t *b) { return (uint16_t)(b[0] << 8 | b[1]); }

int lidar_assemble(lidar_assembler_t *a, const lidar_packet_t *p, int64_t t, lidar_scan_t *scan)
{
	if (p->cmd == LIDAR_SPEED && p->n_param >= 1) {
		a->rev_s = p->param[0] * 0.05;
		a->t_speed = t;
		return 0;
	}
	if (p->cmd != LIDAR_SCAN_NEW || p->n_param < 5)
		return 0;
	double speed = p->param[0] * 0.05;
	int start = u16be(p->param + 3);
	int n = (p->n_param - 5) / 3;
	a->rev_s = speed;
	a->t_speed = t;
	a->sectors++;

	int delivered = 0;
	lidar_scan_t *c = &a->current;
	if (a->last_start >= 0 && start <= a->last_start) {
		/* l'angolo è tornato indietro: il giro precedente è finito */
		if (c->sectors >= 12) {
			c->rev_s = c->sectors ? a->speed_sum / c->sectors : 0;
			c->number = ++a->revolutions;
			memcpy(scan, c, sizeof(*c) - sizeof(c->p) + sizeof(c->p[0]) * c->n);
			delivered = 1;
		} else {
			a->discarded_scans++;
		}
		c->n = 0;
		c->sectors = 0;
		a->speed_sum = 0;
	}
	if (c->sectors == 0)
		c->t_start_ns = t;
	c->t_end_ns = t;
	c->sectors++;
	a->speed_sum += speed;
	a->last_start = start;
	for (int i = 0; i < n && c->n < LIDAR_SCAN_MAX_POINTS; i++) {
		const uint8_t *q = p->param + 5 + 3 * i;
		int angle_now = start + (i * 2250) / n - a->offset_cdeg;
		angle_now %= 36000;
		if (angle_now < 0)
			angle_now += 36000;
		lidar_point_t *pt = &c->p[c->n++];
		pt->angle_cdeg = (uint16_t)angle_now;
		pt->intensity = q[0];
		pt->distance_mm = (uint16_t)((u16be(q + 1) + 2) / 4);
	}
	return delivered;
}

void lidar_regulator_init(lidar_regulator_t *r, double target)
{
	memset(r, 0, sizeof(*r));
	r->target = target;
}

int lidar_regulate(lidar_regulator_t *r, double measured)
{
	double e = r->target - measured;
	r->integral += e;
	if (r->integral > 20)
		r->integral = 20;
	if (r->integral < -20)
		r->integral = -20;
	double p = 60 + r->integral + 10 * e + 10 * (e - r->prev_error);
	r->prev_error = e;
	if (r->target <= 0)
		p = 0;
	r->power = p < 0 ? 0 : p > 100 ? 100 : (int)(p + 0.5);
	return r->power;
}

void lidar_point_xy(const lidar_point_t *p, double *x, double *y)
{
	/* angolo del lidar: 0 = dietro, ORARIO; nel sistema del robot b = 180° - angolo (0 = davanti, antiorario) */
	double b = (180.0 - p->angle_cdeg / 100.0) * M_PI / 180.0;
	double r = p->distance_mm / 1000.0;
	/* il punto è a distanza r lungo b, spostato di LIDAR_OPTICAL_M a destra del raggio (triangolatore) */
	*x = LIDAR_X_M + r * cos(b) + LIDAR_OPTICAL_M * sin(b);
	*y = LIDAR_Y_M + r * sin(b) - LIDAR_OPTICAL_M * cos(b);
}
