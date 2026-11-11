/* contacts.c - Registro degli scatti di anti-caduta e bumper, e degli incastri (vedi contacts.h). */
#include <stdlib.h>
#include <string.h>

#include "contacts.h"

void contacts_init(contacts_t *c)
{
	memset(c, 0, sizeof *c);
}

static void reset_counters(contacts_t *c)
{
	memset(c->count, 0, sizeof c->count);
	memset(c->reported, 0, sizeof c->reported);
}

int contacts_frame(contacts_t *c, const gd32_sensors_t *s, int64_t t, const drv_pose_t *pose, int stuck)
{
	int sensors_ok = (s->control & GD32_CONTROL_SENSORS_ON) != 0;
	int all_void = s->cliff[0] && s->cliff[1] && s->cliff[2] && s->cliff[3];
	if (!sensors_ok || s->on_base || all_void) {        /* i valori non significano niente: si riparte da zero */
		reset_counters(c);
		return 0;
	}
	/* robot inclinato: gli anti-caduta non danno eventi (vedi contacts.h) */
	int tilted = abs(s->angle[1]) > CONTACTS_TILT_CDEG || abs(s->angle[2]) > CONTACTS_TILT_CDEG;
	int active[CONTACT_SENSORS] = { s->cliff[0] && !tilted, s->cliff[1] && !tilted, s->cliff[2] && !tilted,
	                                  s->cliff[3] && !tilted, s->bumper_left, s->bumper_right, stuck != 0 };
	int new_events = 0;
	for (int k = 0; k < CONTACT_SENSORS; k++) {
		if (!active[k]) {
			c->count[k] = 0;
			c->reported[k] = 0;
			continue;
		}
		if (c->count[k]++ == 0) {
			c->t_start[k] = t;
			c->has_pose[k] = pose != NULL;
			if (pose)
				c->start_pose[k] = *pose;
		}
		int threshold = k < CONTACT_BUMPER_LEFT ? CONTACTS_VOID_FRAMES
		           : k == CONTACT_STUCK ? CONTACTS_STUCK_FRAMES : CONTACTS_BUMPER_FRAMES;
		if (!c->reported[k] && c->count[k] >= threshold) {
			c->reported[k] = 1;
			uint32_t n = ++c->published.last;
			drv_contact_t e = { .number = n, .sensor = (uint8_t)k, .t_ns = c->t_start[k], .pose_valid = c->has_pose[k] };
			if (c->has_pose[k])
				e.pose = c->start_pose[k];
			c->published.events[n % CONTACTS_N] = e;
			new_events = 1;
		}
	}
	return new_events;
}
