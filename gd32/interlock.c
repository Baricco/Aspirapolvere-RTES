/* interlock.c - Blocco di sicurezza dei motori: vedi interlock.h. */
#include "interlock.h"

#include <string.h>

void interlock_init(interlock_t *s, int lifted, int64_t max_age_ns)
{
	memset(s, 0, sizeof(*s));
	s->lifted = lifted;
	s->max_age_ns = max_age_ns;
	s->expected_cliff = GD32_CLIFF_ALL_VOID;
}

void interlock_set_expected_cliff(interlock_t *s, uint8_t cliff)
{
	s->expected_cliff = cliff & 0x0f;
}

int interlock_is_motor(uint8_t cmd)
{
	switch (cmd) {
	case GD32_VELOCITY:
	case GD32_WHEEL_SPEED:
	case GD32_SUCTION:
	case GD32_SIDE_BRUSH:
	case GD32_MAIN_BRUSH:
	case GD32_SUCTION_MODE:
	case GD32_SUCTION_PWM:
	case GD32_SUCTION_PWM_BATTERY:
		return 1;
	default:
		return 0;
	}
}

int interlock_is_forbidden(uint8_t cmd)
{
	return cmd == GD32_WATER_PUMP || cmd == GD32_SOC_POWER || cmd == GD32_SOC_RESTART;
}

static int trip(interlock_t *s, const char *reason, int64_t now)
{
	if (s->tripped)
		return 0;
	s->tripped = 1;
	s->reason = reason;
	s->tripped_ns = now;
	return 1;
}

/* Il frame dice "sollevato": anti-caduta come atteso (di regola tutti su vuoto), sensori alimentati, robot non
 * sulla base. */
static int frame_says_lifted(const interlock_t *s, const uint8_t *d)
{
	return (d[3] & 0x0f) == s->expected_cliff && (d[64] & GD32_CONTROL_SENSORS_ON) && (d[7] & 3) == 0;
}

int interlock_received(interlock_t *s, const gd32_frame_t *f, int64_t now)
{
	if (f->cmd != GD32_SENSORS || f->data_len < 96)
		return 0;
	s->has_sensors = 1;
	s->cliff = f->data[3];
	s->lifted_now = frame_says_lifted(s, f->data);
	s->last_sensors_ns = now;
	if (s->lifted_now) {
		s->seen_lifted = 1;
		return 0;
	}
	if (s->seen_lifted)
		return trip(s, "il robot non risulta più sollevato (anti-caduta, sensori spenti o sulla base)", now);
	return 0;
}

int interlock_check(interlock_t *s, int64_t now)
{
	if (s->seen_lifted && now - s->last_sensors_ns > s->max_age_ns)
		return trip(s, "frame dei sensori assenti", now);
	return 0;
}

static int all_zero(const uint8_t *d, size_t n)
{
	for (size_t i = 0; i < n; i++)
		if (d[i])
			return 0;
	return 1;
}

interlock_result_t interlock_permit(interlock_t *s, uint8_t cmd, const uint8_t *data, size_t n, int64_t now)
{
	if (interlock_is_forbidden(cmd)) {
		s->forbidden++;
		return INTERLOCK_FORBIDDEN;
	}
	if (!interlock_is_motor(cmd) || all_zero(data, n))
		return INTERLOCK_ALLOWED;
	/* motore diverso da zero: servono tutte le condizioni */
	interlock_check(s, now);
	if (s->lifted && !s->tripped && s->has_sensors && s->lifted_now &&
	    now - s->last_sensors_ns <= s->max_age_ns)
		return INTERLOCK_ALLOWED;
	s->refused++;
	return INTERLOCK_REFUSED;
}

size_t interlock_stop_frames(uint8_t *out, size_t size)
{
	static const uint8_t zero[8];
	static const struct { uint8_t cmd, n; } full_stop[] = {
		{ GD32_VELOCITY, 8 }, { GD32_WHEEL_SPEED, 8 }, { GD32_SIDE_BRUSH, 1 },
		{ GD32_MAIN_BRUSH, 1 }, { GD32_SUCTION, 2 }, { GD32_SUCTION_PWM, 1 },
	};
	size_t total = 0;
	for (size_t i = 0; i < sizeof full_stop / sizeof full_stop[0]; i++) {
		size_t k = gd32_build_frame(full_stop[i].cmd, zero, full_stop[i].n, out + total, size - total);
		if (!k)
			return 0;
		total += k;
	}
	return total;
}

int interlock_ground_limit(const gd32_sensors_t *s, int leaving_base, int32_t *v_tick, int32_t *w_tick)
{
	/* limiti per uscire dalla base: 0,1 m/s e 0,5 rad/s nelle unità di 0x66 */
	const int32_t V_BASE = (int32_t)(0.1 * GD32_SCALE_V), W_BASE = (int32_t)(0.5 * GD32_SCALE_W);
	int sensors_on = (s->control & GD32_CONTROL_SENSORS_ON) != 0;
	int cliff = s->cliff[0] | s->cliff[1] | s->cliff[2] | s->cliff[3];
	int all_void = s->cliff[0] & s->cliff[1] & s->cliff[2] & s->cliff[3];
	int reasons = 0;

	if (s->on_base) {
		reasons |= INTERLOCK_LIM_BASE;
		if (!leaving_base) {
			*v_tick = *w_tick = 0;
			return reasons;
		}
		if (*v_tick > V_BASE) *v_tick = V_BASE;
		if (*v_tick < -V_BASE) *v_tick = -V_BASE;
		if (*w_tick > W_BASE) *w_tick = W_BASE;
		if (*w_tick < -W_BASE) *w_tick = -W_BASE;
		return reasons;      /* sulla base anti-caduta e bumper non dicono niente (0f, sensori spenti) */
	}
	if (sensors_on && all_void) {
		*v_tick = *w_tick = 0;
		return INTERLOCK_LIM_LIFTED;
	}
	if (!sensors_on)
		reasons |= INTERLOCK_LIM_SENSORS;
	else {
		if (s->bumper_left || s->bumper_right)
			reasons |= INTERLOCK_LIM_BUMPER;
		if (cliff)
			reasons |= INTERLOCK_LIM_CLIFF;
	}
	if (reasons && *v_tick > 0)
		*v_tick = 0;
	return reasons;
}
