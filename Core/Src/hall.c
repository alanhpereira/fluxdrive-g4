/* Hall sensor decoder (hall.h). hall_update() runs in the 25 kHz ISR: -O2 even in a size-optimized build. It runs
 * from flash, not CCM (no room left there): about 0.3 us more per cycle, worst case, than it would from CCM */
#pragma GCC optimize("O2")

#include "hall.h"

#include <math.h>

#include "calib.h" /* CTRL_FS_HZ */
#include "main.h"
#include "params.h"

#define Q32 4294967296.0f
#define HALL_STOP_N 2500.0f /* cycles (100 ms) without an edge: standing still, the angle goes to the sector centre */
/* a state is taken once two samples in a row agree (switching glitches last less than a period). The edge then
 * happened on average 1.5 periods before it was taken */
#define EDGE_LAG 1.5f

volatile uint8_t g_hall_state;
volatile uint16_t g_hall_bad_n;
volatile uint32_t g_hall_bad_total;

/* sector table, from g_params.hall_edge, Q32 electrical. Index = state 1..6 */
static uint32_t s_begin[8], s_ctr[8], s_half[8];
static uint8_t s_next[8], s_prev[8];
/* 0 no table, 1 in use, 2 rebuilt by main: the ISR restarts the decoder and sets 1 */
static volatile uint8_t s_ok;

/* decoder, ISR only */
static uint8_t s_st;		  /* accepted state */
static uint8_t s_cand, s_cand_n; /* a different state seen in s_cand_n samples in a row */
static int8_t s_dir;		  /* direction of the last edge, 0 = standing still */
static uint32_t s_ph;		  /* angle now, Q32 electrical */
static float s_we;			  /* speed, electrical turns per cycle (signed) */
static float s_t = HALL_STOP_N; /* cycles since the last edge */
static float s_dt[6];		  /* the last edge intervals in the same direction, cycles */
static uint8_t s_ri, s_n;

/* HALL1 (PC2) = bit 0, HALL2 (PC1) = bit 1, HALL3 (PC0) = bit 2 */
static inline __attribute__((always_inline)) uint8_t read_pins(void) {
	const uint32_t idr = GPIOC->IDR;
	return (uint8_t)(((idr >> 2) & 1u) | (idr & 2u) | ((idr & 1u) << 2));
}

static inline __attribute__((always_inline)) int valid_state(uint8_t s) {
	return s >= 1 && s <= 6;
}

void hall_table_apply(void) {
	s_ok = 0; /* the ISR leaves the table alone from here */
	__DSB();
	if (!(g_params.flags & PF_HALL_VALID)) return;
	uint8_t ord[6];
	for (int s = 1; s <= 6; s++) s_begin[s] = (uint32_t)g_params.hall_edge[s] << 16;
	for (int k = 0; k < 6; k++) { /* states in the order they begin, going forward */
		uint8_t s = (uint8_t)(k + 1);
		int j = k;
		while (j > 0 && s_begin[ord[j - 1]] > s_begin[s]) {
			ord[j] = ord[j - 1];
			j--;
		}
		ord[j] = s;
	}
	for (int k = 0; k < 6; k++) {
		const uint8_t a = ord[k], b = ord[(k + 1) % 6];
		const uint32_t w = s_begin[b] - s_begin[a]; /* wraps round for the last one */
		if (w == 0) return;							/* two states at one angle: not a table */
		s_next[a] = b;
		s_prev[b] = a;
		s_half[a] = w / 2u;
		s_ctr[a] = s_begin[a] + w / 2u;
	}
	__DSB();
	s_ok = 2;
}

int hall_table_ok(void) {
	return s_ok != 0;
}

void hall_init(void) {
	/* polled, not interrupt driven: the EXTI lines the .ioc sets up stay off (with nothing on J13 they would fire
	 * on noise). Pull-ups, since open-collector sensors need them and the board has none */
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	EXTI->PR1 = EXTI_PR1_PIF0 | EXTI_PR1_PIF1 | EXTI_PR1_PIF2;
	uint32_t pupd = GPIOC->PUPDR & ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD2);
	GPIOC->PUPDR = pupd | GPIO_PUPDR_PUPD0_0 | GPIO_PUPDR_PUPD1_0 | GPIO_PUPDR_PUPD2_0;
	s_st = read_pins();
	g_hall_state = s_st;
	hall_table_apply();
}

/* no speed known (start, reversal, timeout, a jump): sit in the middle of sector s. Rare: in flash */
static __attribute__((noinline)) void restart(uint8_t s) {
	s_dir = 0;
	s_we = 0.0f;
	s_t = HALL_STOP_N;
	s_n = 0;
	s_ri = 0;
	if (valid_state(s)) s_ph = s_ctr[s];
}

/* a new valid state st was taken. At most a few thousand times a second: in flash, CCM is full */
static __attribute__((noinline)) void on_edge(uint8_t st) {
	const uint8_t a = s_st;
	int8_t d = 0;
	uint32_t at = 0;
	if (valid_state(a)) {
		if (st == s_next[a]) {
			d = 1;
			at = s_begin[st];
		}
		else if (st == s_prev[a]) {
			d = -1;
			at = s_begin[a];
		}
	}
	s_st = st;
	if (d == 0) { /* from an invalid state, or a jump over a sector: start again in this one */
		if (valid_state(a)) g_hall_bad_total++;
		restart(st);
		return;
	}
	if (d != s_dir)
		s_n = 0; /* first edge from standstill, or a reversal: no interval to go by */
	else if (s_t < HALL_STOP_N) {
		s_dt[s_ri] = s_t;
		if (++s_ri >= 6) s_ri = 0;
		if (s_n < 6) s_n++;
		/* one electrical turn over the last six intervals: the placement error of each sensor cancels */
		if (s_n >= 6)
			s_we = (float)d / (s_dt[0] + s_dt[1] + s_dt[2] + s_dt[3] + s_dt[4] + s_dt[5]);
		else
			s_we = (float)d / (6.0f * s_t);
	}
	if (s_n == 0) s_we = 0.0f;
	s_dir = d;
	s_t = EDGE_LAG;
	s_ph = at + (uint32_t)(int32_t)(s_we * (EDGE_LAG * Q32));
}

/* per cycle, but in flash: CCM is full. The path that runs on most cycles is short (no edge: one compare, one add) */
int hall_update(uint32_t* th, float* vel) {
	const uint8_t raw = read_pins();
	if (raw == s_st)
		s_cand_n = 0;
	else if (raw == s_cand) {
		if (s_cand_n < 255) s_cand_n++;
	}
	else {
		s_cand = raw;
		s_cand_n = 1;
	}
	const int edge = s_cand_n >= 2;
	const uint8_t st = edge ? s_cand : s_st;
	if (edge) {
		s_cand_n = 0;
		g_hall_state = st;
	}
	const int valid = valid_state(st);
	if (valid)
		g_hall_bad_n = 0;
	else {
		if (g_hall_bad_n < UINT16_MAX) g_hall_bad_n++;
		if (edge) g_hall_bad_total++;
	}

	const uint8_t ok = s_ok;
	if (ok != 1) {
		s_st = st;
		if (ok == 0) { /* no table: the state is still tracked (calibration, diagnostics) */
			*th = 0;
			*vel = 0.0f;
			return 0;
		}
		restart(st);
		s_ok = 1;
	}

	if (s_t < HALL_STOP_N) s_t += 1.0f;
	if (edge) {
		if (valid)
			on_edge(st);
		else
			s_st = st; /* an invalid state: carry on at the last speed, g_hall_bad_n counts */
	}
	else if (s_dir) {
		if (s_t >= HALL_STOP_N)
			restart(s_st);
		else {
			/* the next edge is late: the rotor is slower than it was, at most one sector in s_t */
			if (fabsf(s_we) * 6.0f * s_t > 1.0f) s_we = (float)s_dir / (6.0f * s_t);
			s_ph += (uint32_t)(int32_t)(s_we * Q32);
		}
	}
	const int in = valid_state(s_st);
	if (in) {
		/* never past the sector's ends: an edge that has not come means the rotor is not there yet */
		const uint32_t c = s_ctr[s_st];
		const int32_t h = (int32_t)s_half[s_st];
		const int32_t off = (int32_t)(s_ph - c);
		if (off > h)
			s_ph = c + (uint32_t)h;
		else if (off < -h)
			s_ph = c - (uint32_t)h;
	}
	*th = s_ph;
	*vel = s_we * (CTRL_FS_HZ / (float)g_params.pole_pairs);
	return in;
}

int hall_in_sector(uint32_t th, uint32_t margin) {
	const uint8_t c = s_st;
	if (s_ok != 1 || !valid_state(c)) return 1; /* nothing to check against */
	const int32_t off = (int32_t)(th - s_ctr[c]);
	const uint32_t a = off < 0 ? 0u - (uint32_t)off : (uint32_t)off;
	return a <= s_half[c] + margin;
}
