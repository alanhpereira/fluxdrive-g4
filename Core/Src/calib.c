#include "calib.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bridgeCtrl.h" /* checkAndClearBridgeFault(), disableBridge(), initBridge() */
#include "tim.h"
#include "usart.h"
#include "mt6835.h"
#include "pi_ctrl.h"

volatile ctrl_mode_t g_mode = CTRL_OFF;
volatile float g_vd_cmd, g_vq_cmd;
volatile uint32_t g_theta_cmd;
volatile uint8_t g_telem_en = 1;
volatile float g_rec[REC_N];
volatile uint16_t g_rec_n;
volatile uint8_t g_rec_state, g_rec_use_q;
volatile pi_t g_pi_d, g_pi_q;
volatile float g_id_ref, g_iq_ref;

void pi_apply(void) {
	const float ts = 1.0f / CTRL_FS_HZ;
	g_pi_d.kp = g_params.kp_d;
	g_pi_d.ki_ts = g_params.ki_d * ts;
	g_pi_q.kp = g_params.kp_q;
	g_pi_q.ki_ts = g_params.ki_q * ts;
}

/* ------------------------------------------------------------------ UART RX (single bytes, IRQ) */
#define RXQ 32
static volatile uint8_t rxq[RXQ];
static volatile uint8_t rxh, rxt;
static uint8_t rxb;

void cmd_init(void) {
	HAL_UART_Receive_IT(&huart1, &rxb, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* h) {
	if (h == &huart1) {
		uint8_t n = (uint8_t)((rxh + 1) % RXQ);
		if (n != rxt) {
			rxq[rxh] = rxb;
			rxh = n;
		}
		HAL_UART_Receive_IT(&huart1, &rxb, 1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* h) {
	if (h == &huart1) HAL_UART_Receive_IT(&huart1, &rxb, 1); /* overrun/noise must not kill RX */
}

static int rx_pop(uint8_t* c) {
	if (rxt == rxh) return 0;
	*c = rxq[rxt];
	rxt = (uint8_t)((rxt + 1) % RXQ);
	return 1;
}

/* ------------------------------------------------------------------ text output */
static void uart_wait_idle(void) {
	uint32_t t0 = HAL_GetTick();
	while (huart1.gState != HAL_UART_STATE_READY && HAL_GetTick() - t0 < 50) {
	}
}

/* only call while telemetry is paused (begin()) */
static void say(const char* fmt, ...) {
	char b[220];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(b, sizeof b, fmt, ap);
	va_end(ap);
	if (n > 0) HAL_UART_Transmit(&huart1, (uint8_t*)b, (uint16_t)(n < (int)sizeof b ? n : (int)sizeof b - 1), 100);
}

static void begin(void) {
	g_telem_en = 0;
	uart_wait_idle();
}

/* ramp the current references to zero, then switch off (100 ms) */
static void run_stop(void) {
	for (int i = 0; i < 20; i++) {
		g_id_ref *= 0.8f;
		g_iq_ref *= 0.8f;
		HAL_Delay(5);
	}
	g_mode = CTRL_OFF;
	g_id_ref = 0.0f;
	g_iq_ref = 0.0f;
}

static void finish(void) {
	if (g_mode == CTRL_RUN) run_stop();
	for (int i = 0; i < 20; i++) { /* ramp voltage down instead of stepping it */
		g_vd_cmd *= 0.8f;
		g_vq_cmd *= 0.8f;
		HAL_Delay(5);
	}
	g_vd_cmd = 0;
	g_vq_cmd = 0;
	g_mode = CTRL_OFF;
	g_telem_en = 1;
}

/* ------------------------------------------------------------------ helpers */
/* returns an error string if the routine must stop, NULL if all is well */
static const char* chk(void) {
	uint8_t c;
	while (rx_pop(&c))
		if (c == 'x') return "aborted by user";
	if (checkAndClearBridgeFault() != 0) return "bridge fault";
	if (g_vin < g_params.vbus_min) return "bus voltage too low";
	if (fabsf(g_id) > g_params.max_current || fabsf(g_iq) > g_params.max_current) return "overcurrent";
	return NULL;
}

static const char* precheck(void) {
	if (!(TIM1->BDTR & TIM_BDTR_MOE)) return "bridge disabled (fault latched?)";
	if (g_params.rated_current <= 0.0f || g_params.max_current <= 0.0f) return "set rated/imax first (s rated 1.0)";
	return chk();
}

/* signed difference of two 21-bit encoder readings */
static int32_t enc_delta(uint32_t now, uint32_t prev) {
	return (int32_t)((now - prev) << 11) >> 11;
}

static uint32_t enc_avg(int n, int ms) {
	uint32_t ref = hMT6835.angle;
	int64_t acc = 0;
	for (int i = 0; i < n; i++) {
		acc += enc_delta(hMT6835.angle, ref);
		HAL_Delay(ms);
	}
	return (ref + (uint32_t)(int32_t)(acc / n)) & 0x1FFFFFu;
}

static float avg_axis(int use_q, int n) {
	float a = 0.0f;
	for (int i = 0; i < n; i++) {
		a += use_q ? g_iq : g_id;
		HAL_Delay(1);
	}
	return a / (float)n;
}

/* raise Vd (20 mV every 5 ms) at theta = 0 until Id reaches i_target; rotor aligns on the way */
static const char* ramp_d_to_current(float i_target, float* v_out) {
	const float vmax = 0.8f * g_vin / SQRT3;
	float v = 0.0f;
	while (g_id < i_target) {
		v += 0.02f;
		if (v > vmax) return "voltage limit hit before reaching target current (wiring? current sign?)";
		g_vd_cmd = v;
		HAL_Delay(5);
		const char* e = chk();
		if (e) return e;
	}
	*v_out = v;
	return NULL;
}

/* time between the 75% and 25% crossings of a decaying current is tau * ln(3);
 * measuring a difference makes the fixed one-period delay of the loop cancel out */
static float decay_tau(const float* x, int n, float i0, float fs) {
	const float hi = 0.75f * i0, lo = 0.25f * i0;
	float ta = -1.0f;
	for (int k = 1; k < n; k++) {
		if (ta < 0.0f && x[k] <= hi && x[k - 1] > hi) ta = (float)(k - 1) + (x[k - 1] - hi) / (x[k - 1] - x[k]);
		if (ta >= 0.0f && x[k] <= lo && x[k - 1] > lo) {
			float tb = (float)(k - 1) + (x[k - 1] - lo) / (x[k - 1] - x[k]);
			return (tb - ta) / fs / 1.0986123f;
		}
	}
	return -1.0f;
}

/* ------------------------------------------------------------------ resistance */
static const char* cal_resistance(void) {
	const char* e;
	float v_hi;
	if ((e = precheck())) return e;

	g_mode = CTRL_CAL;
	g_theta_cmd = 0;
	g_vd_cmd = g_vq_cmd = 0;
	if ((e = ramp_d_to_current(0.4f * g_params.rated_current, &v_hi))) return e;
	HAL_Delay(500);
	float i_hi = avg_axis(0, 200);

	float v_lo = 0.5f * v_hi;
	g_vd_cmd = v_lo;
	HAL_Delay(300);
	float i_lo = avg_axis(0, 200);
	if ((e = chk())) return e;
	if (i_hi - i_lo < 0.05f) return "current change too small (current sense scale/sign?)";

	/* two points cancel the fixed dead-time / MOSFET drop */
	float r = (v_hi - v_lo) / (i_hi - i_lo);
	float v_off = v_hi - r * i_hi;
	g_params.rd = r;
	g_params.rq = r; /* surface PM: Rq = Rd. The rotor cannot be held for a q-axis DC test */
	g_params.flags |= PF_R_VALID;
	say("R = %.4f ohm   (%.3f V @ %.3f A, %.3f V @ %.3f A, voltage offset %.3f V)\r\n", r, v_hi, i_hi, v_lo, i_lo, v_off);
	return NULL;
}

/* ------------------------------------------------------------------ inductance */
static const char* cal_inductance(int use_q) {
	const char* e;
	float v;
	if ((e = precheck())) return e;
	if (!(g_params.flags & PF_R_VALID)) return "run 'r' first";

	g_mode = CTRL_CAL;
	g_theta_cmd = 0;
	g_vd_cmd = g_vq_cmd = 0;
	if ((e = ramp_d_to_current(0.4f * g_params.rated_current, &v))) return e;
	HAL_Delay(700); /* rotor settles on the d axis */

	if (use_q) {
		say("hold the shaft firmly. starting in 5 s\r\n");
		for (int i = 0; i < 500; i++) {
			HAL_Delay(10);
			if ((e = chk())) return e;
		}
		g_vq_cmd = v; /* same voltage, now on the q axis */
		g_vd_cmd = 0;
		HAL_Delay(700);
	}

	float i0 = avg_axis(use_q, 200);
	if ((e = chk())) return e;
	if (i0 < 0.05f) return "current too low";

	/* drop the voltage to zero and record the decay */
	g_rec_state = 0;
	g_rec_use_q = (uint8_t)use_q;
	g_vd_cmd = 0;
	g_vq_cmd = 0;
	g_rec_state = 1;
	uint32_t t0 = HAL_GetTick();
	while (g_rec_state != 3)
		if (HAL_GetTick() - t0 > 500) return "recording timeout (is currentLoop running?)";

	static float buf[REC_N];
	for (int k = 0; k < REC_N; k++) buf[k] = g_rec[k];
	float tau = decay_tau(buf, REC_N, i0, CTRL_FS_HZ);
	if (tau < 0.0f) return "current did not fall to 25% inside the window (L/R too slow)";
	if (tau * CTRL_FS_HZ < 3.0f) return "L/R shorter than ~3 samples, cannot resolve";

	float L = tau * g_params.rd;
	if (use_q) {
		g_params.lq = L;
	}
	else {
		g_params.ld = L;
		g_params.lq = L; /* surface PM default, overwrite with 'q' if you measure it */
		g_params.flags |= PF_L_VALID;
	}
	say("L%c = %.1f uH   (tau = %.1f us, I0 = %.3f A)\r\n", use_q ? 'q' : 'd', L * 1e6f, tau * 1e6f, i0);
	return NULL;
}

static const char* cal_ld(void) { return cal_inductance(0); }
static const char* cal_lq(void) { return cal_inductance(1); }

/* ------------------------------------------------------------------ encoder: direction, pole pairs, offset */
static const char* cal_encoder(void) {
	const char* e;
	float v;
	const int N_REVS = 20;	/* electrical revolutions to sweep */
	const float F_E = 2.0f; /* electrical Hz */
	if ((e = precheck())) return e;

	g_mode = CTRL_CAL;
	g_theta_cmd = 0;
	g_vd_cmd = g_vq_cmd = 0;
	say("aligning...\r\n");
	if ((e = ramp_d_to_current(0.5f * g_params.rated_current, &v))) return e;
	HAL_Delay(700);

	say("sweeping %d electrical revolutions...\r\n", N_REVS);
	const uint32_t inc = (uint32_t)(F_E / 1000.0f * 4294967296.0f); /* per 1 ms */
	const int steps = (int)(N_REVS * 1000.0f / F_E);
	uint32_t theta = 0, prev = hMT6835.angle;
	int64_t total = 0; /* unwrapped encoder travel, counts */
	for (int s = 0; s < steps; s++) {
		theta += inc;
		g_theta_cmd = theta;
		HAL_Delay(1);
		uint32_t a = hMT6835.angle;
		total += enc_delta(a, prev);
		prev = a;
		if ((e = chk())) return e;
	}
	g_theta_cmd = 0; /* the sweep is an integer number of turns, so this is not a jump */

	float revs = fabsf((float)total) / 2097152.0f; /* mechanical revolutions */
	if (revs < 0.2f) return "rotor did not follow the field (raise current, check load)";
	int8_t dir = (total >= 0) ? 1 : -1;
	float ppf = (float)N_REVS / revs;
	int pp = (int)lroundf(ppf);
	if (pp < 1 || pp > 255 || fabsf(ppf - (float)pp) > 0.2f) {
		say("pole pairs came out %.2f, not an integer: rotor lost steps\r\n", ppf);
		return "sweep unreliable";
	}

	HAL_Delay(1000); /* settle on the d axis at theta = 0 */
	uint32_t mech = enc_avg(200, 2);
	uint32_t m = (mech << 11) * (uint32_t)pp;
	uint32_t off = (dir > 0) ? (0u - m) : m; /* makes elec_angle() = 0 here */

	g_params.pole_pairs = (uint8_t)pp;
	g_params.enc_dir = dir;
	g_params.elec_offset = off;
	g_params.flags |= PF_ELEC_OFFSET_VALID;
	say("encoder dir %+d, pole pairs %d (measured %.2f), offset %lu\r\n", dir, pp, ppf, (unsigned long)off);
	return NULL;
}

/* ------------------------------------------------------------------ current PI autotune */
/* pole-zero cancellation: zero at R/L cancels the winding pole, closed loop becomes first order at wc */
static const char* pi_autotune(void) {
	const uint32_t need = PF_R_VALID | PF_L_VALID;
	if ((g_params.flags & need) != need) return "measure R and L first ('r', 'l')";
	if (!(g_params.cur_BW > 0.0f)) return "set the bandwidth first (s bw 500)";

	const float bw_max = CTRL_FS_HZ / 20.0f;
	if (g_params.cur_BW > bw_max) {
		say("bw %.0f Hz above fs/20, clamped to %.0f Hz\r\n", (double)g_params.cur_BW, (double)bw_max);
		g_params.cur_BW = bw_max;
	}
	const float wc = 2.0f * (float)M_PI * g_params.cur_BW;
	g_params.kp_d = g_params.ld * wc;
	g_params.ki_d = g_params.rd * wc;
	g_params.kp_q = g_params.lq * wc;
	g_params.ki_q = g_params.rq * wc;
	g_params.flags |= PF_PI_VALID;
	pi_apply();
	say("bw %.0f Hz:  d kp %.4f ki %.2f   q kp %.4f ki %.2f  (RAM only, 'w' saves)\r\n", (double)g_params.cur_BW,
		(double)g_params.kp_d, (double)g_params.ki_d, (double)g_params.kp_q, (double)g_params.ki_q);
	return NULL;
}

/* ------------------------------------------------------------------ closed loop current run */
/* 'g <id> <iq>': returns 1 if the loop is now running (caller must not finish()) */
static int run_start(char* args) {
	const char* e;
	const uint32_t need = PF_ELEC_OFFSET_VALID | PF_PI_VALID;
	if ((g_params.flags & need) != need) {
		say("FAILED: run 'e' and 't' first\r\n");
		return 0;
	}
	if ((e = precheck())) {
		say("FAILED: %s\r\n", e);
		return 0;
	}
	char* end;
	float id = strtof(args, &end);
	float iq = strtof(end, NULL);
	float mag = sqrtf(id * id + iq * iq);
	if (mag > g_params.rated_current) { /* scale the vector, keep its direction */
		float sc = g_params.rated_current / mag;
		id *= sc;
		iq *= sc;
		say("limited to rated %.2f A\r\n", (double)g_params.rated_current);
	}

	if (g_mode != CTRL_RUN) { /* fresh start; while running only the references change */
		g_mode = CTRL_OFF;		/* currentLoop() holds the integrators at zero */
		HAL_Delay(1);
		g_id_ref = id;
		g_iq_ref = iq;
		g_mode = CTRL_RUN;
	}
	else {
		g_id_ref = id;
		g_iq_ref = iq;
	}
	say("run id %.3f A  iq %.3f A   ('o' stops)\r\n", (double)id, (double)iq);
	return 1;
}

/* called from cmd_poll() while running: stop on fault, low bus or overcurrent */
static void run_watchdog(void) {
	if (g_mode != CTRL_RUN) return;
	const char* e = NULL;
	if (fabsf(g_id) > g_params.max_current || fabsf(g_iq) > g_params.max_current) e = "overcurrent";
	else if (!(TIM1->BDTR & TIM_BDTR_MOE) || checkAndClearBridgeFault() != 0) e = "bridge fault";
	else if (g_vin < g_params.vbus_min) e = "bus voltage too low";
	if (!e) return;

	g_mode = CTRL_OFF; /* immediately, no ramp: the loop is not in control any more */
	g_id_ref = 0.0f;
	g_iq_ref = 0.0f;
	begin();
	say("STOPPED: %s (id %.2f A, iq %.2f A)\r\n", e, (double)g_id, (double)g_iq);
	g_telem_en = 1;
}

/* ------------------------------------------------------------------ parameters over serial */
static const struct {
	const char* k;
	void* p;
	char t; /* f float, u uint8, i int8 */
} kTab[] = {
	{"kv", &g_params.kv, 'f'},
	{"pp", &g_params.pole_pairs, 'u'},
	{"dir", &g_params.enc_dir, 'i'},
	{"rd", &g_params.rd, 'f'},
	{"rq", &g_params.rq, 'f'},
	{"ld", &g_params.ld, 'f'},
	{"lq", &g_params.lq, 'f'},
	{"rated", &g_params.rated_current, 'f'},
	{"imax", &g_params.max_current, 'f'},
	{"cscale", &g_params.cur_scale, 'f'},
	{"vscale", &g_params.vin_scale, 'f'},
	{"kpd", &g_params.kp_d, 'f'},
	{"kid", &g_params.ki_d, 'f'},
	{"kpq", &g_params.kp_q, 'f'},
	{"kiq", &g_params.ki_q, 'f'},
	{"bw", &g_params.cur_BW, 'f'},
	{"vmin", &g_params.vbus_min, 'f'},
	{"vmax", &g_params.vbus_max, 'f'},
	{"tmax", &g_params.temp_max, 'f'},
};

static void set_param(char* args) {
	char* key = strtok(args, " ");
	char* val = strtok(NULL, " ");
	if (!key || !val) {
		say("usage: s <key> <value>\r\n");
		return;
	}
	float f = strtof(val, NULL);
	for (unsigned i = 0; i < sizeof kTab / sizeof kTab[0]; i++) {
		if (strcmp(key, kTab[i].k) != 0) continue;
		if (kTab[i].t == 'f')
			*(float*)kTab[i].p = f;
		else if (kTab[i].t == 'u')
			*(uint8_t*)kTab[i].p = (uint8_t)f;
		else
			*(int8_t*)kTab[i].p = (int8_t)f;
		say("%s = %f (RAM only, 'w' saves)\r\n", key, f);
		pi_apply();
		return;
	}
	say("unknown key '%s'\r\n", key);
}

static void print_params(void) {
	const motor_params_t* p = &g_params;
	say("pp %u  dir %+d  offset %lu  kv %.2f\r\n", p->pole_pairs, p->enc_dir, (unsigned long)p->elec_offset, (double)p->kv);
	say("Rd %.4f  Rq %.4f ohm   Ld %.1f  Lq %.1f uH\r\n", (double)p->rd, (double)p->rq, (double)(p->ld * 1e6f), (double)(p->lq * 1e6f));
	say("rated %.2f A  imax %.2f A  cscale %.6f  vscale %.6f\r\n", (double)p->rated_current, (double)p->max_current, (double)p->cur_scale, (double)p->vin_scale);
	say("kp/ki d %.4f %.2f  q %.4f %.2f  bw %.0f Hz\r\n", (double)p->kp_d, (double)p->ki_d, (double)p->kp_q, (double)p->ki_q, (double)p->cur_BW);
	say("vbus %.1f..%.1f V  tmax %.0f C  flags 0x%02lX  seq %lu\r\n", (double)p->vbus_min, (double)p->vbus_max, (double)p->temp_max, (unsigned long)p->flags, (unsigned long)p->seq);
}

static void save_params(void) {
	disableBridge(); /* the CPU stalls ~20 ms during the flash erase */
	HAL_StatusTypeDef st = params_save();
	say("save %s\r\n", st == HAL_OK ? "OK" : "FAILED");
	initBridge(); /* use whatever your normal power-up sequence is */
}

void reset_encoder(void) {
	g_mode = CTRL_OFF;
	MT6835_StopBurst(&hMT6835);
	HAL_Delay(10);
	say("reseting mt6835 zero registers\r\n");
	MT6835_WriteRegister(&hMT6835, MT6835_ADDR_ROT, 0x07);
	MT6835_ResetZero(&hMT6835);
	MT6835_SaveEEPROM(&hMT6835);
	say("saving to EEPROM wait 6s...\r\n");
	HAL_Delay(6000);
	say("Done\r\n");

	MT6835_StartBurst(&hMT6835);
}

/* ------------------------------------------------------------------ dispatcher (line based, LF terminated) */
static void run(const char* name, const char* (*fn)(void)) {
	say("%s\r\n", name);
	const char* err = fn();
	say(err ? "FAILED: %s\r\n" : "done\r\n", err);
}

static void dispatch(char* line) {
	begin();
	switch (line[0]) {
		case 'e':
			run("encoder", cal_encoder);
			break;
		case 'r':
			run("resistance", cal_resistance);
			break;
		case 'l':
			run("inductance d", cal_ld);
			break;
		case 'q':
			run("inductance q", cal_lq);
			break;
		case 'p':
			print_params();
			break;
		case 's':
			set_param(line + 1);
			break;
		case 'w':
			save_params();
			break;
		case 't':
			run("autotune", pi_autotune);
			break;
		case 'g':
			if (run_start(line + 1)) {
				g_telem_en = 1; /* keep running: no finish() */
				return;
			}
			break;
		case 'o':
			say("off\r\n");
			break; /* finish() ramps down and switches off */
		case 'z':
			reset_encoder();
			break;
		case '?':
			say("e encoder  r resistance  l Ld  q Lq (hold shaft)  t PI autotune  g <id> <iq> run  o off\r\n"
				"p print  s <key> <val>  w save  z reset encoder zero  x abort\r\n");
			break;
		default:
			break; /* unknown / stray text: ignored */
	}
	finish();
}

void cmd_poll(void) {
	static char line[48];
	static int n;
	uint8_t c;
	run_watchdog();
	while (rx_pop(&c)) {
		if (c == '\r') continue;
		if (c != '\n') {
			if (n < (int)sizeof line - 1) line[n++] = (char)c;
			continue;
		}
		line[n] = 0;
		n = 0;
		if (line[0]) dispatch(line);
	}
}
