#ifndef CORDICMGR_H
#define CORDICMGR_H

#include <math.h>

#include "stm32g4xx_ll_cordic.h"
#define Q31_TO_F (1.0f / 2147483648.0f)

/* the unit normally sits in sin/cos mode; atan2_q32() switches to phase mode and back. The modulus is
 * written on every sin/cos call: with one write per call the unit reuses the last second argument, which
 * after a phase calculation is atan2's y, and the outputs come out scaled by it */
#define CORDIC_CSR_COS (LL_CORDIC_FUNCTION_COSINE | LL_CORDIC_PRECISION_6CYCLES | LL_CORDIC_SCALE_0 | LL_CORDIC_NBWRITE_2 | LL_CORDIC_NBREAD_2 | LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)
#define CORDIC_CSR_PHASE (LL_CORDIC_FUNCTION_PHASE | LL_CORDIC_PRECISION_6CYCLES | LL_CORDIC_SCALE_0 | LL_CORDIC_NBWRITE_2 | LL_CORDIC_NBREAD_1 | LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)

static inline void cordic_init(void) {
	LL_CORDIC_Config(CORDIC,
					 LL_CORDIC_FUNCTION_COSINE,	   // gives cos then sin
					 LL_CORDIC_PRECISION_6CYCLES,  // 24 iterations; 4CYCLES is faster, still fine for FOC
					 LL_CORDIC_SCALE_0,
					 LL_CORDIC_NBWRITE_2,  // two arguments: angle, modulus
					 LL_CORDIC_NBREAD_2,   // two results: cos, sin
					 LL_CORDIC_INSIZE_32BITS,
					 LL_CORDIC_OUTSIZE_32BITS);
}

static inline void sincos_q31(uint32_t phase, float* s, float* c) {
	LL_CORDIC_WriteData(CORDIC, phase);
	LL_CORDIC_WriteData(CORDIC, 0x7FFFFFFFu);		   // modulus 1.0; the second write starts the calculation
	int32_t cs = (int32_t)LL_CORDIC_ReadData(CORDIC);  // first read: cos (stalls until ready)
	int32_t sn = (int32_t)LL_CORDIC_ReadData(CORDIC);  // second read: sin
	*c = (float)cs * Q31_TO_F;
	*s = (float)sn * Q31_TO_F;
}

/* atan2(y, x) as a Q32 angle (2^32 = one turn). Only call from the ISR: it reprograms the unit */
static inline uint32_t atan2_q32(float y, float x) {
	float m = fabsf(x) + fabsf(y);
	if (!(m > 0.0f)) return 0;
	float k = 2.0e9f / m; /* both inputs into the Q31 range */
	WRITE_REG(CORDIC->CSR, CORDIC_CSR_PHASE);
	LL_CORDIC_WriteData(CORDIC, (uint32_t)(int32_t)(x * k));
	LL_CORDIC_WriteData(CORDIC, (uint32_t)(int32_t)(y * k)); /* second write starts it */
	int32_t ph = (int32_t)LL_CORDIC_ReadData(CORDIC);		 /* Q31 of pi = Q32 of a turn */
	WRITE_REG(CORDIC->CSR, CORDIC_CSR_COS);
	return (uint32_t)ph;
}

#endif
