#ifndef CORDICMGR_H
#define CORDICMGR_H

#include "stm32g4xx_ll_cordic.h"
#define Q31_TO_F (1.0f / 2147483648.0f)

static inline void cordic_init(void) {
	LL_CORDIC_Config(CORDIC,
					 LL_CORDIC_FUNCTION_COSINE,	   // gives cos then sin
					 LL_CORDIC_PRECISION_6CYCLES,  // 24 iterations; 4CYCLES is faster, still fine for FOC
					 LL_CORDIC_SCALE_0,
					 LL_CORDIC_NBWRITE_1,  // one argument (angle); modulus defaults to 1.0
					 LL_CORDIC_NBREAD_2,   // two results: cos, sin
					 LL_CORDIC_INSIZE_32BITS,
					 LL_CORDIC_OUTSIZE_32BITS);
}

static inline void sincos_q31(uint32_t phase, float* s, float* c) {
	LL_CORDIC_WriteData(CORDIC, phase);				   // starts the calculation
	int32_t cs = (int32_t)LL_CORDIC_ReadData(CORDIC);  // first read: cos (stalls until ready)
	int32_t sn = (int32_t)LL_CORDIC_ReadData(CORDIC);  // second read: sin
	*c = (float)cs * Q31_TO_F;
	*s = (float)sn * Q31_TO_F;
}

#endif