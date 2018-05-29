#ifndef __Encoder_H__
#define __Encoder_H__
#include "stm32f4xx.h"

void Quad_Encoder_Configuration(void);
void Encoder_Start(void);
int32_t GetQuadEncoderDiff(void);
#endif
