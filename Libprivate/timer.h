#ifndef __TIMER_H__
#define __TIMER_H__
#include "stm32f4xx.h"

void TIM6_Configuration(void);
void TIM6_Start(void);
void TIM6_Stop(void);
void TIM2_Configuration(void);

//void TIM2_5_Configuration(void);
void TIM8_Configuration(void);
void TIM4_Configuration(void);
uint32_t Get_Time_Micros(void);

#endif
