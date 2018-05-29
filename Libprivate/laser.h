#ifndef _LASER_H_
#define _LASER_H_
#include "stm32f4xx.h"

#define LASER_ON()  GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define LASER_OFF()  GPIO_ResetBits(GPIOA, GPIO_Pin_8)

void Laser_Configuration(void);
#endif

