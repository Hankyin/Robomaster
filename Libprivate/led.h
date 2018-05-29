#ifndef __LED_H__
#define __LED_H__

#include "stm32f4xx.h"

#define GREEN_LED_ON()          GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define GREEN_LED_OFF()         GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define GREEN_LED_TOGGLE()      GPIO_ToggleBits(GPIOC, GPIO_Pin_1)

#define RED_LED_ON()            GPIO_ResetBits(GPIOC, GPIO_Pin_2)
#define RED_LED_OFF()           GPIO_SetBits(GPIOC, GPIO_Pin_2)
#define RED_LED_TOGGLE()        GPIO_ToggleBits(GPIOC, GPIO_Pin_2)

void Led_Configuration(void);

#endif	
