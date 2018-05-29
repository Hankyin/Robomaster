#ifndef __CAN1_H__
#define __CAN1_H__
#include "stm32f4xx.h"
void CAN1_Configuration(void);
void Set_Shoot_Speed(CAN_TypeDef* CANx,unsigned short int speed);
#endif 
