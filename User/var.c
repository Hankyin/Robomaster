#include "var.h"
#include "PIDcontrol.h"
int AdvanceSpeed=0;      //ǰ���ٶ�
int TranslationSpeed=0;  // ƽ���ٶ�
int RotateSpeed=0;       // ��ת�ٶ�
 

volatile TARGET_ANGLE TARGET_Angle;
volatile TARGET_ANGLE TARGET_Speed;
                           //run      mpu usart3 usartrc     GM cm CHECK
volatile STATE_SYS STATE = {NORMAL_STATE, 0,     0,      0,    0,  0, 0};
volatile STATE_SYS isopened ={0,        0,     0,      0,    0,  0, 0};
volatile STATE_SYS isprint  ={0,        0,     0,      0,    0,  0, 0};








