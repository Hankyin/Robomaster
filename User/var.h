/* ȫ�ֱ������� */
#ifndef VAR_H
#define VAR_H
#include <stm32f4xx.h>

extern int AdvanceSpeed;      //ǰ���ٶ�
extern int TranslationSpeed;  // ƽ���ٶ�
extern int RotateSpeed;       // ��ת�ٶ�

typedef struct TARGET_ANGLE{ 
	float Pitch;
	float Yaw;
	float Roll;   
}TARGET_ANGLE;
extern volatile TARGET_ANGLE TARGET_Angle;  //Ŀ��Ƕ�
extern volatile TARGET_ANGLE TARGET_Speed;

#define START_STATE    0     //�ϵ���ʼ��״̬ 5s������
#define NORMAL_STATE  1		//������״̬
#define PAUSE_STATE   2		//��ֹ̨ͣ��ת״̬
#define CM_STATE      3  	//���̵���״̬
#define STOP_STATE   4		//У׼״̬
#define CHECK_STATE   5		//����һ��
#define ERROR_STATE   6		//����״̬

#define OK   1      //Opened��ֵ
#define Err  0      //Opened��ֵ
typedef struct STATE_SYS{ 
	u8 Run;     //�൱��all
	u8 Mpu;     
	u8 usart3;
    u8 usartRC;
    u8 GM;
    u8 CM;
    u8 Check;
}STATE_SYS;
extern volatile STATE_SYS STATE;            //����״̬
extern volatile STATE_SYS isopened;           //isopened?
extern volatile STATE_SYS isprint;  

#define YAWhave 1

#endif

