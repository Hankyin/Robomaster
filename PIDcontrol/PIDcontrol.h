#ifndef _PIDCONTROL_H_
#define _PIDCONTROL_H_

#include <stm32f4xx.h>
typedef struct{
		float kp;		//����p
		float ki;		//����i
		float kd;		//΢��d
    float ref;  //Ŀ��ֵ
    float fdb;  //����ֵ
    float out;  //���ֵ
    float ERR;				//ƫ��ֵ	
    float ERR_Inte;		//ƫ�����
    float ERR_last;   //ǰһ��ƫ��
}PID_Regulator;	//PID����

typedef struct SPEED_Multi	
{ 
	float ka;			//ǰ����Advence
	float kt;			//ƽ�ƣ�Transport
	float kr;   	//������Rotate
}SPEED_Multi;

typedef struct M_Para		//��ֵ����
{ 
	float MAX;		//���ֵ
	float MID;		//�м�ֵ
	float MIN;   	//��Сֵ
}M_Para;

extern volatile SPEED_Multi CM_Speed;	
extern volatile M_Para PITCH;
extern volatile M_Para YAW; 

extern volatile PID_Regulator GMPPositionPID ;     
extern volatile PID_Regulator GMPSpeedPID ;
extern volatile PID_Regulator GMYPositionPID ;			
extern volatile PID_Regulator GMYSpeedPID ;

extern volatile PID_Regulator CMRotatePID ;
extern volatile PID_Regulator CM1SpeedPID ;
extern volatile PID_Regulator CM2SpeedPID ;
extern volatile PID_Regulator CM3SpeedPID ;
extern volatile PID_Regulator CM4SpeedPID ;


extern float CMSpeed[4];
extern int cm;

float PIDCalc(volatile PID_Regulator *PID);
void PIDInit(void);
void PIDControl(void);

void CMControl(void);
void CMTestControl(float speed);
float GMPitchControl(void);
float GMYawControl(void); 
float GMYawSpeedControl(void);

#endif


