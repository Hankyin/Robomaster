#ifndef _PIDCONTROL_H_
#define _PIDCONTROL_H_

#include <stm32f4xx.h>
typedef struct{
		float kp;		//比例p
		float ki;		//积分i
		float kd;		//微分d
    float ref;  //目标值
    float fdb;  //反馈值
    float out;  //输出值
    float ERR;				//偏差值	
    float ERR_Inte;		//偏差积分
    float ERR_last;   //前一次偏差
}PID_Regulator;	//PID规则

typedef struct SPEED_Multi	
{ 
	float ka;			//前进：Advence
	float kt;			//平移：Transport
	float kr;   	//自旋：Rotate
}SPEED_Multi;

typedef struct M_Para		//极值参数
{ 
	float MAX;		//最大值
	float MID;		//中间值
	float MIN;   	//最小值
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


