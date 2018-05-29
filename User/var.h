/* 全局变量声明 */
#ifndef VAR_H
#define VAR_H
#include <stm32f4xx.h>

extern int AdvanceSpeed;      //前进速度
extern int TranslationSpeed;  // 平移速度
extern int RotateSpeed;       // 旋转速度

typedef struct TARGET_ANGLE{ 
	float Pitch;
	float Yaw;
	float Roll;   
}TARGET_ANGLE;
extern volatile TARGET_ANGLE TARGET_Angle;  //目标角度
extern volatile TARGET_ANGLE TARGET_Speed;

#define START_STATE    0     //上电后初始化状态 5s钟左右
#define NORMAL_STATE  1		//无输入状态
#define PAUSE_STATE   2		//云台停止不转状态
#define CM_STATE      3  	//底盘调试状态
#define STOP_STATE   4		//校准状态
#define CHECK_STATE   5		//测试一次
#define ERROR_STATE   6		//错误状态

#define OK   1      //Opened真值
#define Err  0      //Opened假值
typedef struct STATE_SYS{ 
	u8 Run;     //相当于all
	u8 Mpu;     
	u8 usart3;
    u8 usartRC;
    u8 GM;
    u8 CM;
    u8 Check;
}STATE_SYS;
extern volatile STATE_SYS STATE;            //运行状态
extern volatile STATE_SYS isopened;           //isopened?
extern volatile STATE_SYS isprint;  

#define YAWhave 1

#endif

