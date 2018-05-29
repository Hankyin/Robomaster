#include "PIDcontrol.h"
#include "canbus.h"
#include "var.h"
#include "mpu6050_driver.h"
#include "Parasetting.h"
#include <stdio.h>
/*********************************************************************/
//云台电机PID：Gimbal Motor	Pitch/Yaw Position/Speed PID
                                        // P  I   D  r  f  o  e  i  l
volatile PID_Regulator GMPPositionPID = { 20, 0, -4, 0, 0, 0, 0, 0, 0};     
volatile PID_Regulator GMPSpeedPID =    { 50, 0,  0, 0, 0, 0, 0, 0, 0};
volatile PID_Regulator GMYPositionPID = {  4, 0,  0, 0, 0, 0, 0, 0, 0};			
volatile PID_Regulator GMYSpeedPID =    { 35, 0, 50, 0, 0, 0, 0, 0, 0};
//底盘跟随PID：Chassis Motor Rotate Speed PID
volatile PID_Regulator CMRotatePID =    {-12, 0,  1, 0, 0, 0, 0, 0, 0};
//底盘速度PID：Chassis Motor 1/2/3/4 Speed PID
                                    //  P     I   D  r  f  o  e  i  l
volatile PID_Regulator CM1SpeedPID = {-90, -0.4, 15, 0, 0, 0, 0, 0, 0};
volatile PID_Regulator CM2SpeedPID = {-90, -0.4, 15, 0, 0, 0, 0, 0, 0};
volatile PID_Regulator CM3SpeedPID = {-90, -0.4, 15, 0, 0, 0, 0, 0, 0};
volatile PID_Regulator CM4SpeedPID = {-90, -0.4, 15, 0, 0, 0, 0, 0, 0};
/*********************************************************************/
//底盘电机倍率因子	           ka    kt   kr
volatile SPEED_Multi CM_K = {0.25f,0.25f,0.8f};
//Pitch	参数区间        上    中		下
volatile M_Para PITCH={ 76.5,  68.0,  55.0};
//Yaw	参数区间	    左    中		右
volatile M_Para YAW=  {353.0, 299.5, 242.0};
/*********************************************************************/
//PID公式计算函数
float PIDCalc(volatile PID_Regulator *PID)
{    
    PID->ERR = PID->ref - PID->fdb;				//ref:目标值，fdb：反馈值
    PID->ERR_Inte += PID->ERR;						//Inte：积分，ERR：差值
    PID->out = -(PID->kp*PID->ERR + PID->ki*PID->ERR_Inte + PID->kd*(PID->ERR-PID->ERR_last));
    PID->ERR_last = PID->ERR;
    return PID->out;
}
//PID初始化函数
void PID_Init(){
	//底盘差值/积分/微分 清零
    GMPPositionPID.ERR_last = GMPPositionPID.ERR_Inte = GMPPositionPID.ERR = 0;
    GMPSpeedPID.ERR_last = GMPSpeedPID.ERR_Inte = GMPSpeedPID.ERR = 0;
    GMYPositionPID.ERR_last = GMYPositionPID.ERR_Inte = GMYPositionPID.ERR = 0;	
    GMYSpeedPID.ERR_last = GMYSpeedPID.ERR_Inte = GMYSpeedPID.ERR = 0;
	//底盘跟随差值/积分/微分 清零
    CMRotatePID.ERR_last = CMRotatePID.ERR_Inte = CMRotatePID.ERR = 0;  
	//底盘差值/积分/微分 清零
    CM1SpeedPID.ERR_last = CM1SpeedPID.ERR_Inte = CM1SpeedPID.ERR = 0;
    CM2SpeedPID.ERR_last = CM2SpeedPID.ERR_Inte = CM2SpeedPID.ERR = 0;
    CM3SpeedPID.ERR_last = CM3SpeedPID.ERR_Inte = CM3SpeedPID.ERR = 0;
    CM4SpeedPID.ERR_last = CM4SpeedPID.ERR_Inte = CM4SpeedPID.ERR = 0;
}
/*********************************************************************/
//过程变量定义
float CMSpeed[4]={0,0,0,0}; //底盘速度：Chassis Motor Speed
int cm=5;int count = 0;
int run_count = 0;
void PIDControl(void)
{    
/*********************************************************************/    
    switch (STATE.Run){     
/*********************************************************************/        
        case NORMAL_STATE:{            
            if(run_count<500){
								//前500次运算，此时刚开机 做启动动作。
                TARGET_Angle.Yaw =YAW.MID;      //PID启动归中 Mohs
                TARGET_Angle.Pitch=PITCH.MID;
                Set_Gimbal_Current(CAN2,GMYawControl(),GMPitchControl());
                Set_CM_Speed(CAN2,0,0,0,0);
                run_count++;
            }                
            else {
            Set_Gimbal_Current(CAN2,GMYawSpeedControl(),GMPitchControl());
            CMControl();
            }
        }break;
/*********************************************************************/        
        case PAUSE_STATE:{      //暂停模式
            TARGET_Angle.Yaw =YAW.MID;
            TARGET_Angle.Pitch=PITCH.MID;
            Set_Gimbal_Current(CAN2,GMYawControl(),GMPitchControl());
            Set_CM_Speed(CAN2,0,0,0,0);
        }break;
/*********************************************************************/
        case STOP_STATE:{  
            if(isprint.GM){
                if(count%40==0){
                    printf("pitchPOSIfdb=%f\t",GMPitchEncoder.ecd_angle);
                    printf("yawPOSIfdb=%f\t\r\n",GMYawEncoder.ecd_angle);
                }
                if(count>1000)isprint.GM=0,count=0;
                else count++;
            }
            run_count=0;
            PID_Init();
            Set_Gimbal_Current(CAN2,0,0);
            Set_CM_Speed(CAN2,0,0,0,0);      
        }break;
/*********************************************************************/                
       case CM_STATE:{
            Set_Gimbal_Current(CAN2, 0, 0);
            CMTestControl(CMSpeed[cm-1]);
            if(isprint.CM){
                if(count%40==0){
                    switch(cm){
                        case 1:printf("CM1\tRPM=%f\tPIDOUT=%f\r\n",CM1Encoder.filter_rate,CM1SpeedPID.out);break;
                        case 2:printf("CM2\tRPM=%f\tPIDOUT=%f\r\n",CM2Encoder.filter_rate,CM2SpeedPID.out);break;
                        case 3:printf("CM3\tRPM=%f\tPIDOUT=%f\r\n",CM3Encoder.filter_rate,CM3SpeedPID.out);break;
                        case 4:printf("CM4\tRPM=%f\tPIDOUT=%f\r\n",CM4Encoder.filter_rate,CM4SpeedPID.out);break;
                        default:;            
                    }
                }
                if(count>1000)isprint.CM=0,count=0;
                else count++;
            }            
        }break;
/*********************************************************************/         
    }
//    if (STATE.Check) printf("PID_Control(%d)\r\n",STATE.Run);
}
//底盘控制任务
float CM_Current[4]={0,0,0,0};
void CMControl(void)
{    
	//底盘旋转量计算
	if(STATE.Run==STOP_STATE) //启动阶段，底盘不动
	{
        AdvanceSpeed = 0;
		TranslationSpeed = 0;
		RotateSpeed = 0;	 
	}
	else //底盘跟随编码器旋转PID计算	 
    {    
        CMRotatePID.ref = YAW.MID;	 
        CMRotatePID.fdb = GMYawEncoder.ecd_angle;		 
        RotateSpeed = PIDCalc(&CMRotatePID);
      
	}	
	CM1SpeedPID.ref = -AdvanceSpeed*CM_K.ka + TranslationSpeed*CM_K.kt + RotateSpeed*CM_K.kr;
	CM2SpeedPID.ref =  AdvanceSpeed*CM_K.ka + TranslationSpeed*CM_K.kt + RotateSpeed*CM_K.kr;
	CM3SpeedPID.ref =  AdvanceSpeed*CM_K.ka - TranslationSpeed*CM_K.kt + RotateSpeed*CM_K.kr;
	CM4SpeedPID.ref = -AdvanceSpeed*CM_K.ka - TranslationSpeed*CM_K.kt + RotateSpeed*CM_K.kr;
    
	CM1SpeedPID.fdb = CM1Encoder.filter_rate;
	CM2SpeedPID.fdb = CM2Encoder.filter_rate;
	CM3SpeedPID.fdb = CM3Encoder.filter_rate;
	CM4SpeedPID.fdb = CM4Encoder.filter_rate;
    
	CM_Current[0]=PIDCalc(&CM1SpeedPID);
	CM_Current[1]=PIDCalc(&CM2SpeedPID);
	CM_Current[2]=PIDCalc(&CM3SpeedPID);
	CM_Current[3]=PIDCalc(&CM4SpeedPID);  
    
	Set_CM_Speed(CAN2, CM_Current[0],CM_Current[1],CM_Current[2],CM_Current[3]);     
}
void CMTestControl(float speed)
{
    switch(cm){
        case 1:{
            CM1SpeedPID.ref = speed;
            CM1SpeedPID.fdb = CM1Encoder.filter_rate;
            CM_Current[0]=PIDCalc(&CM1SpeedPID);
            Set_CM_Speed(CAN2, CM_Current[0],0,0,0);
        }break;
        case 2:{
            CM2SpeedPID.ref = speed;
            CM2SpeedPID.fdb = CM2Encoder.filter_rate;
            CM_Current[1]=PIDCalc(&CM2SpeedPID);
            Set_CM_Speed(CAN2,0,CM_Current[1],0,0);
        }break;
        case 3:{
            CM3SpeedPID.ref = speed;
            CM3SpeedPID.fdb = CM3Encoder.filter_rate;
            CM_Current[2]=PIDCalc(&CM3SpeedPID);
            Set_CM_Speed(CAN2,0,0,CM_Current[2],0);
        }break;
        case 4:{
            CM4SpeedPID.ref = speed;
            CM4SpeedPID.fdb = CM4Encoder.filter_rate;
            CM_Current[3]=PIDCalc(&CM4SpeedPID);
            Set_CM_Speed(CAN2, 0,0,0 ,CM_Current[3]);
        }break;
        
    } 
}
//云台yaw轴控制程序
float GMYawControl(void)
{
    if(TARGET_Angle.Yaw>=YAW.MAX){TARGET_Angle.Yaw=YAW.MAX; TARGET_Speed.Yaw=0;}
    else if(TARGET_Angle.Yaw<=YAW.MIN){TARGET_Angle.Yaw=YAW.MIN; TARGET_Speed.Yaw=0;}
    else;
	GMYPositionPID.ref = TARGET_Angle.Yaw;
	GMYPositionPID.fdb = GMYawEncoder.ecd_angle;
	//yaw speed control
	GMYSpeedPID.ref = PIDCalc(&GMYPositionPID);
	GMYSpeedPID.fdb = MPU6050_Real_Data.Gyro_Z;
    //if(STATE.Mpu==0)GMYSpeedPID.fdb = GMYawEncoder.filter_rate;
	return PIDCalc(&GMYSpeedPID);			
}
float GMYawSpeedControl(void)
{
	//yaw speed control
	GMYSpeedPID.ref = TARGET_Speed.Yaw;
	GMYSpeedPID.fdb = MPU6050_Real_Data.Gyro_Z;
    //if(STATE.Mpu==0)GMYSpeedPID.fdb = GMYawEncoder.filter_rate;
	return PIDCalc(&GMYSpeedPID);			
}
//云台pitch轴控制程序
float GMPitchControl(void)
{
    if(TARGET_Angle.Pitch>PITCH.MAX)  {TARGET_Angle.Pitch=PITCH.MAX;}
    else if(TARGET_Angle.Pitch<PITCH.MIN) {TARGET_Angle.Pitch=PITCH.MIN;}
    else;
	GMPPositionPID.ref = TARGET_Angle.Pitch;
	GMPPositionPID.fdb = GMPitchEncoder.ecd_angle;
	//pitch speed control
	GMPSpeedPID.ref = PIDCalc(&GMPPositionPID); 
	GMPSpeedPID.fdb = MPU6050_Real_Data.Gyro_Y;
    //if(STATE.Mpu==0)GMPSpeedPID.fdb = GMPitchEncoder.filter_rate;
	return PIDCalc(&GMPSpeedPID);
}



