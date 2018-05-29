#include "PIDcontrol.h"
#include "var.h"
#include "gun.h"
#include "led.h"
#include "delay.h"
#include "laser.h"
#include "timer.h"
#include "mpu6050_driver.h"
#include "mpu6050_interrupt.h"
#include "can1.h"
#include "can2.h"
#include "usart1.h"
#include "usart3.h"
#include "canbus.h"
#include "RControl.h"
#include <stdio.h>

short int mpucount =0; 
int runcount =0;int i=0;
int main(void)
{   STATE.usartRC=DISABLE;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
    Led_Configuration();            //红绿灯配置
    USART3_Configuration(9600);   //命令串口配置
    TIM2_Configuration();	        //闪灯中断配置
    USART1_Configuration(100000);   //遥控串口配置
	PWM_Configuration();            //摩擦轮配置
//    PWM_BridgeInit();               //电调初始化
    Laser_Configuration();          //激光配置
    while(MPU6050_Init()){
        mpucount++;
        printf("MPUInit Error %d Time",mpucount);
        if (mpucount>50) {printf("Please check MPU6050");mpucount=0;break;}
    }   //MPU6050初始化，自带出错死循环，初始化不成功不会跳出 
    MPU6050_Gyro_calibration(MPU6050_DEVICE_ADDRESS);
    while( MPU6050_EnableInt()){
        mpucount++;
        //printf("MPUInit Error %d Time",mpucount);
        if (mpucount>50) {printf("Please check MPU6050");break;}
    }   //MPU6050初始化，自带死循环，初始化不成功不会跳出 
    CAN1_Configuration();           //can1配置
	CAN2_Configuration();           //can2配置
    TIM6_Configuration();           //控制中断配置
	TIM6_Start();                   //控制中断开启
    STATE.usartRC=1;
    RED_LED_OFF();
    GREEN_LED_ON();
    while(1){
//       for(i=0;i>18;i++)printf("T=%d ",USART1_RX_BUF[i]);
//
//        if (runcount>10000){
//            printf("X=%f    ",MPU6050_Real_Data.Gyro_X);
//            printf("Y=%f    ",MPU6050_Real_Data.Accel_Y);
//            printf("Z=%f    \r\n",MPU6050_Real_Data.Gyro_Z);
//           runcount=0;
//        }
//        else runcount++; 
    }   
}
