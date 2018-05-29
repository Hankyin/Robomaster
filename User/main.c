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
    Led_Configuration();            //���̵�����
    USART3_Configuration(9600);   //���������
    TIM2_Configuration();	        //�����ж�����
    USART1_Configuration(100000);   //ң�ش�������
	PWM_Configuration();            //Ħ��������
//    PWM_BridgeInit();               //�����ʼ��
    Laser_Configuration();          //��������
    while(MPU6050_Init()){
        mpucount++;
        printf("MPUInit Error %d Time",mpucount);
        if (mpucount>50) {printf("Please check MPU6050");mpucount=0;break;}
    }   //MPU6050��ʼ�����Դ�������ѭ������ʼ�����ɹ��������� 
    MPU6050_Gyro_calibration(MPU6050_DEVICE_ADDRESS);
    while( MPU6050_EnableInt()){
        mpucount++;
        //printf("MPUInit Error %d Time",mpucount);
        if (mpucount>50) {printf("Please check MPU6050");break;}
    }   //MPU6050��ʼ�����Դ���ѭ������ʼ�����ɹ��������� 
    CAN1_Configuration();           //can1����
	CAN2_Configuration();           //can2����
    TIM6_Configuration();           //�����ж�����
	TIM6_Start();                   //�����жϿ���
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
