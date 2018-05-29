#include "var.h"
#include "gun.h"
#include "can1.h"
#include "delay.h"
#include "canbus.h"
#include "RControl.h"
#include "stdio.h"
    /* ң�������ݲ��� */
RC_Ctl_t RC;
static int StartCheckCount=0;	 //����У�����������ʱ����ֵУ�飬���ҡ��û�й�λ������Ϊ�������󣬵ȴ�����ֱ����λ
void RC_DataProcess(unsigned char* sbus_rx_buffer){
    /* �ж��Ƿ�����ң�������� */
    if(STATE.usartRC==DISABLE)return;
    /* ���յ�ң�����źű�� */
    /* ȡ�ÿ������� */    
    if(sbus_rx_buffer == NULL) return;    
    RC.rc.ch0 = ((int16_t)sbus_rx_buffer[0] | ((int16_t)sbus_rx_buffer[1] << 8)) & 0x07FF; 
    RC.rc.ch1 = (((int16_t)sbus_rx_buffer[1] >> 3) | ((int16_t)sbus_rx_buffer[2] << 5)) & 0x07FF;
    RC.rc.ch2 = (((int16_t)sbus_rx_buffer[2] >> 6) | ((int16_t)sbus_rx_buffer[3] << 2) |
                 ((int16_t)sbus_rx_buffer[4] << 10)) & 0x07FF;
    RC.rc.ch3 = (((int16_t)sbus_rx_buffer[4] >> 1) | ((int16_t)sbus_rx_buffer[5]<<7)) & 0x07FF;    
    RC.rc.s1 = ((sbus_rx_buffer[5] >> 4) & 0x000C) >> 2;
    RC.rc.s2 = ((sbus_rx_buffer[5] >> 4) & 0x0003);
    RC.mouse.x = ((int16_t)sbus_rx_buffer[6]) | ((int16_t)sbus_rx_buffer[7] << 8);
    RC.mouse.y = ((int16_t)sbus_rx_buffer[8]) | ((int16_t)sbus_rx_buffer[9] << 8);
    RC.mouse.z = ((int16_t)sbus_rx_buffer[10]) | ((int16_t)sbus_rx_buffer[11] << 8);    
    RC.mouse.press_l = sbus_rx_buffer[12];
    RC.mouse.press_r = sbus_rx_buffer[13]; 
    RC.key.v = ((int16_t)sbus_rx_buffer[14])| ((int16_t)sbus_rx_buffer[15] << 8); 
    
    /* ����ʱ�������ݼ�飬��ֹDMA��λ��������ң������ɵ���������� */

//    if(StartCheckCount<5){//5��У��ͨ������Ϊң��������
//        if(RC.rc.ch0!=1024 ||RC.rc.ch1!=1024 ||RC.rc.ch2!=1024 || RC.rc.ch3!=1024 || RC.rc.s2!=3 || RC.rc.s1!=3){
//            StartCheckCount=0;
//            while(DMA1_Stream5->NDTR>17);//�����λ���⣬��֤ÿ������ң�����ͳ�����ƥ������λ
//            DMA1_Stream5->NDTR=18;
//        }
//        else{
//            StartCheckCount++;
//        }
//        return;//У��ɹ�֮ǰ�����к�������
//    }
    RC_Control(RC);
}
    /* ң�������ܲ��� */
float Pitch_Speed=0;	 	 //Pitch�ٶ�
float Yaw_Speed=0;		 	 //Yaw�ٶ�
static unsigned char PWMGun=0;	 //Ħ���ֿ���
unsigned char Shoot=0;	 	 //�͵�1����
int mouse_count=0;
void RC_Control(RC_Ctl_t RC) 
{
    TranslationSpeed = (RC.rc.ch0-1024);	 	 //ƽ��
    AdvanceSpeed = (RC.rc.ch1-1024);		 	 //ǰ��
    Yaw_Speed = -(RC.rc.ch2 - 1024) * 0.005f;	 	 	 //��̨�ٶ�
    Pitch_Speed = -(RC.rc.ch3 - 1024) * 0.0025f;
    
    /*����1*/
    if(RC.rc.last_s1!=1 && RC.rc.s1==1){  //�л��˿���1��1����״̬
        PWMGun=1-PWMGun;				   //����Ħ����
    }
    RC.rc.last_s1=RC.rc.s1;  
    
    
    if( RC.rc.s1==2 ){Shoot=1;} 
    else if(PWMGun==1 && RC.mouse.press_l==1){Shoot=1;}
    else {Shoot=0;}        	  //����Ħ���ֹر�ʱ�ͻ�ر��͵�} 
    
    /* ����2 */
    if(RC.rc.s2==1){//�л��˿���1��1����״̬
        STATE.Run=NORMAL_STATE;		 
    }
    if(RC.rc.s2==3){//�л��˿���1��1����״̬
        STATE.Run=PAUSE_STATE;		 
    }
    if(RC.rc.s2==2){//�л��˿���1��1����״̬
        STATE.Run=STOP_STATE;		 
    } 
    RC.rc.last_s2=RC.rc.s2;
    /* ������깦�ܲ��� */
    if(RC.mouse.x>2||RC.mouse.x<-2) {Yaw_Speed=-(RC.mouse.x/80.0); }	 //���������ҡ��
    if(RC.mouse.y>2||RC.mouse.y<-2) {Pitch_Speed=(RC.mouse.y/80.0);}
    TARGET_Speed.Yaw  = -  Yaw_Speed*20.0; 

    TARGET_Angle.Yaw   += Yaw_Speed;
    TARGET_Angle.Pitch += Pitch_Speed;
    /* �͵�������� */
    //1ǹ��

    if(RC.mouse.press_r==1){    //�������Ҽ�ʱ������������ѭ�������ﵽ��������ʱ����Ħ���ֿ���    //Mohs
        mouse_count++;
        if(mouse_count>200){PWMGun=1-PWMGun; mouse_count=0;}
     }
    /* д��ǰ���ٶ� */
    if((1<<0) & RC.key.v && AdvanceSpeed==0){   AdvanceSpeed=660;  }    	//w    
    else if((1<<1) & RC.key.v && AdvanceSpeed==0){  AdvanceSpeed=-660;  }	//s

    /* д��ƽ���ٶ� */
    if((1<<3) & RC.key.v && TranslationSpeed==0){  TranslationSpeed=660;  }  //d	
    else if((1<<2) & RC.key.v && TranslationSpeed==0){  TranslationSpeed=-660; }//a   
    
    /* ��¼����ң�����ͼ���״̬ */    
    /* Ħ���ֿ��� */
    if(PWMGun==1){
        PWM1=1600;
        PWM2=1600;
        GPIO_SetBits(GPIOA, GPIO_Pin_8);
    }
    else{
        PWM1=500;
        PWM2=500;        
        GPIO_ResetBits(GPIOA, GPIO_Pin_8);
    }
       /* �����͵���� */
    if(Shoot==1){
       Set_Shoot_Speed(CAN1,50);  //CAN Mohs
    }
    else{
       Set_Shoot_Speed(CAN1,0);
    }
}



