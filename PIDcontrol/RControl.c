#include "var.h"
#include "gun.h"
#include "can1.h"
#include "delay.h"
#include "canbus.h"
#include "RControl.h"
#include "stdio.h"
    /* 遥控器数据部分 */
RC_Ctl_t RC;
static int StartCheckCount=0;	 //数据校验计数，启动时进行值校验，如果摇杆没有归位，则认为发生错误，等待数据直到归位
void RC_DataProcess(unsigned char* sbus_rx_buffer){
    /* 判断是否允许遥控器控制 */
    if(STATE.usartRC==DISABLE)return;
    /* 接收到遥控器信号标记 */
    /* 取得控制数据 */    
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
    
    /* 启动时进行数据检查，防止DMA错位或者误碰遥控器造成的启动后疯跑 */

//    if(StartCheckCount<5){//5次校验通过才认为遥控器正常
//        if(RC.rc.ch0!=1024 ||RC.rc.ch1!=1024 ||RC.rc.ch2!=1024 || RC.rc.ch3!=1024 || RC.rc.s2!=3 || RC.rc.s1!=3){
//            StartCheckCount=0;
//            while(DMA1_Stream5->NDTR>17);//解决错位问题，保证每次启动遥控器和车都能匹配数据位
//            DMA1_Stream5->NDTR=18;
//        }
//        else{
//            StartCheckCount++;
//        }
//        return;//校验成功之前不进行后续操作
//    }
    RC_Control(RC);
}
    /* 遥控器功能部分 */
float Pitch_Speed=0;	 	 //Pitch速度
float Yaw_Speed=0;		 	 //Yaw速度
static unsigned char PWMGun=0;	 //摩擦轮开关
unsigned char Shoot=0;	 	 //送弹1开关
int mouse_count=0;
void RC_Control(RC_Ctl_t RC) 
{
    TranslationSpeed = (RC.rc.ch0-1024);	 	 //平移
    AdvanceSpeed = (RC.rc.ch1-1024);		 	 //前后
    Yaw_Speed = -(RC.rc.ch2 - 1024) * 0.005f;	 	 	 //云台速度
    Pitch_Speed = -(RC.rc.ch3 - 1024) * 0.0025f;
    
    /*开关1*/
    if(RC.rc.last_s1!=1 && RC.rc.s1==1){  //切换了开关1的1功能状态
        PWMGun=1-PWMGun;				   //开关摩擦轮
    }
    RC.rc.last_s1=RC.rc.s1;  
    
    
    if( RC.rc.s1==2 ){Shoot=1;} 
    else if(PWMGun==1 && RC.mouse.press_l==1){Shoot=1;}
    else {Shoot=0;}        	  //当在摩擦轮关闭时就会关闭送弹} 
    
    /* 开关2 */
    if(RC.rc.s2==1){//切换了开关1的1功能状态
        STATE.Run=NORMAL_STATE;		 
    }
    if(RC.rc.s2==3){//切换了开关1的1功能状态
        STATE.Run=PAUSE_STATE;		 
    }
    if(RC.rc.s2==2){//切换了开关1的1功能状态
        STATE.Run=STOP_STATE;		 
    } 
    RC.rc.last_s2=RC.rc.s2;
    /* 键盘鼠标功能部分 */
    if(RC.mouse.x>2||RC.mouse.x<-2) {Yaw_Speed=-(RC.mouse.x/80.0); }	 //鼠标优先于摇杆
    if(RC.mouse.y>2||RC.mouse.y<-2) {Pitch_Speed=(RC.mouse.y/80.0);}
    TARGET_Speed.Yaw  = -  Yaw_Speed*20.0; 

    TARGET_Angle.Yaw   += Yaw_Speed;
    TARGET_Angle.Pitch += Pitch_Speed;
    /* 送弹电机控制 */
    //1枪管

    if(RC.mouse.press_r==1){    //当按下右键时，会掉入计数死循环，当达到计数次数时出发摩擦轮开关    //Mohs
        mouse_count++;
        if(mouse_count>200){PWMGun=1-PWMGun; mouse_count=0;}
     }
    /* 写入前进速度 */
    if((1<<0) & RC.key.v && AdvanceSpeed==0){   AdvanceSpeed=660;  }    	//w    
    else if((1<<1) & RC.key.v && AdvanceSpeed==0){  AdvanceSpeed=-660;  }	//s

    /* 写入平移速度 */
    if((1<<3) & RC.key.v && TranslationSpeed==0){  TranslationSpeed=660;  }  //d	
    else if((1<<2) & RC.key.v && TranslationSpeed==0){  TranslationSpeed=-660; }//a   
    
    /* 记录当次遥控器和键盘状态 */    
    /* 摩擦轮控制 */
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
       /* 控制送弹电机 */
    if(Shoot==1){
       Set_Shoot_Speed(CAN1,50);  //CAN Mohs
    }
    else{
       Set_Shoot_Speed(CAN1,0);
    }
}



