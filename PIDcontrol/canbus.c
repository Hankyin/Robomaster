#include "canbus.h"
#include "var.h"
#include "stdio.h"

volatile Encoder CM1Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM2Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM3Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM4Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMYawEncoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMPitchEncoder = {0,0,0,0,0,0,0,0,0};
/****************************************************************************************
*
*Name          :EncoderProcess
*Input         :can message
*Return        :void
*Description   :to get the initiatial encoder of the chassis motor 201 202 203 204
*
****************************************************************************************/
int filter=0;
void EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{    
    v->raw_value =(msg->Data[0]<<8)|msg->Data[1];
    filter =(int16_t)((msg->Data[2]<<8)|msg->Data[3]);
    v->filter_rate=filter/27.0;
    if(STATE.CM==1) printf("CM_speed=%f\r\n",v->filter_rate); 
}
int j=0;
void GMEncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{	int i=0;    

    int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
    
    if (isopened.GM==0&&v->raw_value>4096) {v->round_cnt=0;isopened.GM=1;}
    else if (isopened.GM==0&&v->raw_value<4096) {v->round_cnt=1;isopened.GM=1;}
    else;
    
    isopened.GM=1;
	v->diff = v->raw_value - v->last_raw_value;
 	if(v->diff < -7500)    //两次编码器的反馈值差别太大，表示圈数发生了改变，即日界线的问题 Mohs
	{
		v->round_cnt=1;     
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>7500)
    {
		v->round_cnt=0;
		v->ecd_raw_rate = v->diff - 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
    //计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大   0~2*8196
	v->ecd_angle = (float)(v->raw_value)*360/8192 + v->round_cnt * 360;
    //循环式的压入滤波数组 Mohs
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
    if(v->buf_count >= RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE); 
}

void CAN2USART( CanRxMsg * msg)
{
    printf("DATA[01]=%d\r\n",(msg->Data[0]<<8)|msg->Data[1]);
    printf("DATA[23]=%d\r\n",(msg->Data[2]<<8)|msg->Data[3]);
    printf("DATA[45]=%d\r\n",(msg->Data[4]<<8)|msg->Data[5]);
    printf("DATA[67]=%d\r\n",(msg->Data[6]<<8)|msg->Data[7]);
}
/*
************************************************************************************************************************
*Name        : CanReceiveMsgProcess
* Description: This function process the can message representing the encoder data received from the CAN2 bus.
* Arguments  : msg     is a pointer to the can message.
* Returns    : void
* Note(s)    : none
************************************************************************************************************************
*/
void CanReceiveMsgProcess(CanRxMsg * msg)
{      
	switch(msg->StdId)
	{
		case CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID:
		{
			EncoderProcess(&CM1Encoder ,msg);       //获取到编码器的位置和速度      
		}break;
		case CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID:
		{					
			EncoderProcess(&CM2Encoder ,msg);
		}break;
		case CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID:
		{					
			EncoderProcess(&CM3Encoder ,msg);   
		}break;
		case CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID:
        {
			EncoderProcess(&CM4Encoder ,msg);
		}break;
		case CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID:
		{
            GMEncoderProcess(&GMYawEncoder,msg); 
 //           if(isprint.GM==1)printf("YawEnc=%d\r\n",GMYawEncoder.ecd_value);
        }break;
		case CAN_BUS2_MOTOR6_FEEDBACK_MSG_ID:
		{
            GMEncoderProcess(&GMPitchEncoder,msg);
 //           if(isprint.GM==1)printf("PitchEnc=%d\r\n",GMPitchEncoder.ecd_value);
		}break;				
		default:{ if(STATE.Check)CAN2USART(msg);}
    }
}

/********************************************************************************
   给底盘电调板发送指令，ID号为0x200８底盘返回ID为0x201-0x204
*********************************************************************************/
void Set_CM_Speed(CAN_TypeDef* CANx,int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm1_iq;
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    CAN_Transmit(CANx,&tx_message);
    if (STATE.Check) printf("SetCMSpeed(%d,%d,%d,%d)\r",cm1_iq, cm2_iq, cm3_iq, cm4_iq);
    
}
/********************************************************************************
   给电调板发送指令，ID号为0x1FF，只用两个电调板，数据回传ID为0x205和0x206
	 cyq:更改为发送三个电调的指令。
*********************************************************************************/
void Set_Gimbal_Current(CAN_TypeDef* CANx,int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq)
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (unsigned char)(gimbal_yaw_iq >> 8);
    tx_message.Data[1] = (unsigned char)gimbal_yaw_iq;
    tx_message.Data[2] = (unsigned char)(gimbal_pitch_iq >> 8);
    tx_message.Data[3] = (unsigned char)gimbal_pitch_iq;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
    if (STATE.Check) printf("SetGimbal(%d,%d)\r",gimbal_yaw_iq, gimbal_pitch_iq);
}


