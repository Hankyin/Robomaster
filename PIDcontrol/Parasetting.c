#include "Parasetting.h"
#include "PIDcontrol.h"
#include "var.h"
#include "stdio.h"
/* 将当前数据写入到flash */
void WriteParameter(void){
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 
	FLASH_EraseSector(PARAMETER_SECTOR,VoltageRange_3);//清除块的falsh
	FLASH_ProgramWord(PARAMETER_ADDR+0,VERIFY_ID);
    
	FLASH_ProgramWord(PARAMETER_ADDR+4,PITCH.MAX);
	FLASH_ProgramWord(PARAMETER_ADDR+8,PITCH.MID);
	FLASH_ProgramWord(PARAMETER_ADDR+12,PITCH.MIN);
    
	FLASH_ProgramWord(PARAMETER_ADDR+16,YAW.MAX);
	FLASH_ProgramWord(PARAMETER_ADDR+20,YAW.MID);
	FLASH_ProgramWord(PARAMETER_ADDR+24,YAW.MIN);
     
	FLASH_Lock();
}
/* 从flash读取数据 */
char ReadParameter(void){
	//读取云台机械角度设置到变量,需要校验存储区
	if(VERIFY_ID==*(u32*)(PARAMETER_ADDR+0)){

	PITCH.MAX =*(u32*)(PARAMETER_ADDR+4);
	PITCH.MID =*(u32*)(PARAMETER_ADDR+8);
	PITCH.MIN =*(u32*)(PARAMETER_ADDR+12);
    
	YAW.MAX=*(u32*)(PARAMETER_ADDR+16);
	YAW.MID=*(u32*)(PARAMETER_ADDR+20);
	YAW.MIN=*(u32*)(PARAMETER_ADDR+24);
        
    CMRotatePID.kp=(float)*(u32*)(PARAMETER_ADDR+28);
	CMRotatePID.ki=(float)*(u32*)(PARAMETER_ADDR+32);
	CMRotatePID.kd=(float)*(u32*)(PARAMETER_ADDR+36);
    
	GMPPositionPID.kp=(float)*(u32*)(PARAMETER_ADDR+40);
	GMPPositionPID.ki=(float)*(u32*)(PARAMETER_ADDR+44);
	GMPPositionPID.kd=(float)*(u32*)(PARAMETER_ADDR+48);
    
	GMPSpeedPID.kp=*(u32*)(PARAMETER_ADDR+52);
	GMPSpeedPID.ki=*(u32*)(PARAMETER_ADDR+56);	
    GMPSpeedPID.kd=*(u32*)(PARAMETER_ADDR+60);
    
    GMYPositionPID.kp=*(u32*)(PARAMETER_ADDR+64);
	GMYPositionPID.ki=*(u32*)(PARAMETER_ADDR+68);
	GMYPositionPID.kd=*(u32*)(PARAMETER_ADDR+72);
  
	GMYSpeedPID.kp=*(u32*)(PARAMETER_ADDR+76);
	GMYSpeedPID.ki=*(u32*)(PARAMETER_ADDR+80);	
    GMYSpeedPID.kd=*(u32*)(PARAMETER_ADDR+84);  

    CM1SpeedPID.kp=*(u32*)(PARAMETER_ADDR+88);
	CM1SpeedPID.ki=*(u32*)(PARAMETER_ADDR+92);	
    CM1SpeedPID.kd=*(u32*)(PARAMETER_ADDR+96);
    
    CM2SpeedPID.kp=*(u32*)(PARAMETER_ADDR+100);
	CM2SpeedPID.ki=*(u32*)(PARAMETER_ADDR+104);	
    CM2SpeedPID.kd=*(u32*)(PARAMETER_ADDR+108);
    
    CM3SpeedPID.kp=*(u32*)(PARAMETER_ADDR+112);
	CM3SpeedPID.ki=*(u32*)(PARAMETER_ADDR+116);	
    CM3SpeedPID.kd=*(u32*)(PARAMETER_ADDR+120);
    
    CM4SpeedPID.kp=*(u32*)(PARAMETER_ADDR+124);
	CM4SpeedPID.ki=*(u32*)(PARAMETER_ADDR+128);	
    CM4SpeedPID.kd=*(u32*)(PARAMETER_ADDR+132);   
    
    printf("PITCH:MAX=%f  MID=%f  MIN=%f",PITCH.MAX,PITCH.MID,PITCH.MIN);
    printf("YAW  :MAX=%f  MID=%f  MIN=%f",YAW.MAX,YAW.MID,YAW.MIN);
    
		return 0;
	}
	return 1;	
}
void WritePIDPara(void)
{
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
    
    FLASH_ProgramWord(PARAMETER_ADDR+28,(u32)CMRotatePID.kp);
	FLASH_ProgramWord(PARAMETER_ADDR+32,(u32)CMRotatePID.ki);
	FLASH_ProgramWord(PARAMETER_ADDR+36,(u32)CMRotatePID.kd);
    
	FLASH_ProgramWord(PARAMETER_ADDR+40,(u32)GMPPositionPID.kp);
	FLASH_ProgramWord(PARAMETER_ADDR+44,(u32)GMPPositionPID.ki);
	FLASH_ProgramWord(PARAMETER_ADDR+48,(u32)GMPPositionPID.kd);
    
	FLASH_ProgramWord(PARAMETER_ADDR+52,GMPSpeedPID.kp);
	FLASH_ProgramWord(PARAMETER_ADDR+56,GMPSpeedPID.ki);	
    FLASH_ProgramWord(PARAMETER_ADDR+60,GMPSpeedPID.kd);
    
    FLASH_ProgramWord(PARAMETER_ADDR+64,GMYPositionPID.kp);
	FLASH_ProgramWord(PARAMETER_ADDR+68,GMYPositionPID.ki);
	FLASH_ProgramWord(PARAMETER_ADDR+72,GMYPositionPID.kd);
  
	FLASH_ProgramWord(PARAMETER_ADDR+76,GMPSpeedPID.kp);
	FLASH_ProgramWord(PARAMETER_ADDR+80,GMPSpeedPID.ki);	
    FLASH_ProgramWord(PARAMETER_ADDR+84,GMPSpeedPID.kd);  

    FLASH_ProgramWord(PARAMETER_ADDR+88,CM1SpeedPID.kp);
	FLASH_ProgramWord(PARAMETER_ADDR+92,CM1SpeedPID.ki);	
    FLASH_ProgramWord(PARAMETER_ADDR+96,CM1SpeedPID.kd);
    
    FLASH_ProgramWord(PARAMETER_ADDR+100,CM2SpeedPID.kp);
	FLASH_ProgramWord(PARAMETER_ADDR+104,CM2SpeedPID.ki);	
    FLASH_ProgramWord(PARAMETER_ADDR+108,CM2SpeedPID.kd);
    
    FLASH_ProgramWord(PARAMETER_ADDR+112,CM3SpeedPID.kp);
	FLASH_ProgramWord(PARAMETER_ADDR+116,CM3SpeedPID.ki);	
    FLASH_ProgramWord(PARAMETER_ADDR+120,CM3SpeedPID.kd);
    
    FLASH_ProgramWord(PARAMETER_ADDR+124,CM4SpeedPID.kp);
	FLASH_ProgramWord(PARAMETER_ADDR+128,CM4SpeedPID.ki);	
    FLASH_ProgramWord(PARAMETER_ADDR+132,CM4SpeedPID.kd); 

    FLASH_Lock();
}


