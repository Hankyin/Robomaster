#include "Errcontrol.h"
#include "PIDcontrol.h"
#include "Parasetting.h"
#include "canbus.h"
#include "var.h"
#include "usart3.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>

char CMD_buf[64];
int CMD_count=0;
int CMD[2]={0,0};
void USART_CMDProess(char res){
    //USART3_SendChar(res);
    if(res==' '||res=='\t'||res=='\r'||res=='\n')
    {
        CMD_buf[CMD_count]='\0';
        if((CMD_buf[0]<='9' && CMD_buf[0]>='0')|| CMD_buf[0]=='-'||CMD_buf[0]=='.'){
        switch (CMD[0]){
            case 1:{    //pitchposi     kp  ki  kd
                switch (CMD[1]){                    
                    case 2:{GMPPositionPID.kp=atof(CMD_buf);printf("kp=%f\r\n",GMPPositionPID.kp);}break;
                    case 3:{GMPPositionPID.ki=atof(CMD_buf);printf("ki=%f\r\n",GMPPositionPID.ki);}break;
                    case 4:{GMPPositionPID.kd=atof(CMD_buf);printf("kd=%f\r\n",GMPPositionPID.kd);}break;    
                        default:;
                }  
            }break; 
            case 2:{    //pitchspeed     kp  ki  kd
                switch (CMD[1]){                    
                    case 2:{GMPSpeedPID.kp=atof(CMD_buf);printf("kp=%f\r\n",GMPSpeedPID.kp);}break;
                    case 3:{GMPSpeedPID.ki=atof(CMD_buf);printf("ki=%f\r\n",GMPSpeedPID.ki);}break;
                    case 4:{GMPSpeedPID.kd=atof(CMD_buf);printf("kd=%f\r\n",GMPSpeedPID.kd);}break;  
                        default:;
                }  
            }break;
            case 3:{    //yawposi     kp  ki  kd
                switch (CMD[1]){                    
                    case 2:{GMYPositionPID.kp=atof(CMD_buf);printf("kp=%f\r\n",GMYPositionPID.kp);}break;
                    case 3:{GMYPositionPID.ki=atof(CMD_buf);printf("ki=%f\r\n",GMYPositionPID.ki);}break;
                    case 4:{GMYPositionPID.kd=atof(CMD_buf);printf("kd=%f\r\n",GMYPositionPID.kd);}break;
                        default:;
                }  
            }break;
            case 4:{    //yawspeed     kp  ki  kd
                switch (CMD[1]){                    
                    case 2:{GMYSpeedPID.kp=atof(CMD_buf);printf("kp=%f\r\n",GMYSpeedPID.kp);}break;
                    case 3:{GMYSpeedPID.ki=atof(CMD_buf);printf("ki=%f\r\n",GMYSpeedPID.ki);}break;
                    case 4:{GMYSpeedPID.kd=atof(CMD_buf);printf("kd=%f\r\n",GMYSpeedPID.kd);}break; 
                    default:;
                }  
            }break;
                    default:;
             
        }
        CMD_count=0;
        return;
        }
        switch(CMD_count){
            
            case 7:{    //YAWPOSI
                CMD[0]=3;printf("\\YAWPOSI\r\n");
            }break;
            case 8:{    //YAWSPEED
                CMD[0]=4;printf("\\YAWSPEED:\r\n"); 
            }break;                                                     
            case 9:{    //PITCHPOSI
                CMD[0]=1;printf("\\PITCHPOSI:\r\n");
            }break;
            case 10:{    //PITCHSPEED
                CMD[0]=2;printf("\\PITCHSPEED:\r\n");
            }break;
            case 2:{
                if(CMD_buf[1]=='p') CMD[1]=2,printf("\\kp:\r\n");            //kp
                else if(CMD_buf[1]=='i') CMD[1]=3,printf("\\ki:\r\n");       //ki
                else if(CMD_buf[1]=='d') CMD[1]=4,printf("\\kd:\r\n");       //kd            
            }break;
            case 4:{   //SHOW
                switch(CMD[0]){
                    case 1:printf("GMPP:\tkp=%f\tki=%f\tkd=%f\r\n",GMPPositionPID.kp,GMPPositionPID.ki,GMPPositionPID.kd);break;
                    case 2:printf("GMPS:\tkp=%f\tki=%f\tkd=%f\r\n",GMPSpeedPID.kp,GMPSpeedPID.ki,GMPSpeedPID.kd);break;
                    case 3:printf("GMYP:\tkp=%f\tki=%f\tkd=%f\r\n",GMYPositionPID.kp,GMYPositionPID.ki,GMYPositionPID.kd);break;
                    case 4:printf("GMYS:\tkp=%f\tki=%f\tkd=%f\r\n",GMYSpeedPID.kp,GMYSpeedPID.ki,GMYSpeedPID.kd);break;
                    default:;                    
                }
            }break;
            case 5:{   //CLEAR
                switch(CMD[0]){
                    case 1:printf("\\Clear GMPP:\r\n");GMPPositionPID.ERR_Inte=0,GMPPositionPID.ERR_last=0;break;
                    case 2:printf("\\Clear GMPS:\r\n");GMPSpeedPID.ERR_Inte=0,   GMPSpeedPID.ERR_last=0;   break;
                    case 3:printf("\\Clear GMYP:\r\n");GMYPositionPID.ERR_Inte=0,GMYPositionPID.ERR_last=0;break;
                    case 4:printf("\\Clear GMYS:\r\n");GMYSpeedPID.ERR_Inte=0,   GMYSpeedPID.ERR_last=0;   break;
                    default:;                    
                }
            }break;
            default:;            
        }
    CMD_count=0;        
    }
    else{CMD_buf[CMD_count++]=res;}
}

char CM_buf[64];
int CM_count=0;
int cmflag=2;
void Usart_CM(char res){
    //USART3_SendChar(res);
    if(res==' '||res=='\t'||res=='\r'||res=='\n')
    {
        CM_buf[CM_count]='\0';
        if((CM_buf[0]<='9' && CM_buf[0]>='0')|| CM_buf[0]=='-'||CM_buf[0]=='.'){
        switch (cm){
            case 1:{
                switch (cmflag){
                    case 1:{CMSpeed[0]=atof(CM_buf);printf("speed=%f\r\n",CMSpeed[0]);isprint.CM=cm;}break;
                    case 2:{CM1SpeedPID.kp=atof(CM_buf);printf("kp=%f\r\n",CM1SpeedPID.kp);}break;
                    case 3:{CM1SpeedPID.ki=atof(CM_buf);printf("ki=%f\r\n",CM1SpeedPID.ki);}break;
                    case 4:{CM1SpeedPID.kd=atof(CM_buf);printf("kd=%f\r\n",CM1SpeedPID.kd);}break;    
                        default:;
                }  
            }break;
            case 2:{
                switch (cmflag){
                    case 1:{CMSpeed[1]=atof(CM_buf);printf("speed=%f\r\n",CMSpeed[1]);isprint.CM=cm;}break;
                    case 2:{CM2SpeedPID.kp=atof(CM_buf);printf("kp=%f\r\n",CM2SpeedPID.kp);}break;
                    case 3:{CM2SpeedPID.ki=atof(CM_buf);printf("ki=%f\r\n",CM2SpeedPID.ki);}break;
                    case 4:{CM2SpeedPID.kd=atof(CM_buf);printf("kd=%f\r\n",CM2SpeedPID.kd);}break;  
                        default:;
                }  
            }break;
            case 3:{
                switch (cmflag){
                    case 1:{CMSpeed[2]=atof(CM_buf);printf("speed=%f\r\n",CMSpeed[2]);isprint.CM=cm;}break;
                    case 2:{CM3SpeedPID.kp=atof(CM_buf);printf("kp=%f\r\n",CM3SpeedPID.kp);}break;
                    case 3:{CM3SpeedPID.ki=atof(CM_buf);printf("ki=%f\r\n",CM3SpeedPID.ki);}break;
                    case 4:{CM3SpeedPID.kd=atof(CM_buf);printf("kd=%f\r\n",CM3SpeedPID.kd);}break;
                        default:;
                }  
            }break;
            case 4:{
                switch (cmflag){
                    case 1:{CMSpeed[3]=atof(CM_buf);printf("speed=%f\r\n",CMSpeed[3]);isprint.CM=cm;}break;
                    case 2:{CM4SpeedPID.kp=atof(CM_buf);printf("kp=%f\r\n",CM4SpeedPID.kp);}break;
                    case 3:{CM4SpeedPID.ki=atof(CM_buf);printf("ki=%f\r\n",CM4SpeedPID.ki);}break;
                    case 4:{CM4SpeedPID.kd=atof(CM_buf);printf("kd=%f\r\n",CM4SpeedPID.kd);}break; 
                    default:;
                }  
            }break;
                    default:;             
        }
        CM_count=0;
        return;
        }
        switch(CM_count){
            case 3:{cm=(int)(CM_buf[2]-'0');printf("\\CM=%d\r\n",cm);}break; //CMx
            case 5:{
                cmflag=1;printf("\\Speed:\r\n");                
            }break;                                                     //Speed
            case 2:{
                if(CM_buf[1]=='p') cmflag=2,printf("\\kp:\r\n");            //kp
                else if(CM_buf[1]=='i') cmflag=3,printf("\\ki:\r\n");       //ki
                else if(CM_buf[1]=='d') cmflag=4,printf("\\kd:\r\n");       //kd
            }break;
            case 4:{
                printf("CM:\tspeed=%f\t%f\t%f\t%f\r\n",CMSpeed[0],CMSpeed[1],CMSpeed[2],CMSpeed[3]);
                switch(cm){
                    case 1:printf("CM1:\tkp=%f\tki=%f\tkd=%f\r\n",CM1SpeedPID.kp,CM1SpeedPID.ki,CM1SpeedPID.kd);break;
                    case 2:printf("CM2:\tkp=%f\tki=%f\tkd=%f\r\n",CM2SpeedPID.kp,CM2SpeedPID.ki,CM2SpeedPID.kd);break;
                    case 3:printf("CM3:\tkp=%f\tki=%f\tkd=%f\r\n",CM3SpeedPID.kp,CM3SpeedPID.ki,CM3SpeedPID.kd);break;
                    case 4:printf("CM4:\tkp=%f\tki=%f\tkd=%f\r\n",CM4SpeedPID.kp,CM4SpeedPID.ki,CM4SpeedPID.kd);break;
                    default:;                    
                }
            }break;
            default:;            
        }
    CM_count=0;        
    }
    else{CM_buf[CM_count++]=res;}
}
            
char GM_buf[64];
int GM_count=0;
void Usart_MAX(char res){
    //USART3_SendChar(res);
    if(res==' '||res=='\t'||res=='\r'||res=='\n')
    {
        GM_buf[GM_count]='\0';
        if(GM_count==4) isprint.GM=1;        
        GM_count=0;        
    }
    else{GM_buf[GM_count++]=res;}
}
        
