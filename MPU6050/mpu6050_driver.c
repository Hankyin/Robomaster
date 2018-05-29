#include "mpu6050_driver.h"
#include "mpu6050_i2c.h"
#include "mpu6050_interrupt.h"
#include "delay.h"
#include "stdio.h"
#include "var.h"

MPU6050_RAW_DATA    MPU6050_Raw_Data;
MPU6050_REAL_DATA   MPU6050_Real_Data;
static int gyroADC_X_offset=0,gyroADC_Y_offset=0,gyroADC_Z_offset=0;

//MPU6050 初始化，成功返回0  失败返回 0xff
int MPU6050_Init(void)    
{   
    unsigned char temp_data = 0x00;
    IIC_GPIO_Init();  //初始化IIC接口
    HEAT_Configuration();    
    if(IIC_ReadData(MPU6050_DEVICE_ADDRESS,WHO_AM_I,&temp_data,1)==0) //确定IIC总线上挂接的是否是MPU6050
    {
        if (STATE.Check) printf("%d\r\n",temp_data);
        if(temp_data != MPU6050_ID)
        {
            printf("error 1A\r\n");
            return 0xff; //校验失败，返回0xff
        }
    }
    else
    {
        printf("error 1B\r\n");
        return 0xff; //读取失败 返回0xff
    }
    
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,PWR_MGMT_1,0x01) == 0xff)    //解除休眠状态
    {
        printf("error 1C\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,CONFIG,0x03) == 0xff)         //Digital Low-Pass Filter:DLPF_CFG is 3, Fs is 1khz 
    {                                                                     //acc bandwidth 44Hz,gyro 42Hz
        printf("error 1E\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,GYRO_CONFIG,0x10) == 0xff)    //FS_SEL 3 : gyroscope full scale range is +-1000degs/s 
    {
        printf("error 1F\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,ACCEL_CONFIG,0x00) == 0xff)   //AFS_SEL 1: accelerometer full scale range is +-2g
    {
        printf("error 1G\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_PIN_CFG,0x02) == 0xff)    //logic level for the INT pin is active high
                                                                          //the INT pin emits a 50us long pulse, not latched    bypass mode enabled
    {
        printf("error 1H\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x00) == 0xff)      //disable data ready interrupt
    {
        printf("error 1I\r\n");
        return 0xff;
    }
    //设置mpu6050 IIC masters mode  disabled 不让mpu6050控制aux IIC接口
		if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,MPU6050_RA_USER_CTRL,0x00) == 0xff)      //disable data ready interrupt
    {
        printf("error 1I\r\n");
        return 0xff;
    }
		//设置IIC masters mode 为 bypass mode enabled，在INT_PIN_CFG中配置		
		//5883初始化
//		if(HMC5883_Init() == 0xff)
//		{
//        return 0xff;
//		}
    isopened.Mpu|=(1<<5);
    if (STATE.Check) printf("MPU6050_Init\r\n");
    Delay_ms(500);
    MPU6050_IntConfiguration();
    return 0;
}  
int MPU6050_EnableInt(void)
{
	  if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,SMPLRT_DIV,0x01)==0xff)      //Sample Rate: Gyro output rate / (1 + 1) = 500Hz
	  {
        printf("Cannot enable interrupt successfully.\r\n");
        return 0xff;
	  }
      printf("MPU6050 set sample rate done.\n");
	  Delay_ms(10);
	  if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x01) == 0xff)     //enable data ready interrupt 
    {
      printf("error 1I\r\n");
      return 0xff;
    } 
    isopened.Mpu|=(1<<6);
    if (STATE.Check) printf("MPU6050_EnableInt\r\n");
	return 0;
}

//MPU6050  数据读取，成功返回0  失败返回 0xff
int MPU6050_ReadData(u8 Device)
{
    u8 buf[14];    
    if(IIC_ReadData(Device,MPU6050_DATA_START,buf,14) == 0xff)
    {
        printf("error 1J\r\n");
        return 0xff;
    }
    else
    {   //读取寄存器原生数据           
	    MPU6050_Raw_Data.Accel_X = (buf[0]<<8 | buf[1]);
	    MPU6050_Raw_Data.Accel_Y = (buf[2]<<8 | buf[3]);
	    MPU6050_Raw_Data.Accel_Z = (buf[4]<<8 | buf[5]); 
	    MPU6050_Raw_Data.Temp =    (buf[6]<<8 | buf[7]);  
	    MPU6050_Raw_Data.Gyro_X = (buf[8]<<8 | buf[9]);
	    MPU6050_Raw_Data.Gyro_Y = (buf[10]<<8 | buf[11]);
	    MPU6050_Raw_Data.Gyro_Z = (buf[12]<<8 | buf[13]);
	    //将原生数据转换为实际值，计算公式跟寄存器的配置有关
	    MPU6050_Real_Data.Accel_X = -(float)(MPU6050_Raw_Data.Accel_X)/8192.0f; //见datasheet 30 of 47
	    MPU6050_Real_Data.Accel_Y = -(float)(MPU6050_Raw_Data.Accel_Y)/8192.0f; //见datasheet 30 of 47
	    MPU6050_Real_Data.Accel_Z = (float)(MPU6050_Raw_Data.Accel_Z)/8192.0f; //见datasheet 30 of 47
	    MPU6050_Real_Data.Temp =   (float)(MPU6050_Raw_Data.Temp)/340.0f+36.53f;//见datasheet 31 of 47
	    MPU6050_Real_Data.Gyro_X = -(float)(MPU6050_Raw_Data.Gyro_X - gyroADC_X_offset)/65.5f;     //见datasheet 32 of 47
	    MPU6050_Real_Data.Gyro_Y = -(float)(MPU6050_Raw_Data.Gyro_Y - gyroADC_Y_offset)/65.5f;     //见datasheet 32 of 47
	    MPU6050_Real_Data.Gyro_Z = (float)(MPU6050_Raw_Data.Gyro_Z - gyroADC_Z_offset)/65.5f;     //见datasheet 32 of 47
    }     
    return 0;
}
void MPU6050_Gyro_calibration(u8 Device)
{
	u16 i;
	float x_temp=0,y_temp=0,z_temp=0;
		for(i=0; i<1000; i++)
		{
			MPU6050_ReadData(Device);
			delay_ms(5);
			x_temp=x_temp+MPU6050_Raw_Data.Gyro_X;
			y_temp=y_temp+MPU6050_Raw_Data.Gyro_Y;
			z_temp=z_temp+MPU6050_Raw_Data.Gyro_Z;
		}			
		
		x_temp=x_temp/1000.00f;
		y_temp=y_temp/1000.00f;	
		z_temp=z_temp/1000.00f;
	
		gyroADC_X_offset=(int)x_temp;
		gyroADC_Y_offset=(int)y_temp;
		gyroADC_Z_offset=(int)z_temp;
}
