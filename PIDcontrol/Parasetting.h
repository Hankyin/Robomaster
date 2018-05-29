#ifndef _PARA_SETTING_H_
#define _PARA_SETTING_H_

#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"

//ע��m4��Flash�ÿ黮�� 
//	((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
//	((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */ 
//	((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */ 
//	((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */ 
//	((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */ 
//	((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */ 
//	((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */ 
//	((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */ 
//	((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */ 
//	((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
//	((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */ 
//	((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */ 

//  ����������ַ���� 
#define PARAMETER_ADDR		0x080E0000
#define PARAMETER_SECTOR	FLASH_Sector_11
#define VERIFY_ID			0x55aa55aa	  		//��ַУ��
/* ����ǰ����д�뵽flash */
void WriteParameter(void);
/* ��flash��ȡ���� */
char ReadParameter(void);
/*PID���ú��� */
void WritePIDPara(void);

#endif


