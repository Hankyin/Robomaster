#ifndef __USART1_H__
#define __USART1_H__
#include "stm32f4xx.h"

/*
*********************************************************************************************************
*                                               MACROS
*********************************************************************************************************
*/

#define  BSP_USART1_DMA_RX_BUF_LEN               30u                   
#define  RC_FRAME_LENGTH                            18u 

/*
*********************************************************************************************************
*                                             FUNCTION PROTOTYPES
*********************************************************************************************************
*/
void USART1_IRQHandler(void);
static void USART1_FIFO_Init(void);
void *USART1_GetRxBuf(void);
void USART1_Configuration(uint32_t baud_rate);

//extern uint8_t USART1_RX_BUF[BUF_Size];

#endif
