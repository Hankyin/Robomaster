#ifndef _RCONTROL_H_
#define _RCONTROL_H_

#include <stm32f4xx.h>

#define PITCH_MAX 19.0f
#define YAW_MAX 720.0f  //720.0				//cyq:ÔÆÌ¨½Ç¶ÈµÄ·¶Î§

/* ----------------------- Data Struct ------------------------------------- */
typedef struct
{
	struct
	{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
        uint8_t last_s1;
        uint8_t last_s2;
		uint8_t s1;
		uint8_t s2;
	}rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
        uint8_t last_press_l;
        uint8_t last_press_r;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	struct
	{
		uint16_t v;
	}key;
}RC_Ctl_t ;

void RC_DataProcess(unsigned char* sbus_rx_buffer);
void RC_Control(RC_Ctl_t RC);

#endif
