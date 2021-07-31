#include "usart.h"
#include <stdio.h>
#include "host.h"

#define OTIME 10
#define HUART huart3

uint8_t sendBUF[100];

void HostSend( u8 d_addr, u8 id, u8 len, u8* data)
{
	
	uint8_t sumcheck = 0, addcheck = 0, cnt = 0;
	
	sendBUF[cnt++] = 0xAA;
	sendBUF[cnt++] = d_addr;
	sendBUF[cnt++] = id;
	sendBUF[cnt++] = len;
	
	sumcheck += 0xAA;
	addcheck += sumcheck;
	
	sumcheck += d_addr;
	addcheck += sumcheck;
	
	sumcheck += id;
	addcheck += sumcheck;
	
	sumcheck += len;
	addcheck += sumcheck;
	
	for(uint8_t i=0;i < len;i++)
	{
		sendBUF[i+4] = data[i];
		sumcheck += data[i];
		addcheck += sumcheck;
	}
	
	sendBUF[len+4] = sumcheck;
	sendBUF[len+5] = addcheck;
	
	HAL_UART_Transmit(&HUART, sendBUF, len+6, OTIME);
	
}


void HostSendLog(u8 color, const char* str)
{
	u8 cnt = 0, sc = 0, ac = 0, len = 0;
	sendBUF[cnt] = 0xAA;	sc += sendBUF[cnt++];	ac += sc;
	sendBUF[cnt] = 0xFF;	sc += sendBUF[cnt++];	ac += sc;
	sendBUF[cnt] = 0xA0;	sc += sendBUF[cnt++];	ac += sc;
	cnt++; //留空len位
	sendBUF[cnt++] = color;
	
	
	while(*(str+len) != '\0' && len < 80)
	{
		sendBUF[cnt++] = *(str+len);
		len++;
	}
	
	sendBUF[3] = len + 1;
	
	for(u8 i = 3;i < len+5;i++)
	{
		sc += sendBUF[i];
		ac += sc;
	}
	
	sendBUF[cnt++] = sc;
	sendBUF[cnt++] = ac;
	
	HAL_UART_Transmit(&HUART, sendBUF, cnt, OTIME);
	
}

void HostSendLogVal(const char* str, uint32_t val)
{
	u8 cnt = 0, sc = 0, ac = 0, len = 0;
	sendBUF[cnt] = 0xAA;	sc += sendBUF[cnt++];	ac += sc;
	sendBUF[cnt] = 0xFF;	sc += sendBUF[cnt++];	ac += sc;
	sendBUF[cnt] = 0xA1;	sc += sendBUF[cnt++];	ac += sc;
	cnt++; //留空len位
	sendBUF[cnt++] = *((char *)&val); //0# 第0位
	sendBUF[cnt++] = *((char *)&val + 1); //1#
	sendBUF[cnt++] = *((char *)&val + 2); //2#
	sendBUF[cnt++] = *((char *)&val + 3); //3#
	
	while(*(str+len) != '\0' && len < 80)
	{
		sendBUF[cnt++] = *(str+len);
		len++;
	}
	
	sendBUF[3] = len + 4;
	
	for(u8 i = 3;i < cnt;i++)
	{
		sc += sendBUF[i];
		ac += sc;
	}
	
	sendBUF[cnt++] = sc;
	sendBUF[cnt++] = ac;
	
	HAL_UART_Transmit(&HUART, sendBUF, cnt, OTIME);
	
}

void HostSendData(u8 id, struct DataPack* data, u8 n)
{
	u8 cnt = 0, sc = 0, ac = 0, len = 0;
	sendBUF[cnt++] = 0xAA;
	sendBUF[cnt++] = 0xFF;
	sendBUF[cnt++] = id;
	cnt++;
	
	for(u8 i=0;i<n;i++)
	{
		switch(data[i].type)
		{
			case DATA_TYPE_U8:
				len += 1;
				sendBUF[cnt++] = data[i].val.ui8;
				break;
			case DATA_TYPE_U16:
				len += 2;
				sendBUF[cnt++] = data[i].val.ui16 % 256;
			sendBUF[cnt++] = data[i].val.ui16 / 256;
				break;
			case DATA_TYPE_S16:
				len += 2;
				sendBUF[cnt++] = data[i].val.ui16 % 256;
				sendBUF[cnt++] = data[i].val.ui16 / 256;
				break;
		}
	}
	sendBUF[3] = len;
	
	for(u8 i=0;i<cnt;i++)
	{
		sc += sendBUF[i];
		ac += sc;
	}
	
	sendBUF[cnt++] = sc;
	sendBUF[cnt++] = ac;
	
	HAL_UART_Transmit(&HUART, sendBUF, cnt, OTIME);
	
	
}

