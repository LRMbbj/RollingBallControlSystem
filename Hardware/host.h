#ifndef __HOST_H__
#define __HOST_H__

#include "main.h"

#define LOG_COLOR_BLACK 0
#define LOG_COLOR_RED 1
#define LOG_COLOR_GREEN 2

#define DATA_TYPE_U8 0
#define DATA_TYPE_U16 1
#define DATA_TYPE_S16 2
#define DATA_TYPE_U32 3
#define DATA_TYPE_S32 4


typedef uint8_t u8;

union Data
{
	uint8_t ui8;
	uint16_t ui16;
	int16_t i16;
};

union DataL
{
	uint32_t ui32;
	int32_t i32;
};

struct DataPack
{
	u8 type;
	union Data val;
};

void HostSend( u8 d_addr, u8 id, u8 len, u8 data[]);
void HostSendLog(u8 color, const char* str);
void HostSendLogVal(const char* str, uint32_t val);
void HostSendData(u8 id, struct DataPack* data, u8 n);


#endif
