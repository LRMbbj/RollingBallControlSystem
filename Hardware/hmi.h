#ifndef __HMI__
#define __HMI__

#include "usart.h"

#define u8 uint8_t
#define u32 uint32_t

#define UART                   huart1
#define BUF_MAX_SIZE           200

extern u8 HMI_DATA_BUF[BUF_MAX_SIZE];
extern u8 HMI_DATA_BUF_SIZE;

void HMI_Init(void); //初始化HMI
u8 HMIGetOrder(void);
void HMIGetData(u8 *pData);
u32 HMIReadInt(void);
void ClearBUF(void);
void HMISendOrder( char*, char*);
void HMISetValue( char*, char*);

#endif
