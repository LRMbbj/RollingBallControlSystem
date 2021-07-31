#include "hmi.h"
#include "usart.h"

u8 HMI_DATA_BUF[BUF_MAX_SIZE];
u8 HMI_DATA_BUF_SIZE = 0;

void HMI_Init(void)
{

}

void HMI_puts(char *str)
{
    u8 j=0;
    while (*(str+j))
    {
        HAL_UART_Transmit(&UART, (u8*)str + j, 1, 10);
        j++;
    }
}

void HMISetValue(char *obj,char *val)
{
    
    HMI_puts(obj);
    HMI_puts("=\"");
    HMI_puts(val);
    HMI_puts("\"\xff\xff\xff");
    
}

void HMISendOrder(char *obj,char *val)
{
    
    HMI_puts(obj);
    HMI_puts(" ");
    HMI_puts(val);
    HMI_puts("\xff\xff\xff");
    
}


u32 HMIReadInt()
{
    ClearBUF();
    while( HMI_DATA_BUF_SIZE != 4 );
    HMI_DATA_BUF_SIZE = 0;
    return HMI_DATA_BUF[0] + 
                256 * HMI_DATA_BUF[1] + 
                65536 * HMI_DATA_BUF[2] + 
                16777216 * HMI_DATA_BUF[3];
    
}

u8 HMIGetOrder()
{
    ClearBUF();
    HAL_UART_Receive_IT(&UART, &hmi.byteBUF, 1);
    while (HMI_DATA_BUF[0] != 'O' || HMI_DATA_BUF[1] != 'D' || *hmi.psize != 3);    
    return HMI_DATA_BUF[2];
}

void ClearBUF()
{
    while( HMI_DATA_BUF_SIZE > 0 )
    {
        HMI_DATA_BUF_SIZE--;
        HMI_DATA_BUF[HMI_DATA_BUF_SIZE] = 0;
    }
}



void HMIGetData(u8 *pData) // 首位为长度
{
    ClearBUF();
    while (hmi.state != 0 || *hmi.psize < 1);
    pData[0] = *hmi.psize;
    for (u8 i = 1; i <= pData[0]; i++)
    {
        pData[i] = HMI_DATA_BUF[i - 1];
    }
}
