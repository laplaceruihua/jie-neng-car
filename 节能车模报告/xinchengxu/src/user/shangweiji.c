#include "shangweiji.h"
#include "common.h"
#include "uart.h"
#define UartDataNum      17
uint8  uSendBuf[UartDataNum*2]={0};
uint8 FreeCarsDataNum=UartDataNum*2;

void push(uint8 chanel,uint16 data)
{
     uSendBuf[chanel*2]=data/256;
     uSendBuf[chanel*2+1]=data%256;
}

void sendDataToScope(void)
{
  uint8 i,sum=0;
  Uart_SendChar(UARTR2,251);
  Uart_SendChar(UARTR2,109);
  Uart_SendChar(UARTR2,37);
  sum+=(251);      
  sum+=(109);
  sum+=(37);
   for(i=0;i<FreeCarsDataNum;i++)
  {
    Uart_SendChar(UARTR2,uSendBuf[i]);
    sum+=uSendBuf[i];         
  }
  Uart_SendChar(UARTR2,sum);
}